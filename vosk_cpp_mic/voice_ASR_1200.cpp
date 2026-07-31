#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <vosk_api.h>
#include <locale.h>
#include <audio>

#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

using namespace std::experimental;

// Align with Python test_microphone.py:
//   - mono int16 PCM
//   - sample rate = device rate (also passed to recognizer)
//   - queue audio from callback -> main thread AcceptWaveform
// Flow: press S to start recording, ESC to stop, then print final result.

namespace {

constexpr const char* kDefaultModelPath = "model/vosk-model-cn-kaldi-multicn-0.15";

std::mutex g_queue_mutex;
std::queue<std::vector<int16_t>> g_pcm_queue;
std::atomic<bool> g_recording{false};
std::atomic<int> g_input_channels{1};

bool is_default_device(const audio_device& d)
{
    if (d.is_input())
    {
        auto default_in = get_default_audio_input_device();
        return default_in.has_value() && d.device_id() == default_in->device_id();
    }
    else if (d.is_output())
    {
        auto default_out = get_default_audio_output_device();
        return default_out.has_value() && d.device_id() == default_out->device_id();
    }

    return false;
}

void print_device_info(const audio_device& d)
{
    std::cout << "- \"" << d.name() << "\", ";
    std::cout << "sample rate = " << d.get_sample_rate() << " Hz, ";
    std::cout << "buffer size = " << d.get_buffer_size_frames() << " frames, ";
    std::cout << (d.is_input() ? d.get_num_input_channels() : d.get_num_output_channels())
              << " channels";
    std::cout << (is_default_device(d) ? " [DEFAULT DEVICE]\n" : "\n");
}

void print_device_list(const audio_device_list& list)
{
    for (auto& item : list)
    {
        print_device_info(item);
    }
}

void print_all_devices()
{
    std::cout << "Input devices:\n==============\n";
    print_device_list(get_audio_input_device_list());

    std::cout << "\nOutput devices:\n===============\n";
    print_device_list(get_audio_output_device_list());
}

void clear_pcm_queue()
{
    std::lock_guard<std::mutex> lock(g_queue_mutex);
    std::queue<std::vector<int16_t>> empty;
    g_pcm_queue.swap(empty);
}

// Convert float interleaved input to mono int16 (same as Python dtype=int16, channels=1).
void callback(audio_device&, audio_device_io<float>& io) noexcept
{
    if (!io.input_buffer.has_value() || !g_recording.load(std::memory_order_relaxed))
        return;

    const auto& in = *io.input_buffer;
    const int frames = static_cast<int>(in.size_frames());
    if (frames <= 0)
        return;

    const int ch = std::max(1, g_input_channels.load(std::memory_order_relaxed));

    std::vector<int16_t> pcm(static_cast<size_t>(frames));
    const float* data = in.data();
    for (int i = 0; i < frames; ++i)
    {
        float sample = data[static_cast<size_t>(i) * static_cast<size_t>(ch)];
        if (sample > 1.0f)
            sample = 1.0f;
        else if (sample < -1.0f)
            sample = -1.0f;
        pcm[static_cast<size_t>(i)] = static_cast<int16_t>(sample * 32767.0f);
    }

    {
        std::lock_guard<std::mutex> lock(g_queue_mutex);
        g_pcm_queue.push(std::move(pcm));
    }
}

#ifdef _WIN32

bool key_down(int vk)
{
    return (GetKeyState(vk) & 0x8000) != 0;
}

// Wait until key is pressed (edge), ignoring held state from previous press.
void wait_key_press(int vk)
{
    while (key_down(vk))
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    while (!key_down(vk))
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

bool escape_pressed()
{
    return key_down(VK_ESCAPE);
}

#else

class TermRawMode
{
public:
    TermRawMode()
    {
        if (tcgetattr(STDIN_FILENO, &m_old) == 0)
        {
            termios raw = m_old;
            raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            m_ok = true;
        }
    }

    ~TermRawMode()
    {
        if (m_ok)
            tcsetattr(STDIN_FILENO, TCSANOW, &m_old);
    }

    TermRawMode(const TermRawMode&) = delete;
    TermRawMode& operator=(const TermRawMode&) = delete;

private:
    termios m_old{};
    bool m_ok = false;
};

int poll_key()
{
    unsigned char ch = 0;
    const ssize_t n = read(STDIN_FILENO, &ch, 1);
    if (n <= 0)
        return -1;
    return static_cast<int>(ch);
}

void wait_key_press_char(char want)
{
    for (;;)
    {
        const int ch = poll_key();
        if (ch == want || ch == (want - 'A' + 'a') || ch == (want - 'a' + 'A'))
            return;
        // ESC while waiting to start: ignore here; handled in recording loop.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool escape_pressed()
{
    // Drain available keys; return true if ESC seen.
    for (;;)
    {
        const int ch = poll_key();
        if (ch < 0)
            return false;
        if (ch == 27)
            return true;
    }
}

#endif

std::string extract_json_text(const char* json)
{
    if (!json)
        return {};

    const char* key = std::strstr(json, "\"text\"");
    if (!key)
        return {};

    const char* colon = std::strchr(key, ':');
    if (!colon)
        return {};

    const char* first = std::strchr(colon, '"');
    if (!first)
        return {};

    ++first;
    std::string out;
    for (const char* p = first; *p; ++p)
    {
        if (*p == '\\' && p[1] != '\0')
        {
            out.push_back(p[1]);
            ++p;
            continue;
        }
        if (*p == '"')
            break;
        out.push_back(*p);
    }
    return out;
}

void feed_pcm(VoskRecognizer* recognizer, const std::vector<int16_t>& pcm)
{
    (void)vosk_recognizer_accept_waveform_s(
        recognizer, pcm.data(), static_cast<int>(pcm.size()));
}

void drain_and_feed(VoskRecognizer* recognizer)
{
    for (;;)
    {
        std::vector<int16_t> pcm;
        {
            std::lock_guard<std::mutex> lock(g_queue_mutex);
            if (g_pcm_queue.empty())
                break;
            pcm = std::move(g_pcm_queue.front());
            g_pcm_queue.pop();
        }
        if (!pcm.empty())
            feed_pcm(recognizer, pcm);
    }
}

} // namespace

// Adaptation reference
// https://alphacephei.com/vosk/adaptation

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "zh_CN.utf8");

#ifndef _WIN32
    TermRawMode raw_mode;
#endif

    const char* model_path = kDefaultModelPath;
    if (argc > 1 && argv[1] != nullptr && argv[1][0] != '\0')
        model_path = argv[1];

    float sample_rate = 16000.0f;
    const int batch_size = 8000;

    print_all_devices();

    set_audio_device_list_callback(audio_device_list_event::device_list_changed, [] {
        std::cout << "\n=== Audio device list changed! ===\n\n";
        print_all_devices();
    });

    set_audio_device_list_callback(audio_device_list_event::default_input_device_changed, [] {
        std::cout << "\n=== Default input device changed! ===\n\n";
        print_all_devices();
    });

    set_audio_device_list_callback(audio_device_list_event::default_output_device_changed, [] {
        std::cout << "\n=== Default output device changed! ===\n\n";
        print_all_devices();
    });

    auto device = get_default_audio_input_device();
    if (!device)
    {
        std::cerr << "No default audio input device.\n";
        return 1;
    }

    device->set_sample_rate(sample_rate);
    sample_rate = static_cast<float>(device->get_sample_rate());
    g_input_channels.store(std::max(1, static_cast<int>(device->get_num_input_channels())),
                           std::memory_order_relaxed);
    std::cout << "Sample rate " << sample_rate
              << ", input channels " << g_input_channels.load() << std::endl;

    device->set_buffer_size_frames(batch_size);

    VoskModel* model = vosk_model_new(model_path);
    if (!model)
    {
        std::cerr << "Failed to load model: " << model_path << "\n";
        return 1;
    }

    VoskRecognizer* recognizer = vosk_recognizer_new(model, sample_rate);
    if (!recognizer)
    {
        std::cerr << "Failed to create recognizer.\n";
        vosk_model_free(model);
        return 1;
    }

    device->connect(callback);
    // Keep device running; only queue PCM while g_recording is true.
    device->start();

    std::cout << std::string(80, '#') << "\n";
    std::cout << "Model: " << model_path << "\n";
    std::cout << "Press S to start recording, ESC to stop and show result.\n";
    std::cout << std::string(80, '#') << "\n";
    std::cout.flush();

    // ---- wait for S to start ----
    std::cout << "Waiting for S...\n";
    std::cout.flush();
#ifdef _WIN32
    wait_key_press('S');
#else
    wait_key_press_char('S');
#endif

    vosk_recognizer_reset(recognizer);
    clear_pcm_queue();
    g_recording.store(true, std::memory_order_relaxed);

    std::cout << "Recording... (press ESC to stop)\n";
    std::cout.flush();

    // Consume any ESC that was already down before recording started.
#ifdef _WIN32
    while (escape_pressed())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif

    // ---- record until ESC ----
    while (device->is_running() && !escape_pressed())
    {
        drain_and_feed(recognizer);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    g_recording.store(false, std::memory_order_relaxed);
    device->stop();

    // Flush remaining buffered audio, then emit one final result.
    drain_and_feed(recognizer);

    const char* final_json = vosk_recognizer_final_result(recognizer);
    const std::string text = extract_json_text(final_json);

    std::cout << std::string(80, '#') << "\n";
    std::cout << "Recognition result:\n";
    if (!text.empty())
        std::cout << text << "\n";
    else if (final_json)
        std::cout << final_json << "\n";
    else
        std::cout << "(empty)\n";
    std::cout << std::string(80, '#') << "\n";

    vosk_recognizer_free(recognizer);
    vosk_model_free(model);
    return 0;
}
