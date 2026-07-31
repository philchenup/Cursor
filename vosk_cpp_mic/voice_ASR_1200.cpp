#include <algorithm>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <cmath>
#include <cstring>
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
#endif

using namespace std::experimental;

// Align with Python test_microphone.py:
//   - mono int16 PCM
//   - sample rate = device rate (also passed to recognizer)
//   - queue audio from callback -> main thread AcceptWaveform
// Do NOT scale float samples by 32768 and feed vosk_recognizer_accept_waveform_f.

namespace {

constexpr const char* kDefaultModelPath = "model/vosk-model-cn-kaldi-multicn-0.15";

std::mutex g_queue_mutex;
std::queue<std::vector<int16_t>> g_pcm_queue;
std::atomic<bool> g_running{true};
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

// Convert float interleaved input to mono int16 (same as Python dtype=int16, channels=1).
void callback(audio_device&, audio_device_io<float>& io) noexcept
{
    if (!io.input_buffer.has_value() || !g_running.load(std::memory_order_relaxed))
        return;

    const auto& in = *io.input_buffer;
    const int frames = static_cast<int>(in.size_frames());
    if (frames <= 0)
        return;

    // Prefer channel 0 only; experimental::audio layout is frame-major interleaved.
    // Matches Python: channels=1.
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

bool escape_pressed()
{
#ifdef _WIN32
    return (GetKeyState(VK_ESCAPE) & 0x8000) != 0;
#else
    return false;
#endif
}

} // namespace

// Adaptation reference
// https://alphacephei.com/vosk/adaptation

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "zh_CN.utf8");

    const char* model_path = kDefaultModelPath;
    if (argc > 1 && argv[1] != nullptr && argv[1][0] != '\0')
        model_path = argv[1];

    // Prefer 16 kHz like common Vosk CN models; fall back to device default if unset fails.
    float sample_rate = 16000.0f;
    const int batch_size = 8000; // match Python blocksize=8000

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

    // Sample rate MUST match the PCM we feed (same as KaldiRecognizer(model, samplerate)).
    VoskRecognizer* recognizer = vosk_recognizer_new(model, sample_rate);
    if (!recognizer)
    {
        std::cerr << "Failed to create recognizer.\n";
        vosk_model_free(model);
        return 1;
    }

    device->connect(callback);
    device->start();

    std::cout << std::string(80, '#') << "\n";
    std::cout << "Press Esc (Windows) / Ctrl+C to stop. Model: " << model_path << "\n";
    std::cout << std::string(80, '#') << "\n";

    while (device->is_running())
    {
        std::vector<int16_t> pcm;
        {
            std::lock_guard<std::mutex> lock(g_queue_mutex);
            if (!g_pcm_queue.empty())
            {
                pcm = std::move(g_pcm_queue.front());
                g_pcm_queue.pop();
            }
        }

        if (!pcm.empty())
        {
            // Same path as Python AcceptWaveform(int16 bytes).
            if (vosk_recognizer_accept_waveform_s(
                    recognizer, pcm.data(), static_cast<int>(pcm.size())))
            {
                printf("%s\n", vosk_recognizer_result(recognizer));
            }
            else
            {
                const char* partial = vosk_recognizer_partial_result(recognizer);
                // Skip empty partial: {"partial" : ""}
                if (partial && std::strstr(partial, "\"partial\"") != nullptr)
                {
                    const char* q = std::strchr(partial, ':');
                    if (q)
                    {
                        const char* first = std::strchr(q, '"');
                        if (first && first[1] != '"')
                            printf("%s\n", partial);
                    }
                }
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (escape_pressed())
        {
            g_running.store(false, std::memory_order_relaxed);
            device->stop();
        }
    }

    printf("%s\n", vosk_recognizer_final_result(recognizer));

    vosk_recognizer_free(recognizer);
    vosk_model_free(model);
    return 0;
}
