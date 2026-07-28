/**
 * @file msc_mock.cpp
 * @brief 无正式 MSC 动态库时的本地 Mock，便于先跑通 Qt 对话框与录音链路。
 *
 * 编译选项 -DXFYUN_USE_MOCK=ON 时链接本文件；接入正式 SDK 后关闭该选项。
 */
#include "msp_cmn.h"
#include "msp_errors.h"
#include "qisr.h"

#include <cstring>
#include <mutex>
#include <string>

namespace {

std::mutex g_mutex;
bool g_logged_in = false;
std::string g_session_id;
int g_audio_chunks = 0;
bool g_result_ready = false;

// UTF-8: "这是一次模拟识别结果，请替换为讯飞正式 SDK。"
const char* kFakeResultUtf8 =
    "\xe8\xbf\x99\xe6\x98\xaf\xe4\xb8\x80\xe6\xac\xa1\xe6\xa8\xa1\xe6\x8b\x9f"
    "\xe8\xaf\x86\xe5\x88\xab\xe7\xbb\x93\xe6\x9e\x9c\xef\xbc\x8c\xe8\xaf\xb7"
    "\xe6\x9b\xbf\xe6\x8d\xa2\xe4\xb8\xba\xe8\xae\xaf\xe9\xa3\x9e\xe6\xad\xa3"
    "\xe5\xbc\x8f SDK\xe3\x80\x82";

}  // namespace

extern "C" int MSPLogin(const char* /*usr*/, const char* /*pwd*/, const char* /*params*/)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_logged_in = true;
    return MSP_SUCCESS;
}

extern "C" int MSPLogout()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_logged_in = false;
    g_session_id.clear();
    return MSP_SUCCESS;
}

extern "C" const char* MSPGetVersion(const char* /*verName*/)
{
    return "xfyun-mock-1.0";
}

extern "C" const char* QISRSessionBegin(const char* /*grammarList*/,
                                        const char* /*params*/,
                                        int* errorCode)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_logged_in) {
        if (errorCode) {
            *errorCode = MSP_ERROR_NOT_INIT;
        }
        return nullptr;
    }
    g_session_id = "mock-session-001";
    g_audio_chunks = 0;
    g_result_ready = false;
    if (errorCode) {
        *errorCode = MSP_SUCCESS;
    }
    return g_session_id.c_str();
}

extern "C" int QISRAudioWrite(const char* sessionID,
                              const void* /*waveData*/,
                              unsigned int waveLen,
                              int audioStatus,
                              int* epStatus,
                              int* recogStatus)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!sessionID || g_session_id != sessionID) {
        return MSP_ERROR_INVALID_HANDLE;
    }
    if (waveLen > 0) {
        ++g_audio_chunks;
    }
    if (epStatus) {
        *epStatus = (audioStatus & MSP_AUDIO_SAMPLE_LAST)
                        ? MSP_EP_AFTER_SPEECH
                        : MSP_EP_IN_SPEECH;
    }
    if (recogStatus) {
        *recogStatus = MSP_REC_STATUS_SUCCESS;
    }
    if (audioStatus & MSP_AUDIO_SAMPLE_LAST) {
        g_result_ready = true;
    }
    return MSP_SUCCESS;
}

extern "C" const char* QISRGetResult(const char* sessionID,
                                     int* rsltStatus,
                                     int /*waitTime*/,
                                     int* errorCode)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!sessionID || g_session_id != sessionID) {
        if (errorCode) {
            *errorCode = MSP_ERROR_INVALID_HANDLE;
        }
        return nullptr;
    }
    if (errorCode) {
        *errorCode = MSP_SUCCESS;
    }
    if (!g_result_ready) {
        if (rsltStatus) {
            *rsltStatus = MSP_REC_STATUS_INCOMPLETE;
        }
        return nullptr;
    }
    if (rsltStatus) {
        *rsltStatus = MSP_REC_STATUS_COMPLETE;
    }
    g_result_ready = false;
    return kFakeResultUtf8;
}

extern "C" int QISRSessionEnd(const char* sessionID, const char* /*hints*/)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!sessionID || g_session_id != sessionID) {
        return MSP_ERROR_INVALID_HANDLE;
    }
    g_session_id.clear();
    return MSP_SUCCESS;
}
