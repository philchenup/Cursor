/**
 * @file qisr.h
 * @brief 科大讯飞语音听写（QISR）接口
 */
#ifndef __QISR_H__
#define __QISR_H__

#include "msp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 开启一次听写会话。
 * @param grammarList 语法列表，听写场景传 NULL
 * @param params      会话参数，例如
 *   "sub = iat, domain = iat, language = zh_cn, accent = mandarin,
 *    sample_rate = 16000, result_type = plain, result_encoding = utf8,
 *    vad_eos = 3000, vad_bos = 5000, ptt = 1"
 * @param errorCode   输出错误码
 * @return 会话 ID（失败返回 NULL）
 */
const char* QISRSessionBegin(const char* grammarList,
                             const char* params,
                             int* errorCode);

/**
 * 向当前会话写入 PCM 音频。
 * 音频要求：16kHz / 16bit / 单声道 / 小端 PCM。
 */
int QISRAudioWrite(const char* sessionID,
                   const void* waveData,
                   unsigned int waveLen,
                   int audioStatus,
                   int* epStatus,
                   int* recogStatus);

/**
 * 获取识别结果。
 * @param waitTime 阻塞等待时间（毫秒），录音过程中建议传 0
 */
const char* QISRGetResult(const char* sessionID, int* rsltStatus, int waitTime, int* errorCode);

/** 结束会话并释放服务端资源。 */
int QISRSessionEnd(const char* sessionID, const char* hints);

#ifdef __cplusplus
}
#endif

#endif /* __QISR_H__ */
