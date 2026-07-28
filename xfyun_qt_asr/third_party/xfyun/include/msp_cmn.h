/**
 * @file msp_cmn.h
 * @brief 科大讯飞 MSC 公共接口（登录/登出）
 */
#ifndef __MSP_CMN_H__
#define __MSP_CMN_H__

#include "msp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 登录讯飞开放平台。
 * @param usr    用户名，在线听写通常传 NULL
 * @param pwd    密码，在线听写通常传 NULL
 * @param params 登录参数，例如 "appid = xxxxxxxx, work_dir = ."
 */
int MSPLogin(const char* usr, const char* pwd, const char* params);

/** 登出并释放 MSC 资源。 */
int MSPLogout();

/** 按键名查询配置/版本等信息。 */
const char* MSPGetVersion(const char* verName);

#ifdef __cplusplus
}
#endif

#endif /* __MSP_CMN_H__ */
