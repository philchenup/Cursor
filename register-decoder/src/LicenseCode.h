#pragma once

#include <QString>

// 与 frmRegister::GetKey 相同：MD5 十六进制小写，再拼成 5 + 前10位 + 1 + 其余 + 2。
QString makeAuthorizationCode(const QString& serial);
