QT += core gui widgets multimedia

CONFIG += c++17
TEMPLATE = app
TARGET = xfyun_qt_asr

# 默认使用 Mock；接入正式 SDK 时在 .pro 或命令行去掉该宏，并链接 msc。
DEFINES += XFYUN_USE_MOCK

INCLUDEPATH += $$PWD/include \
               $$PWD/third_party/xfyun/include

HEADERS += \
    include/audio_capture.h \
    include/xfyun_asr_engine.h \
    include/voice_input_dialog.h

SOURCES += \
    src/main.cpp \
    src/audio_capture.cpp \
    src/xfyun_asr_engine.cpp \
    src/voice_input_dialog.cpp \
    src/msc_mock.cpp

# 正式 SDK 示例（关闭 XFYUN_USE_MOCK 后启用）：
# unix: LIBS += -L$$PWD/third_party/xfyun/lib -lmsc
# win32:contains(QT_ARCH, x86_64): LIBS += $$PWD/third_party/xfyun/lib/msc_x64.lib
# win32:!contains(QT_ARCH, x86_64): LIBS += $$PWD/third_party/xfyun/lib/msc.lib
