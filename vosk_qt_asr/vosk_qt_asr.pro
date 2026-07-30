QT += core gui widgets multimedia

CONFIG += c++17
TEMPLATE = app
TARGET = vosk_qt_asr

# Visual Studio + Qt VS Tools：可用本 .pro 打开工程
# 或使用 CMakeLists.txt（推荐）生成 VS 解决方案

INCLUDEPATH += $$PWD/include \
               $$PWD/third_party/vosk

HEADERS += \
    include/audio_capture.h \
    include/vosk_asr_engine.h \
    include/main_window.h \
    include/vosk_api.h

SOURCES += \
    src/main.cpp \
    src/audio_capture.cpp \
    src/vosk_asr_engine.cpp \
    src/main_window.cpp

# ---- Vosk 链接 ----
# Linux: 将 libvosk.so 放到 third_party/vosk/
unix:!macx {
    LIBS += -L$$PWD/third_party/vosk -lvosk
    QMAKE_RPATHDIR += $$PWD/third_party/vosk
}

# Windows (Visual Studio x64):
# 从 https://github.com/alphacep/vosk-api/releases 下载 vosk-win64-*.zip
# 将 libvosk.dll / libvosk.lib / vosk_api.h 放到 third_party/vosk/ 与 include/
win32 {
    LIBS += -L$$PWD/third_party/vosk -llibvosk
    # 调试时把 DLL 拷到输出目录
    CONFIG(debug, debug|release) {
        DESTDIR_TARGET = $$OUT_PWD/debug
    } else {
        DESTDIR_TARGET = $$OUT_PWD/release
    }
}

# 工作目录设为项目根，便于找到 ./model
# Qt Creator: Projects → Run → Working directory = %{sourceDir}
# Visual Studio: 调试 → 工作目录 = $(ProjectDir) 或 vosk_qt_asr 根目录
