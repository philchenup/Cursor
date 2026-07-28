将科大讯飞官方 MSC SDK 的库文件放在此目录：

- Linux: libmsc.so（或厂商提供的同名动态库）
- Windows x64: msc_x64.lib（并把 msc_x64.dll 放到可执行文件目录）
- Windows x86: msc.lib / msc.dll

头文件放在 ../include，可用官方包覆盖。
