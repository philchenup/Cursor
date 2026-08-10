# Windows / MSVC: `LNK2001` LZ4 symbols from `pcd_utils.obj`

## Symptom

Visual Studio reports unresolved externals such as:

```text
pcd_utils.obj : error LNK2001: 无法解析的外部符号 LZ4_resetStreamHC
pcd_utils.obj : error LNK2001: 无法解析的外部符号 LZ4_setStreamDecode
pcd_utils.obj : error LNK2001: 无法解析的外部符号 LZ4_decompress_safe_continue
pcd_utils.obj : error LNK2001: 无法解析的外部符号 LZ4_decompress_safe
pcd_utils.obj : error LNK2001: 无法解析的外部符号 LZ4_compress_HC_continue
```

(English MSVC: `unresolved external symbol LZ4_…`)

These five symbols always appear together.

## Root cause

They come from **LZ4** (including **LZ4HC**), pulled in by **FLANN** serialization — not from PCL’s own PCD LZF codec.

Typical call chain on Windows:

1. Your `.cpp` (here `pcd_utils.cpp`) uses PCL search/kdtree APIs  
   (e.g. `pcl::KdTreeFLANN`, `pcl::search::KdTree`, or headers that include FLANN).
2. FLANN headers reference `LZ4_compress_HC_continue` / `LZ4_decompress_safe*` / stream APIs.
3. The **final** exe/DLL must link `lz4.lib`. PCL/FLANN often do **not** propagate that dependency into a hand-made VS project (MakeTool, etc.).

`pcd_utils.obj` is only the translation unit that *instantiated* those templates; the missing library is still **LZ4**.

| Symbol | Provided by |
| --- | --- |
| `LZ4_decompress_safe` | `lz4` |
| `LZ4_decompress_safe_continue` | `lz4` |
| `LZ4_setStreamDecode` | `lz4` |
| `LZ4_compress_HC_continue` | `lz4hc` (same `lz4` package) |
| `LZ4_resetStreamHC` | `lz4hc` (same `lz4` package) |

One complete `lz4` install covers all five. You do **not** need a separate “lz4hc.lib” from most vcpkg/official builds.

## Fix (Visual Studio project, e.g. MakeTool)

1. Install LZ4 for your triplet, e.g. via vcpkg:
   ```text
   vcpkg install lz4:x64-windows
   ```
   Or use the `lz4.lib` that shipped next to your PCL/FLANN build.
2. **Linker → General → Additional Library Directories**  
   add the folder that contains `lz4.lib`  
   (often `...\vcpkg\installed\x64-windows\lib`, or your third-party `lib` dir).
3. **Linker → Input → Additional Dependencies**  
   add:
   ```text
   lz4.lib
   ```
   Some packs name it `liblz4.lib` — use the exact filename on disk.
4. Match **x64/x86** and **Debug/Release** (and the same CRT) as your app and PCL.
5. Rebuild the **executable / DLL** that links `pcd_utils.obj` (not only a static lib that still leaves LZ4 unresolved for the final link).

### If symbols look like `__imp_LZ4_*`

That means the headers expect a **DLL import** build. Either:

- link the **import** `.lib` that belongs to `lz4.dll`, and ensure `lz4.dll` is on `PATH` / next to the exe, **or**
- link the **static** `lz4.lib` and do **not** define `LZ4_DLL_IMPORT`.

Do not mix a static `lz4.lib` with headers compiled as DLL-import (or the reverse).

### Still failing after adding `lz4.lib`

| Check | Action |
| --- | --- |
| Wrong arch | x64 project must use x64 `lz4.lib` |
| Stale name | Confirm file is actually `lz4.lib` / `liblz4.lib` |
| Only `lz4.c` without HC | Use a full LZ4 package (must include HC APIs) |
| FLANN static + no lz4 | Always add `lz4.lib` on the final link line |
| Multiple LZ4 copies | One include path + one lib; avoid mixing versions |

## Fix (CMake)

```cmake
find_package(PCL REQUIRED COMPONENTS common io kdtree search)
find_package(lz4 CONFIG QUIET)
if(lz4_FOUND)
  set(_LZ4_TARGET lz4::lz4)
else()
  find_library(LZ4_LIBRARY NAMES lz4 liblz4 REQUIRED)
  set(_LZ4_TARGET ${LZ4_LIBRARY})
endif()

add_library(pcd_utils STATIC pcd_utils.cpp)  # or your target name
target_link_libraries(pcd_utils PUBLIC ${PCL_LIBRARIES} ${_LZ4_TARGET})
```

For a plain VS-generated or MakeTool-style link line, the equivalent is appending `lz4.lib` (and its library directory).

Minimal probe (optional):

```cmake
include(cmake/FindOrLinkLZ4.cmake)
target_link_lz4(YourTarget)
```

See [`cmake/FindOrLinkLZ4.cmake`](cmake/FindOrLinkLZ4.cmake).

## What this is *not*

- **Not** PCL `BINARY_COMPRESSED` PCD / `pcl::lzf` — those use **LZF**, not these `LZ4_*` symbols.
- **Not** fixed by adding more `pcl_*.lib` alone if LZ4 was never linked into your project.
- **Not** fixed by only rebuilding `pcd_utils.cpp` without changing **Additional Dependencies**.

## Quick checklist

1. `lz4.lib` is on the final link line of the failing target.  
2. Library directory matches that `.lib`.  
3. Arch / config match the rest of the PCL stack.  
4. Rebuild the exe/DLL after the linker change.
