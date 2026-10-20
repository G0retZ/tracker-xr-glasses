# User-settings file handed to opentrack's build via -DOPENTRACK_USERCONFIG.
#
# Without it, opentrack's cmake/opentrack-load-user-settings.cmake picks
# "sdk-paths-<username>@MSVC-windows.cmake" from its source root. The GitHub
# runner's username is "runneradmin", and opentrack commits a file for that
# name (its own CI config) which hard-requires a pre-built OpenCV. We only
# build one tracker module, so this file replaces it with the minimum needed
# to stay binary-compatible with opentrack's release builds.

set(opentrack_install-debug-info FALSE CACHE INTERNAL "" FORCE)

# opentrack releases link the DLL C runtime (they ship msvcp140.dll next to
# opentrack.exe). The plugin must use the same runtime or heap/CRT state is
# split between the two.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
add_compile_options(-MD)

# No SDK_* paths: this module needs none, and every other SDK-gated opentrack
# module simply stays unbuilt.
