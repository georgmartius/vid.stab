# Single source of truth for the library version, included by the top-level
# CMakeLists.txt and by tests/CMakeLists.txt, which is a standalone project.
set(MAJOR_VERSION 1)
# SOVERSION is MAJOR.MINOR: bump the minor whenever a public struct changes
# size, or consumers that embed one by value corrupt their heap.
set(MINOR_VERSION 4)
set(PATCH_VERSION 0)
