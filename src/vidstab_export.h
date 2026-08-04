/*
 *  vidstab_export.h
 *
 *  Symbol visibility for the public library interface.
 *
 *  VS_API expands to the export attribute while the library itself is being
 *  built, to the import attribute for code that includes these headers from
 *  outside, and to nothing when the sources are linked in statically.  The
 *  three cases are told apart the way the build system sets them up:
 *
 *    vidstab_EXPORTS        defined by CMake in the library's own translation
 *                           units, and only for a shared build
 *    VIDSTAB_STATIC_DEFINE  defined by the build for a static library, and by
 *                           anything that compiles the sources in directly
 *                           (the test suite and the transcode plugins do)
 *
 *  Names and semantics are those of CMake's generate_export_header(), so a
 *  build that would rather use the generated header can substitute it without
 *  touching a single source file.
 *
 *  Copyright (C) Georg Martius - 2026
 *   georg dot martius at web dot de
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License,
 *  as published by the Free Software Foundation; either version 2, or
 *  (at your option) any later version.
 *
 *  vid.stab is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with GNU Make; see the file COPYING.  If not, write to
 *  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *  Boston, MA 02110-1301, USA.
 *
 */

#ifndef VIDSTAB_EXPORT_H
#define VIDSTAB_EXPORT_H

#ifdef VIDSTAB_STATIC_DEFINE
#  define VS_API
#elif defined(_WIN32) || defined(__CYGWIN__)
/* __declspec is understood by MSVC, clang-cl and MinGW alike, so the test is
   on the platform and not on the compiler. */
#  ifdef vidstab_EXPORTS
#    define VS_API __declspec(dllexport)
#  else
#    define VS_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define VS_API __attribute__((visibility("default")))
#else
#  define VS_API
#endif

#endif /* VIDSTAB_EXPORT_H */
