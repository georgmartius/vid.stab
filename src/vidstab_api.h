/*
 *  vidstab_api.h
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
 *                           (the test suite does)
 *
 *  The macro names are those of CMake's generate_export_header(), so a build
 *  that would rather use the generated header only has to point the includes
 *  at it; nothing else in the sources changes.
 *
 *  Copyright (C) Georg Martius - 2026
 *   georg dot martius at web dot de
 *
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published
 *  by the Free Software Foundation; either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  vid.stab is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with vid.stab; see the file COPYING.LESSER.  If not, see
 *  <https://www.gnu.org/licenses/>.
 *
 */

#ifndef VIDSTAB_API_H
#define VIDSTAB_API_H

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

#endif /* VIDSTAB_API_H */
