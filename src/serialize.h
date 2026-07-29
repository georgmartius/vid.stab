/*
 * serialize.h
 *
 *  Copyright (C) Georg Martius - January 2013
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
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __SERIALIZE_H
#define __SERIALIZE_H

#define LIBVIDSTAB_FILE_FORMAT_VERSION 1

#include "transformtype.h"
#include "motiondetect.h"
#include "transform.h"
#include "vidstab_api.h"

/*
 * IMPORTANT - contract for all FILE* passed to the functions below:
 *
 * The stream MUST be opened in BINARY mode, i.e. fopen(path, "wb") for
 * writing and fopen(path, "rb") for reading.
 *
 * The default (BINARY_SERIALIZATION_MODE) transform file format contains
 * arbitrary binary data (int16/int32/double). On platforms that distinguish
 * text and binary streams (Windows/MSVCRT, but also OS/2, DOS, ...) a stream
 * opened in text mode mangles that data:
 *   - reading stops at the first 0x1A (Ctrl-Z) byte, which is interpreted as
 *     end-of-file. Such a byte occurs in practically every transform file
 *     (~0.15% of all bytes, mostly inside the double mantissas), so the file
 *     is silently truncated after a few frames and parsing fails with
 *     "Cannot parse localmotion!" - see issue #104.
 *   - 0x0A bytes are expanded to 0x0D 0x0A on writing, so the file is not
 *     portable to a reader that (correctly) uses a binary stream.
 * Note that on POSIX systems text and binary mode are identical, which is why
 * a caller that forgets the "b" only breaks on Windows.
 */

/// Vector of LocalMotions
typedef VSVector VSManyLocalMotions;
/// helper macro to access a localmotions vector in the VSVector of all Frames
#define VSMLMGet(manylocalmotions,index) \
    ((LocalMotions*)vs_vector_get(manylocalmotions,index))

/// guess the serialization mode of the local motions file
VS_API int vsGuessSerializationMode(FILE* f);

/// stores local motions to file
VS_API int vsStoreLocalmotions(FILE* f, const LocalMotions* lms, const int serializationMode);

/// restores local motions from file
VS_API LocalMotions vsRestoreLocalmotions(FILE* f, const int serializationMode);


/// writes the header to the file that is to be holding the local motions
VS_API int vsPrepareFile(const VSMotionDetect* td, FILE* f);

/// appends the given localmotions to the file
VS_API int vsWriteToFile(const VSMotionDetect* td, FILE* f, const LocalMotions* lms);

/// reads the header of the file and return the version number (used by readLocalmotionsFile)
VS_API int vsReadFileVersion(FILE* f, const int serializationMode);

/*
 * reads the next set of localmotions from the file, return VS_ERROR on error or
 * if nothing is read (used by readLocalmotionsFile)
 */
VS_API int vsReadFromFile(FILE* f, LocalMotions* lms, const int serializationMode);

/*
 * reads the entire file of localmotions, return VS_ERROR on error or if nothing is read
 *
 *  The format is as follows:
 *   The file must begin with 'VID.STAB version\n'
 *   Lines with # at the beginning are comments and will be ignored
 *   Data lines have the structure: Frame NUM (<LocalMotions>)
 *   where LocalMotions ::= List [(LM v.x v.y f.x f.y f.size contrast match),...]
 */
VS_API int vsReadLocalMotionsFile(FILE* f, VSManyLocalMotions* lms);

// read the transformations from the given file (Deprecated format)
VS_API int vsReadOldTransforms(const VSTransformData* td, FILE* f , VSTransformations* trans);


#endif
