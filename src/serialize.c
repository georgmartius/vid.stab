/*
 * serialize.c
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

#include <assert.h>
#include <string.h>

#include "serialize.h"
#include "transformtype.h"
#include "transformtype_operations.h"
#include "motiondetect.h"

#if defined(__BYTE_ORDER) && __BYTE_ORDER == __BIG_ENDIAN || \
    defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ || \
    defined(__BIG_ENDIAN__) || \
    defined(__ARMEB__) || \
    defined(__THUMBEB__) || \
    defined(__AARCH64EB__) || \
    defined(_MIBSEB) || defined(__MIBSEB) || defined(__MIBSEB__)
// It's a big-endian target architecture
#define __IS_BIG_ENDIAN__
#include <byteswap.h>
static double byteSwapDouble(double v)
{
    char in[8], out[8];
    double result;
    memcpy(in, &v, 8);
    out[0] = in[7];
    out[1] = in[6];
    out[2] = in[5];
    out[3] = in[4];
    out[4] = in[3];
    out[5] = in[2];
    out[6] = in[1];
    out[7] = in[0];
    memcpy(&result, out, 8);
    return result;
}
#elif defined(__BYTE_ORDER) && __BYTE_ORDER == __LITTLE_ENDIAN || \
    defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ || \
    defined(__LITTLE_ENDIAN__) || \
    defined(__ARMEL__) || \
    defined(__THUMBEL__) || \
    defined(__AARCH64EL__) || \
    defined(_MIPSEL) || defined(__MIPSEL) || defined(__MIPSEL__)
// It's a little-endian target architecture
#define __IS_LITTLE_ENDIAN__
#else
#error "I don't know what architecture this is!"
#endif

const char* modname = "vid.stab - serialization";

int vsPrepareFileText(const VSMotionDetect* md, FILE* f);
int vsPrepareFileBinary(const VSMotionDetect* md, FILE* f);
int vsWriteToFileText(const VSMotionDetect* md, FILE* f, const LocalMotions* lms);
int vsWriteToFileBinary(const VSMotionDetect* md, FILE* f, const LocalMotions* lms);
int vsStoreLocalmotionsText(FILE* f, const LocalMotions* lms);
int vsStoreLocalmotionsBinary(FILE* f, const LocalMotions* lms);
int storeLocalmotionText(FILE* f, const LocalMotion* lm);
int storeLocalmotionBinary(FILE* f, const LocalMotion* lm);
LocalMotions vsRestoreLocalmotionsText(FILE* f);
LocalMotions vsRestoreLocalmotionsBinary(FILE* f);
LocalMotion restoreLocalmotionText(FILE* f);
LocalMotion restoreLocalmotionBinary(FILE* f);
int vsReadFileVersionText(FILE* f);
int vsReadFileVersionBinary(FILE* f);
int vsReadFromFileText(FILE* f, LocalMotions* lms);
int vsReadFromFileBinary(FILE* f, LocalMotions* lms);

int readInt16(int16_t* i, FILE* f){
  int result = fread(i, sizeof(int16_t), 1, f);
  #ifdef __IS_BIG_ENDIAN__
  if(result>0) *i = __bswap_16(*i);
  #endif
  return result;
}

int readInt32(int32_t* i, FILE* f){
  int result = fread(i, sizeof(int32_t), 1, f);
  #ifdef __IS_BIG_ENDIAN__
  if(result>0) *i = __bswap_32(*i);
  #endif
  return result;
}

int readDouble(double* d, FILE* f){
  int result = fread(d, sizeof(double), 1, f);
  #ifdef __IS_BIG_ENDIAN__
  if(result>0) *d = byteSwapDouble(*d);
  #endif
  return result;
}

int writeInt16(const int16_t* i, FILE* f){
  int16_t val = *i;
  #ifdef __IS_BIG_ENDIAN__
  val = __bswap_16(val);
  #endif
  return fwrite((const void*)&val, sizeof(int16_t), 1, f);
}

int writeInt32(const int32_t* i, FILE* f){
  int32_t val = *i;
  #ifdef __IS_BIG_ENDIAN__
  val = __bswap_32(val);
  #endif
  return fwrite((const void*)&val, sizeof(int32_t), 1, f);
}

int writeDouble(const double* d, FILE* f){
  double val = *d;
  #ifdef __IS_BIG_ENDIAN__
  val = byteSwapDouble(val);
  #endif
  return fwrite((const void*)&val, sizeof(double), 1, f);
}

int storeLocalmotion(FILE* f, const LocalMotion* lm, int serializationMode){
  if(serializationMode == BINARY_SERIALIZATION_MODE){
    return storeLocalmotionBinary(f, lm);
  } else {
    return storeLocalmotionText(f, lm);
  }
}

int storeLocalmotionText(FILE* f, const LocalMotion* lm) {
  return fprintf(f,"(LM %hi %hi %hi %hi %hi %lf %lf)", lm->v.x,lm->v.y,lm->f.x,lm->f.y,lm->f.size,
                 lm->contrast, lm->match);
}

int storeLocalmotionBinary(FILE* f, const LocalMotion* lm) {
  if (writeInt16(&lm->v.x, f)<=0) return 0;
  if (writeInt16(&lm->v.y, f)<=0) return 0;
  if (writeInt16(&lm->f.x, f)<=0) return 0;
  if (writeInt16(&lm->f.y, f)<=0) return 0;
  if (writeInt16(&lm->f.size, f)<=0) return 0;
  if (writeDouble(&lm->contrast, f)<=0) return 0;
  if (writeDouble(&lm->match, f)<=0) return 0;
  return 1;
}

/// restore local motion from file
LocalMotion restoreLocalmotion(FILE* f, const int serializationMode){
  if(serializationMode == BINARY_SERIALIZATION_MODE) {
    return restoreLocalmotionBinary(f);
  } else {
    return restoreLocalmotionText(f);
  }
}

LocalMotion restoreLocalmotionText(FILE* f){
  LocalMotion lm;
  char c;
  if(fscanf(f,"(LM %hi %hi %hi %hi %hi %lf %lf", &lm.v.x,&lm.v.y,&lm.f.x,&lm.f.y,&lm.f.size,
            &lm.contrast, &lm.match) != 7) {
    vs_log_error(modname, "Cannot parse localmotion!\n");
    return null_localmotion();
  }
  while((c=fgetc(f)) && c!=')' && c!=EOF);
  if(c==EOF){
    vs_log_error(modname, "Cannot parse localmotion missing ')'!\n");
    return null_localmotion();
  }
  return lm;
}

LocalMotion restoreLocalmotionBinary(FILE* f){
  LocalMotion lm;

  if (readInt16(&lm.v.x, f)<=0) goto parse_error_handling;
  if (readInt16(&lm.v.y, f)<=0) goto parse_error_handling;
  if (readInt16(&lm.f.x, f)<=0) goto parse_error_handling;
  if (readInt16(&lm.f.y, f)<=0) goto parse_error_handling;
  if (readInt16(&lm.f.size, f)<=0) goto parse_error_handling;
  if (readDouble(&lm.contrast, f)<=0) goto parse_error_handling;
  if (readDouble(&lm.match, f)<=0) goto parse_error_handling;

  return lm;

parse_error_handling:
  vs_log_error(modname, "Cannot parse localmotion!\n");
  return null_localmotion();
}

int vsStoreLocalmotions(FILE* f, const LocalMotions* lms, const int serializationMode){
  if(serializationMode == BINARY_SERIALIZATION_MODE) {
    return vsStoreLocalmotionsBinary(f, lms);
  } else {
    return vsStoreLocalmotionsText(f, lms);
  }
}

int vsStoreLocalmotionsText(FILE* f, const LocalMotions* lms){
  int len = vs_vector_size(lms);
  int i;
  fprintf(f,"List %i [",len);
  for (i=0; i<len; i++){
    if(i>0) fprintf(f,",");
    if(storeLocalmotion(f,LMGet(lms,i),ASCII_SERIALIZATION_MODE) <= 0) return 0;
  }
  fprintf(f,"]");
  return 1;
}

int vsStoreLocalmotionsBinary(FILE* f, const LocalMotions* lms){
  const int len = vs_vector_size(lms);
  int i;
  if(writeInt32(&len, f)<=0) return 0;
  for (i=0; i<len; i++){
    if(storeLocalmotion(f,LMGet(lms,i),BINARY_SERIALIZATION_MODE) <= 0) return 0;
  }
  return 1;
}

/// restores local motions from file
LocalMotions vsRestoreLocalmotions(FILE* f, const int serializationMode){
  if(serializationMode == BINARY_SERIALIZATION_MODE) {
    return vsRestoreLocalmotionsBinary(f);
  } else {
    return vsRestoreLocalmotionsText(f);
  }
}

LocalMotions vsRestoreLocalmotionsText(FILE* f){
  LocalMotions lms;
  int i;
  char c;
  int len;
  vs_vector_init(&lms,0);
  if(fscanf(f,"List %i [", &len) != 1) {
    vs_log_error(modname, "Cannot parse localmotions list expect 'List len ['!\n");
    return lms;
  }
  if (len>0){
    vs_vector_init(&lms,len);
    for (i=0; i<len; i++){
      if(i>0) while((c=fgetc(f)) && c!=',' && c!=EOF);
      LocalMotion lm = restoreLocalmotion(f,ASCII_SERIALIZATION_MODE);
      vs_vector_append_dup(&lms,&lm,sizeof(LocalMotion));
    }
  }
  if(len != vs_vector_size(&lms)){
    vs_log_error(modname, "Cannot parse the given number of localmotions!\n");
    return lms;
  }
  while((c=fgetc(f)) && c!=']' && c!=EOF);
  if(c==EOF){
    vs_log_error(modname, "Cannot parse localmotions list missing ']'!\n");
    return lms;
  }
  return lms;
}

LocalMotions vsRestoreLocalmotionsBinary(FILE* f){
  LocalMotions lms;
  int i;
  int len;
  vs_vector_init(&lms,0);
  if(readInt32(&len, f) <= 0) {
    vs_log_error(modname, "Cannot parse localmotions list!\n");
    return lms;
  }
  if (len>0){
    vs_vector_init(&lms,len);
    for (i=0; i<len; i++){
      LocalMotion lm = restoreLocalmotion(f,BINARY_SERIALIZATION_MODE);
      vs_vector_append_dup(&lms,&lm,sizeof(LocalMotion));
    }
  }
  if(len != vs_vector_size(&lms)){
    vs_log_error(modname, "Cannot parse the given number of localmotions!\n");
  }
  return lms;
}

int vsPrepareFile(const VSMotionDetect* md, FILE* f){
  if(md->serializationMode == BINARY_SERIALIZATION_MODE) {
    return vsPrepareFileBinary(md, f);
  } else {
    return vsPrepareFileText(md, f);
  }
}

int vsPrepareFileText(const VSMotionDetect* md, FILE* f){
  if(!f) return VS_ERROR;
  fprintf(f, "VID.STAB %i\n", LIBVIDSTAB_FILE_FORMAT_VERSION);
  fprintf(f, "#      accuracy = %d\n", md->conf.accuracy);
  fprintf(f, "#     shakiness = %d\n", md->conf.shakiness);
  fprintf(f, "#     seek_range = %d\n", md->conf.seek_range);
  fprintf(f, "#      stepsize = %d\n", md->conf.stepSize);
  fprintf(f, "#   mincontrast = %f\n", md->conf.contrastThreshold);
  return VS_OK;
}

int vsPrepareFileBinary(const VSMotionDetect* md, FILE* f){
  static const unsigned char kFileFormatVersion = LIBVIDSTAB_FILE_FORMAT_VERSION;
  if(!f) return VS_ERROR;
  fprintf(f, "TRF%hhu", kFileFormatVersion);
  writeInt32(&md->conf.accuracy, f);
  writeInt32(&md->conf.shakiness, f);
  writeInt32(&md->conf.seek_range, f);
  writeInt32(&md->conf.stepSize, f);
  writeDouble(&md->conf.contrastThreshold, f);
  return VS_OK;
}

int vsWriteToFile(const VSMotionDetect* md, FILE* f, const LocalMotions* lms){
  if(md->serializationMode == BINARY_SERIALIZATION_MODE) {
    return vsWriteToFileBinary(md, f, lms);
  } else {
    return vsWriteToFileText(md, f, lms);
  }
}

int vsWriteToFileText(const VSMotionDetect* md, FILE* f, const LocalMotions* lms){
  if(!f || !lms) return VS_ERROR;

  if(fprintf(f, "Frame %i (", md->frameNum)>0
     && vsStoreLocalmotions(f, lms, ASCII_SERIALIZATION_MODE)>0 && fprintf(f, ")\n"))
    return VS_OK;
  else
    return VS_ERROR;
}

int vsWriteToFileBinary(const VSMotionDetect* md, FILE* f, const LocalMotions* lms){
  if(!f || !lms) return VS_ERROR;

  if(writeInt32(&md->frameNum, f)<=0) return VS_ERROR;
  if(vsStoreLocalmotions(f, lms, BINARY_SERIALIZATION_MODE)<=0) return VS_ERROR;

  return VS_OK;
}

int vsGuessSerializationMode(FILE* f){
  int serializationMode = ASCII_SERIALIZATION_MODE;
  const int pos = ftell(f);

  if(fgetc(f) == 'T' 
      && fgetc(f) == 'R'
      && fgetc(f) == 'F') {
    serializationMode = BINARY_SERIALIZATION_MODE;
  }
  
  fseek(f, pos, SEEK_SET);
  return serializationMode;
}

/// reads the header of the file and return the version number
int vsReadFileVersion(FILE* f, const int serializationMode){
  if(serializationMode == BINARY_SERIALIZATION_MODE) {
    return vsReadFileVersionBinary(f);
  } else {
    return vsReadFileVersionText(f);
  }
}

int vsReadFileVersionText(FILE* f){
  if(!f) return VS_ERROR;
  int version;
  if(fscanf(f, "VID.STAB %i\n", &version)!=LIBVIDSTAB_FILE_FORMAT_VERSION)
    return VS_ERROR;
  else return version;
}

int vsReadFileVersionBinary(FILE* f){
  if(!f) return VS_ERROR;
  unsigned char version;
  VSMotionDetectConfig conf;

  if(fscanf(f, "TRF%hhu\n", &version)!=LIBVIDSTAB_FILE_FORMAT_VERSION) goto parse_error_handling;
  if(readInt32(&conf.accuracy, f)<=0) goto parse_error_handling;
  if(readInt32(&conf.shakiness, f)<=0) goto parse_error_handling;
  if(readInt32(&conf.stepSize, f)<=0) goto parse_error_handling;
  if(readDouble(&conf.contrastThreshold, f)<=0) goto parse_error_handling;
  
  return version;
parse_error_handling:
  return VS_ERROR;
}

int vsReadFromFile(FILE* f, LocalMotions* lms, const int serializationMode){
  if(serializationMode == BINARY_SERIALIZATION_MODE) {
    return vsReadFromFileBinary(f,lms);
  } else {
    return vsReadFromFileText(f,lms);
  }
}

int vsReadFromFileText(FILE* f, LocalMotions* lms){
  char c = fgetc(f);
  if(c=='F') {
    int num;
    if(fscanf(f,"rame %i (", &num)!=1) {
      vs_log_error(modname,"cannot read file, expect 'Frame num (...'");
      return VS_ERROR;
    }
    *lms = vsRestoreLocalmotions(f,ASCII_SERIALIZATION_MODE);
    if(fscanf(f,")\n")<0) {
      vs_log_error(modname,"cannot read file, expect '...)'");
      return VS_ERROR;
    }
    return num;
  } else if(c=='#') {
    char l[1024];
    if(fgets(l, sizeof(l), f)==0) return VS_ERROR;
    return vsReadFromFile(f,lms,ASCII_SERIALIZATION_MODE);
  } else if(c=='\n' || c==' ') {
    return vsReadFromFile(f,lms,ASCII_SERIALIZATION_MODE);
  } else if(c==EOF) {
    return VS_ERROR;
  } else {
    vs_log_error(modname,"cannot read frame local motions from file, got %c (%i)",
                 c, (int) c);
    return VS_ERROR;
  }
}

int vsReadFromFileBinary(FILE* f, LocalMotions* lms){
  int frameNum;
  if(readInt32(&frameNum, f)<=0) return VS_ERROR;
  *lms = vsRestoreLocalmotions(f, BINARY_SERIALIZATION_MODE);
  return frameNum;
}

int vsReadLocalMotionsFile(FILE* f, VSManyLocalMotions* mlms){
  const int serializationMode = vsGuessSerializationMode(f);
  int version = vsReadFileVersion(f, serializationMode);
  if(version<1) // old format or unknown
    return VS_ERROR;
  if(version>1){
    vs_log_error(modname,"Version of VID.STAB file too large: got %i, expect <= 1",
                 version);
    return VS_ERROR;
  }
  assert(mlms);
  // initial number of frames, but it will automatically be increaseed
  vs_vector_init(mlms,1024);
  int index;
  int oldindex = 0;
  LocalMotions lms;
  while((index = vsReadFromFile(f,&lms,serializationMode)) != VS_ERROR){
    if(index > oldindex+1){
      vs_log_info(modname,"VID.STAB file: index of frames is not continuous %i -< %i",
                  oldindex, index);
    }
    if(index<1){
      vs_log_info(modname,"VID.STAB file: Frame number < 1 (%i)", index);
    } else {
      vs_vector_set_dup(mlms,index-1,&lms, sizeof(LocalMotions));
    }
    oldindex=index;
  }
  return VS_OK;
}


/**
 * vsReadOldTransforms: read transforms file (Deprecated format)
 *  The format is as follows:
 *   Lines with # at the beginning are comments and will be ignored
 *   Data lines have 5 columns seperated by space or tab containing
 *   time, x-translation, y-translation, alpha-rotation, extra
 *   where time and extra are integers
 *   and the latter is unused at the moment
 *
 * Parameters:
 *         f:  file description
 *         trans: place to store the transforms
 * Return value:
 *         number of transforms read
 * Preconditions: f is opened
 */
int vsReadOldTransforms(const VSTransformData* td, FILE* f , VSTransformations* trans)
{
  char l[1024];
  int s = 0;
  int i = 0;
  int ti; // time (ignored)
  VSTransform t;

  while (fgets(l, sizeof(l), f)) {
    t = null_transform();
    if (l[0] == '#')
      continue;    //  ignore comments
    if (strlen(l) == 0)
      continue; //  ignore empty lines
    // try new format
    if (sscanf(l, "%i %lf %lf %lf %lf %i", &ti, &t.x, &t.y, &t.alpha,
               &t.zoom, &t.extra) != 6) {
      if (sscanf(l, "%i %lf %lf %lf %i", &ti, &t.x, &t.y, &t.alpha,
                 &t.extra) != 5) {
        vs_log_error(td->conf.modName, "Cannot parse line: %s", l);
        return 0;
      }
      t.zoom=0;
    }

    if (i>=s) { // resize transform array
      if (s == 0)
        s = 256;
      else
        s*=2;
      /* vs_log_info(td->modName, "resize: %i\n", s); */
      trans->ts = vs_realloc(trans->ts, sizeof(VSTransform)* s);
      if (!trans->ts) {
        vs_log_error(td->conf.modName, "Cannot allocate memory"
                     " for transformations: %i\n", s);
        return 0;
      }
    }
    trans->ts[i] = t;
    i++;
  }
  trans->len = i;

  return i;
}


//     t = vsSimpleMotionsToTransform(md, &localmotions);


/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
