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

const char* modname = "vid.stab - serialization";


int storeLocalmotion(FILE* f, const LocalMotion* lm){
  return fprintf(f,"(LM %i %i %i %i %i %lf %lf)", lm->v.x,lm->v.y,lm->f.x,lm->f.y,lm->f.size,
                 lm->contrast, lm->match);
}

/// restore local motion from file
LocalMotion restoreLocalmotion(FILE* f){
  LocalMotion lm;
  char c;
  if(fscanf(f,"(LM %i %i %i %i %i %lf %lf", &lm.v.x,&lm.v.y,&lm.f.x,&lm.f.y,&lm.f.size,
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

int vsStoreLocalmotions(FILE* f, const LocalMotions* lms){
  int len = vs_vector_size(lms);
  int i;
  fprintf(f,"List %i [",len);
  for (i=0; i<len; i++){
    if(i>0) fprintf(f,",");
    if(storeLocalmotion(f,LMGet(lms,i)) <= 0) return 0;
  }
  fprintf(f,"]");
  return 1;
}

/// restores local motions from file
LocalMotions vsRestoreLocalmotions(FILE* f){
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
      LocalMotion lm = restoreLocalmotion(f);
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

int vsPrepareFile(const VSMotionDetect* md, FILE* f){
    if(!f) return VS_ERROR;
    fprintf(f, "VID.STAB 1\n");
    fprintf(f, "#      accuracy = %d\n", md->conf.accuracy);
    fprintf(f, "#     shakiness = %d\n", md->conf.shakiness);
    fprintf(f, "#      stepsize = %d\n", md->conf.stepSize);
    fprintf(f, "#   mincontrast = %f\n", md->conf.contrastThreshold);
    return VS_OK;
}

int vsWriteToFile(const VSMotionDetect* md, FILE* f, const LocalMotions* lms){
  if(!f || !lms) return VS_ERROR;

  if(fprintf(f, "Frame %i (", md->frameNum)>0
     && vsStoreLocalmotions(f,lms)>0 && fprintf(f, ")\n"))
    return VS_OK;
  else
    return VS_ERROR;
}

/// reads the header of the file and return the version number
int vsReadFileVersion(FILE* f){
  if(!f) return VS_ERROR;
  int version;
  if(fscanf(f, "VID.STAB %i\n", &version)!=1)
    return VS_ERROR;
  else return version;
}

int vsReadFromFile(FILE* f, LocalMotions* lms){
  char c = fgetc(f);
  if(c=='F') {
    int num;
    if(fscanf(f,"rame %i (", &num)!=1) {
      vs_log_error(modname,"cannot read file, expect 'Frame num (...'");
      return VS_ERROR;
    }
    *lms = vsRestoreLocalmotions(f);
    if(fscanf(f,")\n")<0) {
      vs_log_error(modname,"cannot read file, expect '...)'");
      return VS_ERROR;
    }
    return num;
  } else if(c=='#') {
    char l[1024];
    if(fgets(l, sizeof(l), f)==0) return VS_ERROR;
    return vsReadFromFile(f,lms);
  } else if(c=='\n' || c==' ') {
    return vsReadFromFile(f,lms);
  } else if(c==EOF) {
    return VS_ERROR;
  } else {
    vs_log_error(modname,"cannot read frame local motions from file, got %c (%i)",
                 c, (int) c);
    return VS_ERROR;
  }
}

int vsReadLocalMotionsFile(FILE* f, VSManyLocalMotions* mlms){
  int version = vsReadFileVersion(f);
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
  while((index = vsReadFromFile(f,&lms)) != VS_ERROR){
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
