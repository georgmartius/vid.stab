/*
 *  filter_deshake.c
 *
 *  Copyright (C) Georg Martius - November 2011
 *   georg dot martius at web dot de 
 * 
 *   This program is free software; you can redistribute it and/or modify 
 *   it under the terms of the GNU General Public License as published by 
 *   the Free Software Foundation; either version 2 of the License, or 
 *   (at your option) any later version. 
 * 
 *   This program is distributed in the hope that it will be useful, 
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 *   GNU General Public License for more details. 
 * 
 *   You should have received a copy of the GNU General Public License 
 *   along with this program; if not, write to the 
 *   Free Software Foundation, Inc., 
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA. 
 *
 */

/* Typical call:
 *  transcode -V -J deshake=shakiness=5:smoothing=10 
 *         -i inp.mpeg -y xvid,tc_aud -o out.avi
 *  all parameters are optional
 */

#define MOD_NAME    "filter_deshake.so"
#define MOD_VERSION "v0.82 (2011-08-01)"
#define MOD_CAP     "deshakes a video clip by extracting relative transformations\n\
    of subsequent frames and transforms the high-frequency away\n\
    This is a single pass verion of stabilize and transform plugin"
#define MOD_AUTHOR  "Georg Martius"


#define MOD_FEATURES                                    \
  TC_MODULE_FEATURE_FILTER|TC_MODULE_FEATURE_VIDEO
#define MOD_FLAGS					\
  TC_MODULE_FLAG_RECONFIGURABLE | TC_MODULE_FLAG_DELAY

#define DEFAULT_TRANS_FILE_NAME     "transforms.dat"


#include <math.h>
#include <libgen.h>
  
#include "transcode.h"
#include "filter.h"
#include "libtc/libtc.h"
#include "libtc/optstr.h"
#include "libtc/tccodecs.h"
#include "libtc/tcmodule-plugin.h"

#include "libdeshake.h"
#include "transformtype.h"
#include "frameinfo.h"
#include "deshakedefines.h"
#include "transform.h"
#include "transformfixedpoint.h"


/* private date structure of this filter*/
typedef struct _deshake_data {
  MotionDetect md;
  TransformData td;
  SlidingAvgTrans avg;
  
  vob_t* vob;  // pointer to information structure
  char* result;
  FILE* f;
  
  char conf_str[TC_BUF_MIN];
} DeshakeData;

static const char deshake_help[] = ""
  "Overview:\n"
  "    Deshakes a video clip. It only uses past information, such that it is less\n"
  "     powerful than the filters stabilize and transform. \n"
  "     It also generates a file with relative transform information\n"
  "     to be used by the transform filter separately."
  "Options\n"
  "    'smoothing' number of frames*2 + 1 used for lowpass filtering \n"
  "                used for stabilizing (def: 10)\n"
  "    'shakiness'   how shaky is the video and how quick is the camera?\n"
  "                  1: little (fast) 10: very strong/quick (slow) (def: 4)\n"
  "    'accuracy'    accuracy of detection process (>=shakiness)\n"
  "                  1: low (fast) 15: high (slow) (def: 4)\n"
  "    'stepsize'    stepsize of search process, region around minimum \n"
  "                  is scanned with 1 pixel resolution (def: 6)\n"
  "    'algo'        0: brute force (translation only);\n"
  "                  1: small measurement fields (def)\n"
  "    'mincontrast' below this contrast a field is discarded (0-1) (def: 0.3)\n"
  "    'result'      path to the file used to write the transforms\n"
  "                  (def:inputfile.stab)\n"
  "    'maxshift'    maximal number of pixels to translate image\n"
  "                  (def: -1 no limit)\n"
  "    'maxangle'    maximal angle in rad to rotate image (def: -1 no limit)\n"
  "    'crop'        0: keep border (def), 1: black background\n"
  "    'zoom'        percentage to zoom >0: zoom in, <0 zoom out (def: 2)\n"
  "    'optzoom'     0: nothing, 1: determine optimal zoom (def)\n"
  "    'interpol'    type of interpolation: 0: no interpolation, \n"
  "                  1: linear (horizontal), 2: bi-linear (def), \n"
  "                  3: bi-cubic\n"
  "    'sharpen'     amount of sharpening: 0: no sharpening (def: 0.8)\n"
  "                  uses filter unsharp with 5x5 matrix\n"
  "    'help'        print this help message\n";

/*************************************************************************/

/* Module interface routines and data. */

/*************************************************************************/

/**
 * deshake_init:  Initialize this instance of the module.  See
 * tcmodule-data.h for function details.
 */

static int deshake_init(TCModuleInstance *self, uint32_t features)
{
  DeshakeData* sd = NULL;
  TC_MODULE_SELF_CHECK(self, "init");
  TC_MODULE_INIT_CHECK(self, MOD_FEATURES, features);

  sd = tc_zalloc(sizeof(DeshakeData)); // allocation with zero values
  if (!sd) {
    if (verbose > TC_INFO)
      tc_log_error(MOD_NAME, "init: out of memory!");
    return TC_ERROR;
  }

  sd->vob = tc_get_vob();
  if (!sd->vob)
    return TC_ERROR;

  /**** Initialise private data structure */

  self->userdata = sd;
  if (verbose & TC_INFO){
    tc_log_info(MOD_NAME, "%s %s", MOD_VERSION, MOD_CAP);
  }

  return TC_OK;
}


/*
 * deshake_fini:  Clean up after this instance of the module.  See
 * tcmodule-data.h for function details.
 */
static int deshake_fini(TCModuleInstance *self)
{
  DeshakeData *sd = NULL;
  TC_MODULE_SELF_CHECK(self, "fini");
  sd = self->userdata;

  tc_free(sd);
  self->userdata = NULL;
  return TC_OK;
}

/*
 * deshake_configure:  Configure this instance of the module.  See
 * tcmodule-data.h for function details.
 */
static int deshake_configure(TCModuleInstance *self,
			     const char *options, vob_t *vob)
{
  DeshakeData *sd = NULL;
  TC_MODULE_SELF_CHECK(self, "configure");
  char* filenamecopy, *filebasename;

  sd = self->userdata;

  /*    sd->framesize = sd->vob->im_v_width * MAX_PLANES * 
	sizeof(char) * 2 * sd->vob->im_v_height * 2;     */

  MotionDetect* md = &(sd->md);
  TransformData* td = &(sd->td);

  // init MotionDetect part
  DSFrameInfo fi;
  fi.width=sd->vob->ex_v_width;
  fi.height=sd->vob->ex_v_height;
  fi.strive=sd->vob->ex_v_width;
  fi.framesize=sd->vob->im_v_size;
  fi.pFormat = vob->im_v_codec;
  if(initMotionDetect(md, &fi, MOD_NAME) != DS_OK){
    tc_log_error(MOD_NAME, "initialization of Motion Detection failed");
    return TC_ERROR;
  }
    
  sd->result = tc_malloc(TC_BUF_LINE);
  filenamecopy = tc_strdup(sd->vob->video_in_file);
  filebasename = basename(filenamecopy);
  if (strlen(filebasename) < TC_BUF_LINE - 4) {
    tc_snprintf(sd->result, TC_BUF_LINE, "%s.trf", filebasename);
  } else {
    tc_log_warn(MOD_NAME, "input name too long, using default `%s'",
		DEFAULT_TRANS_FILE_NAME);
    tc_snprintf(sd->result, TC_BUF_LINE, DEFAULT_TRANS_FILE_NAME);
  }

  // init trasform part
  DSFrameInfo fi_dest;
  /* Todo: in case we can scale the images, calc new size later */
  fi_dest.width     = sd->vob->ex_v_width;
  fi_dest.height    = sd->vob->ex_v_height;
  fi_dest.framesize = sd->vob->im_v_size;
  fi_dest.strive    =	sd->vob->ex_v_width;

  if(initTransformData(td, &fi, &fi_dest, MOD_NAME) != DS_OK){
    tc_log_error(MOD_NAME, "initialization of TransformData failed");
    return TC_ERROR;
  }
  td->verbose=verbose;


  if (options != NULL) {            
    // for some reason this plugin is called in the old fashion 
    //  (not with inspect). Anyway we support both ways of getting help.
    if(optstr_lookup(options, "help")) {
      tc_log_info(MOD_NAME,deshake_help);
      return(TC_IMPORT_ERROR);
    }

    optstr_get(options, "result",     "%[^:]", sd->result);
    optstr_get(options, "shakiness",  "%d", &md->shakiness);
    optstr_get(options, "accuracy",   "%d", &md->accuracy);
    optstr_get(options, "stepsize",   "%d", &md->stepSize);
    optstr_get(options, "algo",       "%d", &md->algo);
    optstr_get(options, "mincontrast","%lf",&md->contrastThreshold);
    md->show = 0;

    optstr_get(options, "maxshift",  "%d", &td->maxShift);
    optstr_get(options, "maxangle",  "%lf", &td->maxAngle);
    optstr_get(options, "smoothing", "%d", &td->smoothing);
    optstr_get(options, "crop"     , "%d", &td->crop);
    optstr_get(options, "zoom"     , "%lf",&td->zoom);
    optstr_get(options, "optzoom"  , "%d", &td->optZoom);
    optstr_get(options, "interpol" , "%d", (int*)(&td->interpolType));
    optstr_get(options, "sharpen"  , "%lf",&td->sharpen);
    td->relative=1;
    td->invert=0;
  }

  if(configureMotionDetect(md)!= DS_OK){
    tc_log_error(MOD_NAME, "configuration of Motion Detection failed");
    return TC_ERROR;
  }
  if(configureTransformData(td)!= DS_OK){
    tc_log_error(MOD_NAME, "configuration of Tranform failed");
    return TC_ERROR;
  }
    
  if (verbose) {
    tc_log_info(MOD_NAME, "Video Deshake  Settings:");
    tc_log_info(MOD_NAME, "    smoothing = %d", td->smoothing);
    tc_log_info(MOD_NAME, "    shakiness = %d", md->shakiness);
    tc_log_info(MOD_NAME, "     accuracy = %d", md->accuracy);
    tc_log_info(MOD_NAME, "     stepsize = %d", md->stepSize);
    tc_log_info(MOD_NAME, "         algo = %d", md->algo);
    tc_log_info(MOD_NAME, "  mincontrast = %f", md->contrastThreshold);
    tc_log_info(MOD_NAME, "         show = %d", md->show);
    tc_log_info(MOD_NAME, "       result = %s", sd->result);
    tc_log_info(MOD_NAME, "    maxshift  = %d", td->maxShift);
    tc_log_info(MOD_NAME, "    maxangle  = %f", td->maxAngle);
    tc_log_info(MOD_NAME, "         crop = %s", 
		td->crop ? "Black" : "Keep");
    tc_log_info(MOD_NAME, "         zoom = %f", td->zoom);
    tc_log_info(MOD_NAME, "      optzoom = %s", 
		td->optZoom ? "On" : "Off");
    tc_log_info(MOD_NAME, "     interpol = %s", 
		interpolTypes[td->interpolType]);
    tc_log_info(MOD_NAME, "      sharpen = %f", td->sharpen);

  }

  sd->avg.initialized=0;

  sd->f = fopen(sd->result, "w");
  if (sd->f == NULL) {
    tc_log_error(MOD_NAME, "cannot open result file %s!\n", sd->result);
    return TC_ERROR;
  }    

  return TC_OK;
}


/**
 * deshake_filter_video: performs the analysis of subsequent frames
 * See tcmodule-data.h for function details.
 */

static int deshake_filter_video(TCModuleInstance *self, 
				vframe_list_t *frame)
{
  DeshakeData *sd = NULL;
  
  TC_MODULE_SELF_CHECK(self, "filter_video");
  TC_MODULE_SELF_CHECK(frame, "filter_video");
  
  sd = self->userdata;    
  MotionDetect* md = &(sd->md);
  Transform motion;
  if(motionDetection(md, &motion, frame->video_buf)!= DS_OK){
    tc_log_error(MOD_NAME, "motion detection failed");
    return TC_ERROR;
  } else {
    fprintf(sd->f, "%i %6.4lf %6.4lf %8.5lf %6.4lf %i\n",
	    md->frameNum, motion.x, motion.y, motion.alpha, motion.zoom, 
	    motion.extra);
  }

    
    
  transformPrepare(&sd->td, frame->video_buf);  
    
  Transform t = lowPassTransforms(&sd->td, &sd->avg, &motion);
  tc_log_error(MOD_NAME, "Trans: det: %f %f %f \n\t\t act: %f %f %f %f", 
	       motion.x, motion.y, motion.alpha,
	       t.x, t.y, t.alpha, t.zoom);
                     
  if (sd->vob->im_v_codec == CODEC_RGB) {
    transformRGB(&sd->td, t);
  } else if (sd->vob->im_v_codec == CODEC_YUV) {
    transformYUV(&sd->td, t);
  } else {
    tc_log_error(MOD_NAME, "unsupported Codec: %i\n", sd->vob->im_v_codec);
    return TC_ERROR;
  }
  transformFinish(&sd->td);
  return TC_OK;    
}

/**
 * deshake_stop:  Reset this instance of the module.  See tcmodule-data.h
 * for function details.
 */

static int deshake_stop(TCModuleInstance *self)
{
  DeshakeData *sd = NULL;
  TC_MODULE_SELF_CHECK(self, "stop");
  sd = self->userdata;
  // print transs
  if (sd->f) {
    fclose(sd->f);
    sd->f = NULL;
  }

  cleanupMotionDetection(&sd->md);
  if (sd->result) {
    tc_free(sd->result);
    sd->result = NULL;
  }

  cleanupTransformData(&sd->td);
    
  return TC_OK;
}

/* checks for parameter in function _inspect */
#define CHECKPARAM(paramname, formatstring, variable)   \
  if (optstr_lookup(param, paramname)) {		\
    tc_snprintf(sd->conf_str, sizeof(sd->conf_str),	\
		formatstring, variable);		\
    *value = sd->conf_str;				\
  }

/**
 * deshake_inspect:  Return the value of an option in this instance of
 * the module.  See tcmodule-data.h for function details.
 */

static int deshake_inspect(TCModuleInstance *self,
			   const char *param, const char **value)
{
  DeshakeData *sd = NULL;
    
  TC_MODULE_SELF_CHECK(self, "inspect");
  TC_MODULE_SELF_CHECK(param, "inspect");
  TC_MODULE_SELF_CHECK(value, "inspect");
  sd = self->userdata;
  MotionDetect* md = &(sd->md);
  if (optstr_lookup(param, "help")) {
    *value = deshake_help;
  }
  CHECKPARAM("shakiness","shakiness=%d", md->shakiness);
  CHECKPARAM("accuracy", "accuracy=%d",  md->accuracy);
  CHECKPARAM("stepsize", "stepsize=%d",  md->stepSize);
  CHECKPARAM("allowmax", "allowmax=%d",  md->allowMax);
  CHECKPARAM("algo",     "algo=%d",      md->algo);
  CHECKPARAM("result",   "result=%s",    sd->result);
  CHECKPARAM("maxshift", "maxshift=%d",  sd->td.maxShift);
  CHECKPARAM("maxangle", "maxangle=%f",  sd->td.maxAngle);
  CHECKPARAM("smoothing","smoothing=%d", sd->td.smoothing);
  CHECKPARAM("crop",     "crop=%d",      sd->td.crop);
  CHECKPARAM("optzoom",  "optzoom=%i",   sd->td.optZoom);
  CHECKPARAM("zoom",     "zoom=%f",      sd->td.zoom);
  CHECKPARAM("sharpen",  "sharpen=%f",   sd->td.sharpen);

  return TC_OK;
}

static const TCCodecID deshake_codecs_in[] = { 
  TC_CODEC_YUV420P, TC_CODEC_YUV422P, TC_CODEC_RGB, TC_CODEC_ERROR 
};
static const TCCodecID deshake_codecs_out[] = { 
  TC_CODEC_YUV420P, TC_CODEC_YUV422P, TC_CODEC_RGB, TC_CODEC_ERROR 
};
TC_MODULE_FILTER_FORMATS(deshake); 

TC_MODULE_INFO(deshake);

static const TCModuleClass deshake_class = {
  TC_MODULE_CLASS_HEAD(deshake),

  .init         = deshake_init,
  .fini         = deshake_fini,
  .configure    = deshake_configure,
  .stop         = deshake_stop,
  .inspect      = deshake_inspect,

  .filter_video = deshake_filter_video,
};

TC_MODULE_ENTRY_POINT(deshake)

/*************************************************************************/

static int deshake_get_config(TCModuleInstance *self, char *options)
{
  TC_MODULE_SELF_CHECK(self, "get_config");

  optstr_filter_desc(options, MOD_NAME, MOD_CAP, MOD_VERSION,
		     MOD_AUTHOR, "VRY4", "1");

  return TC_OK;
}

static int deshake_process(TCModuleInstance *self, frame_list_t *frame)
{
  TC_MODULE_SELF_CHECK(self, "process");

  //    if (frame->tag & TC_PRE_S_PROCESS && frame->tag & TC_VIDEO) {
  if (frame->tag & TC_POST_S_PROCESS && frame->tag & TC_VIDEO) {
    return deshake_filter_video(self, (vframe_list_t *)frame);
  }
  return TC_OK;
}

/*************************************************************************/

TC_FILTER_OLDINTERFACE(deshake)

/*************************************************************************/

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
