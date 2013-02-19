/*
 *  filter_stabilize.c
 *
 *  Copyright (C) Georg Martius - June 2007
 *   georg dot martius at web dot de
 *
 *  This file is part of transcode, a video stream processing tool
 *
 *  transcode is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  transcode is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with GNU Make; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/* Typical call:
 *  transcode -V -J stabilize=shakiness=5:show=1,preview
 *         -i inp.mpeg -y null,null -o dummy
 *  all parameters are optional
 */

#include "libdeshake.h"

#define MOD_NAME    "filter_stabilize.so"
#define MOD_VERSION LIBDESHAKE_VERSION
#define MOD_CAP     "extracts relative transformations of \n\
    subsequent frames (used for stabilization together with the\n\
    transform filter in a second pass)"
#define MOD_AUTHOR  "Georg Martius"

/* Ideas:
   - Try OpenCL/Cuda, this should work great
   - use smoothing on the frames and then use gradient decent!
   - stepsize could be adapted (maybe to check only one field with large
   stepsize and use the maximally required for the other fields
*/

#define MOD_FEATURES                                    \
    TC_MODULE_FEATURE_FILTER|TC_MODULE_FEATURE_VIDEO
#define MOD_FLAGS                                               \
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

#include "pix_formats.h"

/* private date structure of this filter*/
typedef struct _stab_data {
    MotionDetect md;
    vob_t* vob;  // pointer to information structure

    char* result;
    FILE* f;

    char conf_str[TC_BUF_MIN];
} StabData;

/*************************************************************************/

/* Module interface routines and data. */

/*************************************************************************/

/**
 * stabilize_init:  Initialize this instance of the module.  See
 * tcmodule-data.h for function details.
 */

static int stabilize_init(TCModuleInstance *self, uint32_t features)
{
    StabData* sd = NULL;
    TC_MODULE_SELF_CHECK(self, "init");
    TC_MODULE_INIT_CHECK(self, MOD_FEATURES, features);

    sd = tc_zalloc(sizeof(StabData)); // allocation with zero values
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
 * stabilize_fini:  Clean up after this instance of the module.  See
 * tcmodule-data.h for function details.
 */
static int stabilize_fini(TCModuleInstance *self)
{
    StabData *sd = NULL;
    TC_MODULE_SELF_CHECK(self, "fini");
    sd = self->userdata;

    tc_free(sd);
    self->userdata = NULL;
    return TC_OK;
}

/*
 * stabilize_configure:  Configure this instance of the module.  See
 * tcmodule-data.h for function details.
 */
static int stabilize_configure(TCModuleInstance *self,
                               const char *options, vob_t *vob)
{
    StabData *sd = NULL;
    TC_MODULE_SELF_CHECK(self, "configure");
    char* filenamecopy, *filebasename;

    sd = self->userdata;

    /*    sd->framesize = sd->vob->im_v_width * MAX_PLANES *
          sizeof(char) * 2 * sd->vob->im_v_height * 2;     */

    MotionDetect* md = &(sd->md);
    DSFrameInfo fi;
    initFrameInfo(&fi, sd->vob->ex_v_width, sd->vob->ex_v_height,
                  transcode2ourPF(vob->im_v_codec));

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

    if (options != NULL) {
        // for some reason this plugin is called in the old fashion
        //  (not with inspect). Anyway we support both ways of getting help.
        if(optstr_lookup(options, "help")) {
            tc_log_info(MOD_NAME,motiondetect_help);
            return(TC_IMPORT_ERROR);
        }

        optstr_get(options, "result",     "%[^:]", sd->result);
        optstr_get(options, "shakiness",  "%d", &md->shakiness);
        optstr_get(options, "accuracy",   "%d", &md->accuracy);
        optstr_get(options, "stepsize",   "%d", &md->stepSize);
        optstr_get(options, "algo",       "%d", &md->algo);
        optstr_get(options, "mincontrast","%lf",&md->contrastThreshold);
        optstr_get(options, "tripod",     "%d", &md->virtualTripod);
        optstr_get(options, "show",       "%d", &md->show);
    }

    if(configureMotionDetect(md)!= DS_OK){
    	tc_log_error(MOD_NAME, "configuration of Motion Detection failed");
        return TC_ERROR;
    }

    if (verbose) {
        tc_log_info(MOD_NAME, "Image Stabilization Settings:");
        tc_log_info(MOD_NAME, "     shakiness = %d", md->shakiness);
        tc_log_info(MOD_NAME, "      accuracy = %d", md->accuracy);
        tc_log_info(MOD_NAME, "      stepsize = %d", md->stepSize);
        tc_log_info(MOD_NAME, "          algo = %d", md->algo);
        tc_log_info(MOD_NAME, "   mincontrast = %f", md->contrastThreshold);
        tc_log_info(MOD_NAME, "        tripod = %d", md->virtualTripod);
        tc_log_info(MOD_NAME, "          show = %d", md->show);
        tc_log_info(MOD_NAME, "        result = %s", sd->result);
    }

    sd->f = fopen(sd->result, "w");
    if (sd->f == NULL) {
        tc_log_error(MOD_NAME, "cannot open result file %s!\n", sd->result);
        return TC_ERROR;
    }else{
        if(prepareFile(md, sd->f) != DS_OK){
            tc_log_error(MOD_NAME, "cannot write to result file %s", sd->result);
            return TC_ERROR;
        }
    }

    /***** This is now done by boxblur ****/
    /* /\* load unsharp filter to smooth the frames. This allows larger stepsize.*\/ */
    /* char unsharp_param[128]; */
    /* int masksize = TC_MIN(13,md->stepSize*1.5); // only works up to 13. */
    /* sprintf(unsharp_param,"luma=-1:luma_matrix=%ix%i:pre=1", */
    /*         masksize, masksize); */
    /* if (!tc_filter_add("unsharp", unsharp_param)) { */
    /*     tc_log_warn(MOD_NAME, "cannot load unsharp filter!"); */
    /* } */

    return TC_OK;
}


/**
 * stabilize_filter_video: performs the analysis of subsequent frames
 * See tcmodule-data.h for function details.
 */

static int stabilize_filter_video(TCModuleInstance *self,
                                  vframe_list_t *frame)
{
    StabData *sd = NULL;

    TC_MODULE_SELF_CHECK(self, "filter_video");
    TC_MODULE_SELF_CHECK(frame, "filter_video");

    sd = self->userdata;
    MotionDetect* md = &(sd->md);
    LocalMotions localmotions;
    DSFrame dsFrame;
    fillFrameFromBuffer(&dsFrame,frame->video_buf, &md->fi);

    if(motionDetection(md, &localmotions, &dsFrame)!= DS_OK){
    	tc_log_error(MOD_NAME, "motion detection failed");
    	return TC_ERROR;
    }
    if(writeToFile(md, sd->f, &localmotions) != DS_OK){
        ds_vector_del(&localmotions);
        return TC_ERROR;
    } else {
        ds_vector_del(&localmotions);
        return TC_OK;
    }
}

/**
 * stabilize_stop:  Reset this instance of the module.  See tcmodule-data.h
 * for function details.
 */

static int stabilize_stop(TCModuleInstance *self)
{
    StabData *sd = NULL;
    TC_MODULE_SELF_CHECK(self, "stop");
    sd = self->userdata;
    MotionDetect* md = &(sd->md);
    if (sd->f) {
        fclose(sd->f);
        sd->f = NULL;
    }

    cleanupMotionDetection(md);
    if (sd->result) {
        tc_free(sd->result);
        sd->result = NULL;
    }
    return TC_OK;
}

/* checks for parameter in function _inspect */
#define CHECKPARAM(paramname, formatstring, variable)   \
    if (optstr_lookup(param, paramname)) {              \
        tc_snprintf(sd->conf_str, sizeof(sd->conf_str), \
                    formatstring, variable);            \
        *value = sd->conf_str;                          \
    }

/**
 * stabilize_inspect:  Return the value of an option in this instance of
 * the module.  See tcmodule-data.h for function details.
 */

static int stabilize_inspect(TCModuleInstance *self,
			     const char *param, const char **value)
{
    StabData *sd = NULL;

    TC_MODULE_SELF_CHECK(self, "inspect");
    TC_MODULE_SELF_CHECK(param, "inspect");
    TC_MODULE_SELF_CHECK(value, "inspect");
    sd = self->userdata;
    MotionDetect* md = &(sd->md);
    if (optstr_lookup(param, "help")) {
        *value = motiondetect_help;
    }
    CHECKPARAM("shakiness","shakiness=%d", md->shakiness);
    CHECKPARAM("accuracy", "accuracy=%d",  md->accuracy);
    CHECKPARAM("stepsize", "stepsize=%d",  md->stepSize);
    CHECKPARAM("allowmax", "allowmax=%d",  md->allowMax);
    CHECKPARAM("algo",     "algo=%d",      md->algo);
    CHECKPARAM("tripod",   "tripod=%d",    md->virtualTripod);
    CHECKPARAM("show",     "show=%d",      md->show);
    CHECKPARAM("result",   "result=%s",    sd->result);
    return TC_OK;
}

static const TCCodecID stabilize_codecs_in[] = {
    TC_CODEC_YUV420P, TC_CODEC_YUV422P, TC_CODEC_RGB, TC_CODEC_ERROR
};
static const TCCodecID stabilize_codecs_out[] = {
    TC_CODEC_YUV420P, TC_CODEC_YUV422P, TC_CODEC_RGB, TC_CODEC_ERROR
};
TC_MODULE_FILTER_FORMATS(stabilize);

TC_MODULE_INFO(stabilize);

static const TCModuleClass stabilize_class = {
    TC_MODULE_CLASS_HEAD(stabilize),

    .init         = stabilize_init,
    .fini         = stabilize_fini,
    .configure    = stabilize_configure,
    .stop         = stabilize_stop,
    .inspect      = stabilize_inspect,

    .filter_video = stabilize_filter_video,
};

TC_MODULE_ENTRY_POINT(stabilize)

/*************************************************************************/

static int stabilize_get_config(TCModuleInstance *self, char *options)
{
    TC_MODULE_SELF_CHECK(self, "get_config");

    optstr_filter_desc(options, MOD_NAME, MOD_CAP, MOD_VERSION,
                       MOD_AUTHOR, "VRY4", "1");

    return TC_OK;
}

static int stabilize_process(TCModuleInstance *self, frame_list_t *frame)
{
    TC_MODULE_SELF_CHECK(self, "process");

//    if (frame->tag & TC_PRE_S_PROCESS && frame->tag & TC_VIDEO) {
    if (frame->tag & TC_POST_S_PROCESS && frame->tag & TC_VIDEO) {
        return stabilize_filter_video(self, (vframe_list_t *)frame);
    }
    return TC_OK;
}

/*************************************************************************/

TC_FILTER_OLDINTERFACE(stabilize)

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
