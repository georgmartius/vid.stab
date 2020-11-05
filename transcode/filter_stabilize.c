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

#include "libvidstab.h"

#define MOD_NAME    "filter_stabilize.so"
#define MOD_VERSION LIBVIDSTAB_VERSION
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

#include "transcode_specifics.h"

/* private date structure of this filter*/
typedef struct _stab_data {
    VSMotionDetect md;
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

    setLogFunctions();

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

    VSMotionDetect* md = &(sd->md);
    VSFrameInfo fi;
    vsFrameInfoInit(&fi, sd->vob->ex_v_width, sd->vob->ex_v_height,
                  transcode2ourPF(vob->im_v_codec));

    VSMotionDetectConfig conf = vsMotionDetectGetDefaultConfig(MOD_NAME);

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
            tc_log_info(MOD_NAME,vs_motiondetect_help);
            return(TC_IMPORT_ERROR);
        }

        optstr_get(options, "fileformat", "%d", &md->serializationMode);
        optstr_get(options, "result",     "%[^:]", sd->result);
        optstr_get(options, "shakiness",  "%d", &conf.shakiness);
        optstr_get(options, "accuracy",   "%d", &conf.accuracy);
        optstr_get(options, "stepsize",   "%d", &conf.stepSize);
        optstr_get(options, "algo",       "%d", &conf.algo);
        optstr_get(options, "mincontrast","%lf",&conf.contrastThreshold);
        optstr_get(options, "tripod",     "%d", &conf.virtualTripod);
        optstr_get(options, "show",       "%d", &conf.show);
    }

    if(vsMotionDetectInit(md, &conf, &fi) != VS_OK){
        tc_log_error(MOD_NAME, "initialization of Motion Detection failed");
        return TC_ERROR;
    }
    vsMotionDetectGetConfig(&conf,md);

    if (verbose) {
        tc_log_info(MOD_NAME, "Image Stabilization Settings:");
        tc_log_info(MOD_NAME, "     shakiness = %d", conf.shakiness);
        tc_log_info(MOD_NAME, "      accuracy = %d", conf.accuracy);
        tc_log_info(MOD_NAME, "      stepsize = %d", conf.stepSize);
        tc_log_info(MOD_NAME, "          algo = %d", conf.algo);
        tc_log_info(MOD_NAME, "   mincontrast = %f", conf.contrastThreshold);
        tc_log_info(MOD_NAME, "        tripod = %d", conf.virtualTripod);
        tc_log_info(MOD_NAME, "          show = %d", conf.show);
        tc_log_info(MOD_NAME, "        result = %s", sd->result);
    }

    sd->f = fopen(sd->result, "w");
    if (sd->f == NULL) {
        tc_log_error(MOD_NAME, "cannot open result file %s!\n", sd->result);
        return TC_ERROR;
    }else{
        if(vsPrepareFile(md, sd->f) != VS_OK){
            tc_log_error(MOD_NAME, "cannot write to result file %s", sd->result);
            return TC_ERROR;
        }
    }

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
    VSMotionDetect* md = &(sd->md);
    LocalMotions localmotions;
    VSFrame vsFrame;
    vsFrameFillFromBuffer(&vsFrame,frame->video_buf, &md->fi);

    if(vsMotionDetection(md, &localmotions, &vsFrame)!= VS_OK){
      tc_log_error(MOD_NAME, "motion detection failed");
      return TC_ERROR;
    }
    if(vsWriteToFile(md, sd->f, &localmotions) != VS_OK){
        vs_vector_del(&localmotions);
        return TC_ERROR;
    } else {
        vs_vector_del(&localmotions);
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
    VSMotionDetect* md = &(sd->md);
    if (sd->f) {
        fclose(sd->f);
        sd->f = NULL;
    }

    vsMotionDetectionCleanup(md);
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
    VSMotionDetect* md = &(sd->md);
    if (optstr_lookup(param, "help")) {
        *value = vs_motiondetect_help;
    }
    VSMotionDetectConfig conf;
    vsMotionDetectGetConfig(&conf,md);

    CHECKPARAM("shakiness","shakiness=%d", conf.shakiness);
    CHECKPARAM("accuracy", "accuracy=%d",  conf.accuracy);
    CHECKPARAM("stepsize", "stepsize=%d",  conf.stepSize);
    CHECKPARAM("algo",     "algo=%d",      conf.algo);
    CHECKPARAM("tripod",   "tripod=%d",    conf.virtualTripod);
    CHECKPARAM("show",     "show=%d",      conf.show);
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
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
