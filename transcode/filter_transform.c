/*
 *  filter_transform.c
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
 * Typical call:
 * transcode -J transform -i inp.mpeg -y xdiv,tcaud inp_stab.avi
*/

#include "libvidstab.h"

#define MOD_NAME    "filter_transform.so"
#define MOD_VERSION LIBVIDSTAB_VERSION
#define MOD_CAP     "transforms each frame according to transformations\n\
 given in an input file (e.g. translation, rotate) see also filter stabilize"
#define MOD_AUTHOR  "Georg Martius"
#define MOD_FEATURES \
    TC_MODULE_FEATURE_FILTER|TC_MODULE_FEATURE_VIDEO
#define MOD_FLAGS \
    TC_MODULE_FLAG_RECONFIGURABLE

#include "transcode.h"
#include "filter.h"

#include "libtc/libtc.h"
#include "libtc/optstr.h"
#include "libtc/tccodecs.h"
#include "libtc/tcmodule-plugin.h"

#include "transcode_specifics.h"

#define DEFAULT_TRANS_FILE_NAME     "transforms.dat"

typedef struct {
    TransformData td;
    vob_t* vob;          // pointer to information structure

    Transformations trans; // transformations

    char input[TC_BUF_LINE];
    char conf_str[TC_BUF_MIN];
} FilterData;

/**
 * transform_init:  Initialize this instance of the module.  See
 * tcmodule-data.h for function details.
 */
static int transform_init(TCModuleInstance *self, uint32_t features)
{
    FilterData* fd = NULL;
    TC_MODULE_SELF_CHECK(self, "init");
    TC_MODULE_INIT_CHECK(self, MOD_FEATURES, features);

    setLogFunctions();

    fd = tc_zalloc(sizeof(FilterData));
    if (fd == NULL) {
        tc_log_error(MOD_NAME, "init: out of memory!");
        return TC_ERROR;
    }
    self->userdata = fd;
    if (verbose) {
        tc_log_info(MOD_NAME, "%s %s", MOD_VERSION, MOD_CAP);
    }

    return TC_OK;
}


/**
 * transform_configure:  Configure this instance of the module.  See
 * tcmodule-data.h for function details.
 */
static int transform_configure(TCModuleInstance *self,
			       const char *options, vob_t *vob)
{
    FilterData *fd = NULL;
    char* filenamecopy, *filebasename;
    FILE* f;
    TC_MODULE_SELF_CHECK(self, "configure");

    fd = self->userdata;
    TransformData* td = &(fd->td);

    fd->vob = vob;
    if (!fd->vob)
        return TC_ERROR; /* cannot happen */

    /**** Initialise private data structure */

    VSFrameInfo fi_src;
    VSFrameInfo fi_dest;
    initFrameInfo(&fi_src, fd->vob->ex_v_width, fd->vob->ex_v_height,
                  transcode2ourPF(fd->vob->im_v_codec));
    initFrameInfo(&fi_dest, fd->vob->ex_v_width, fd->vob->ex_v_height,
                  transcode2ourPF(fd->vob->im_v_codec));

    if(initTransformData(td, &fi_src, &fi_dest, MOD_NAME) != VS_OK){
        tc_log_error(MOD_NAME, "initialization of TransformData failed");
        return TC_ERROR;
    }
    td->verbose=verbose;

    initTransformations(&fd->trans);

    filenamecopy = tc_strdup(fd->vob->video_in_file);
    filebasename = basename(filenamecopy);
    if (strlen(filebasename) < TC_BUF_LINE - 4) {
        tc_snprintf(fd->input, TC_BUF_LINE, "%s.trf", filebasename);
    } else {
        tc_log_warn(MOD_NAME, "input name too long, using default `%s'",
                    DEFAULT_TRANS_FILE_NAME);
        tc_snprintf(fd->input, TC_BUF_LINE, DEFAULT_TRANS_FILE_NAME);
    }



    /* process remaining options */
    if (options != NULL) {
        // We support also the help option.
        if(optstr_lookup(options, "help")) {
            tc_log_info(MOD_NAME,transform_help);
            return(TC_IMPORT_ERROR);
        }
        optstr_get(options, "input", "%[^:]", (char*)&fd->input);
        optstr_get(options, "maxshift",  "%d", &td->maxShift);
        optstr_get(options, "maxangle",  "%lf", &td->maxAngle);
        optstr_get(options, "smoothing", "%d", &td->smoothing);
        optstr_get(options, "crop"     , "%d", (int*)&td->crop);
        optstr_get(options, "invert"   , "%d", &td->invert);
        optstr_get(options, "relative" , "%d", &td->relative);
        optstr_get(options, "zoom"     , "%lf",&td->zoom);
        optstr_get(options, "optzoom"  , "%d", &td->optZoom);
        optstr_get(options, "interpol" , "%d", (int*)(&td->interpolType));
        optstr_get(options, "sharpen"  , "%lf",&td->sharpen);
        if(optstr_lookup(options, "tripod")){
            tc_log_info(MOD_NAME,"Virtual tripod mode: relative=False, smoothing=0");
            td->relative=0;
            td->smoothing=0;
        }
    }

    if(configureTransformData(td)!= VS_OK){
        tc_log_error(MOD_NAME, "configuration of TransformData failed");
        return TC_ERROR;
    }

    if (verbose) {
        tc_log_info(MOD_NAME, "Image Transformation/Stabilization Settings:");
        tc_log_info(MOD_NAME, "    input     = %s", fd->input);
        tc_log_info(MOD_NAME, "    smoothing = %d", td->smoothing);
        tc_log_info(MOD_NAME, "    maxshift  = %d", td->maxShift);
        tc_log_info(MOD_NAME, "    maxangle  = %f", td->maxAngle);
        tc_log_info(MOD_NAME, "    crop      = %s",
                        td->crop ? "Black" : "Keep");
        tc_log_info(MOD_NAME, "    relative  = %s",
                    td->relative ? "True": "False");
        tc_log_info(MOD_NAME, "    invert    = %s",
                    td->invert ? "True" : "False");
        tc_log_info(MOD_NAME, "    zoom      = %f", td->zoom);
        tc_log_info(MOD_NAME, "    optzoom   = %s",
                    td->optZoom ? "On" : "Off");
        tc_log_info(MOD_NAME, "    interpol  = %s",
                    interpolTypes[td->interpolType]);
        tc_log_info(MOD_NAME, "    sharpen   = %f", td->sharpen);
    }

    f = fopen(fd->input, "r");
    if (f == NULL) {
        tc_log_error(MOD_NAME, "cannot open input file %s!\n", fd->input);
        /* return (-1); when called using tcmodinfo this will fail */
    } else {
        ManyLocalMotions mlms;
        if(readLocalMotionsFile(f,&mlms)==VS_OK){
            // calculate the actual transforms from the localmotions
            if(localmotions2TransformsSimple(td, &mlms,&fd->trans)!=VS_OK)
                tc_log_error(MOD_NAME, "calculating transformations failed!\n");
        }else{ // try to read old format
            if (!readOldTransforms(td, f, &fd->trans)) { /* read input file */
                tc_log_error(MOD_NAME, "error parsing input file %s!\n", fd->input);
            }
        }
    }
    fclose(f);

    if (preprocessTransforms(td, &fd->trans)!= VS_OK ) {
        tc_log_error(MOD_NAME, "error while preprocessing transforms!");
        return TC_ERROR;
    }

    // sharpen is still in transcode...
    /* Is this the right point to add the filter? Seems to be the case.*/
    if(td->sharpen>0){
        /* load unsharp filter */
        char unsharp_param[256];
        sprintf(unsharp_param,"luma=%f:%s:chroma=%f:%s",
                td->sharpen, "luma_matrix=5x5",
                td->sharpen/2, "chroma_matrix=5x5");
        if (!tc_filter_add("unsharp", unsharp_param)) {
            tc_log_warn(MOD_NAME, "cannot load unsharp filter!");
        }
    }

    return TC_OK;
}


/**
 * transform_filter_video: performs the transformation of frames
 * See tcmodule-data.h for function details.
 */
static int transform_filter_video(TCModuleInstance *self,
                                  vframe_list_t *frame)
{
    FilterData *fd = NULL;

    TC_MODULE_SELF_CHECK(self, "filter_video");
    TC_MODULE_SELF_CHECK(frame, "filter_video");

    fd = self->userdata;
    VSFrame vsFrame;
    fillFrameFromBuffer(&vsFrame,frame->video_buf, &fd->td.fiSrc);

    transformPrepare(&fd->td, &vsFrame,  &vsFrame);

    Transform t = getNextTransform(&fd->td, &fd->trans);
    if (fd->vob->im_v_codec == CODEC_RGB) {
        transformRGB(&fd->td, t);
    } else if (fd->vob->im_v_codec == CODEC_YUV) {
        transformYUV(&fd->td, t);
    } else {
        tc_log_error(MOD_NAME, "unsupported Codec: %i\n", fd->vob->im_v_codec);
        return TC_ERROR;
    }
    transformFinish(&fd->td);
    return TC_OK;
}


/**
 * transform_fini:  Clean up after this instance of the module.  See
 * tcmodule-data.h for function details.
 */
static int transform_fini(TCModuleInstance *self)
{
    FilterData *fd = NULL;
    TC_MODULE_SELF_CHECK(self, "fini");
    fd = self->userdata;
    tc_free(fd);
    self->userdata = NULL;
    return TC_OK;
}


/**
 * transform_stop:  Reset this instance of the module.  See tcmodule-data.h
 * for function details.
 */
static int transform_stop(TCModuleInstance *self)
{
    FilterData *fd = NULL;
    TC_MODULE_SELF_CHECK(self, "stop");
    fd = self->userdata;
    cleanupTransformData(&fd->td);

    cleanupTransformations(&fd->trans);
    return TC_OK;
}

/* checks for parameter in function _inspect */
#define CHECKPARAM(paramname, formatstring, variable)       \
    if (optstr_lookup(param, paramname)) {                \
        tc_snprintf(fd->conf_str, sizeof(fd->conf_str),   \
                    formatstring, variable);              \
        *value = fd->conf_str;                            \
    }

/**
 * stabilize_inspect:  Return the value of an option in this instance of
 * the module.  See tcmodule-data.h for function details.
 */
static int transform_inspect(TCModuleInstance *self,
            			     const char *param, const char **value)
{
    FilterData *fd = NULL;
    TC_MODULE_SELF_CHECK(self,  "inspect");
    TC_MODULE_SELF_CHECK(param, "inspect");
    TC_MODULE_SELF_CHECK(value, "inspect");

    fd = self->userdata;

    if (optstr_lookup(param, "help")) {
        *value = transform_help;
    }
    CHECKPARAM("maxshift", "maxshift=%d",  fd->td.maxShift);
    CHECKPARAM("maxangle", "maxangle=%f",  fd->td.maxAngle);
    CHECKPARAM("smoothing","smoothing=%d", fd->td.smoothing);
    CHECKPARAM("crop",     "crop=%d",      fd->td.crop);
    CHECKPARAM("relative", "relative=%d",  fd->td.relative);
    CHECKPARAM("invert",   "invert=%i",    fd->td.invert);
    CHECKPARAM("input",    "input=%s",     fd->input);
    CHECKPARAM("optzoom",  "optzoom=%i",   fd->td.optZoom);
    CHECKPARAM("zoom",     "zoom=%f",      fd->td.zoom);
    CHECKPARAM("sharpen",  "sharpen=%f",   fd->td.sharpen);

    return TC_OK;
};


static const TCCodecID transform_codecs_in[] = {
    TC_CODEC_YUV420P, TC_CODEC_YUV422P, TC_CODEC_RGB, TC_CODEC_ERROR
};
static const TCCodecID transform_codecs_out[] = {
    TC_CODEC_YUV420P, TC_CODEC_YUV422P, TC_CODEC_RGB, TC_CODEC_ERROR
};
TC_MODULE_FILTER_FORMATS(transform);

TC_MODULE_INFO(transform);

static const TCModuleClass transform_class = {
    TC_MODULE_CLASS_HEAD(transform),

    .init         = transform_init,
    .fini         = transform_fini,
    .configure    = transform_configure,
    .stop         = transform_stop,
    .inspect      = transform_inspect,

    .filter_video = transform_filter_video,
};

TC_MODULE_ENTRY_POINT(transform)

/*************************************************************************/

static int transform_get_config(TCModuleInstance *self, char *options)
{
    TC_MODULE_SELF_CHECK(self, "get_config");

    optstr_filter_desc(options, MOD_NAME, MOD_CAP, MOD_VERSION,
                       MOD_AUTHOR, "VRY4", "1");

    return TC_OK;
}

static int transform_process(TCModuleInstance *self, frame_list_t *frame)
{
    TC_MODULE_SELF_CHECK(self, "process");

    if (frame->tag & TC_PRE_S_PROCESS && frame->tag & TC_VIDEO) {
        return transform_filter_video(self, (vframe_list_t *)frame);
    }
    return TC_OK;
}

/*************************************************************************/

TC_FILTER_OLDINTERFACE(transform)

/*************************************************************************/
/*
  TODO:
  - check for optimization, e.g. mmx stuff
*/

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
