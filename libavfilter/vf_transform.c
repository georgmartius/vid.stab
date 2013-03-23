/*
 *  vf_transform.c
 *
 *  Copyright (C) Georg Martius - Jan 2012
 *   georg dot martius at web dot de
 *
 *  This file is part of vid.stab, video deshaking lib
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
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

/* Typical call:
 *  ffmpeg -i inp.mpeg -vf transform,unsharp=5:5:0.8:3:3:0.4 inp_s.mpeg
 *  all parameters are optional
 */

#define DEFAULT_INPUT_NAME     "transforms.trf"

#include "libavutil/common.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
// #include "libavcodec/dsputil.h"
#include "avfilter.h"
#include "internal.h"

#include <vid.stab/libvidstab.h>

/* private date structure of this filter*/
typedef struct {
    const AVClass* class;

    TransformData td;

    Transformations trans; // transformations
    char* args;
    char* input;           // name of transform file
    int tripod;
} TransformContext;

/*** Commandline options ****/

#define OFFSET(x) offsetof(TransformContext, x)
#define OFFSETTD(x) (offsetof(TransformContext, td)+offsetof(TransformData, x))
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption transform_options[]= {
    {"input",     "path to the file storing the transforms (def:transforms.trf)",   OFFSET(input),
                   AV_OPT_TYPE_STRING, {.str = DEFAULT_INPUT_NAME} },
    {"smoothing", "number of frames*2 + 1 used for lowpass filtering (def: 10)",    OFFSETTD(smoothing),
                   AV_OPT_TYPE_INT,    {.i64 = 10},       1, 1000, FLAGS},
    {"maxshift",  "maximal number of pixels to translate image (def: -1 no limit)", OFFSETTD(maxShift),
                   AV_OPT_TYPE_INT,    {.i64 = -1},      -1, 500,  FLAGS},
    {"maxangle",  "maximal angle in rad to rotate image (def: -1 no limit)",        OFFSETTD(maxAngle),
                   AV_OPT_TYPE_DOUBLE, {.dbl = -1.0},  -1.0, 3.14, FLAGS},
    {"crop",      "keep: (def), black",                                             OFFSETTD(crop),
                   AV_OPT_TYPE_INT,    {.i64 = 0},        0, 1,    FLAGS, "crop"},
    {  "keep",    "keep border",                                                    0,
                   AV_OPT_TYPE_CONST,  {.i64 = KeepBorder }, 0, 0, FLAGS, "crop"},
    {  "black",   "black border",                                                   0,
                   AV_OPT_TYPE_CONST,  {.i64 = CropBorder }, 0, 0, FLAGS, "crop"},
    {"invert",    "1: invert transforms(def: 0)",                                   OFFSETTD(invert),
                   AV_OPT_TYPE_INT,    {.i64 = 1},        0, 1,    FLAGS},
    {"relative",  "consider transforms as 0: absolute, 1: relative (def)",          OFFSETTD(relative),
                   AV_OPT_TYPE_INT,    {.i64 = 0},        0, 1,    FLAGS},
    {"zoom",      "percentage to zoom >0: zoom in, <0 zoom out (def: 0)",           OFFSETTD(zoom),
                   AV_OPT_TYPE_DOUBLE, {.dbl = 0},        0, 100,  FLAGS},
    {"optzoom",   "0: nothing, 1: determine optimal zoom (def) (added to 'zoom')",  OFFSETTD(optZoom),
                   AV_OPT_TYPE_INT,    {.i64 = 1},        0, 1,    FLAGS},
    {"interpol",  "type of interpolation, no, linear, bilinear (def) , bicubic",    OFFSETTD(interpolType),
                   AV_OPT_TYPE_INT,    {.i64 = 2},        0, 3,    FLAGS, "interpol"},
    {  "no",      "no interpolation",                                               0,
                   AV_OPT_TYPE_CONST,  {.i64 = Zero  },   0, 0,    FLAGS, "interpol"},
    {  "linear",  "linear (horizontal)",                                            0,
                   AV_OPT_TYPE_CONST,  {.i64 = Linear },  0, 0,    FLAGS, "interpol"},
    {  "bilinear","bi-linear",                                                      0,
                   AV_OPT_TYPE_CONST,  {.i64 = BiLinear}, 0, 0,    FLAGS, "interpol"},
    {  "bicubic", "bi-cubic",                                                       0,
                   AV_OPT_TYPE_CONST,  {.i64 = BiCubic }, 0, 0,    FLAGS, "interpol"},
    {"tripod",    "if 1: virtual tripod mode (equiv. to relative=0:smoothing=0)",   OFFSET(tripod),
                   AV_OPT_TYPE_INT,    {.i64 = 0},        0, 1,    FLAGS},
    {NULL},
};

AVFILTER_DEFINE_CLASS(transform);

/*** some conversions from avlib to vid.stab constants and functions ****/

/** convert AV's pixelformat to vid.stab pixelformat */
static PixelFormat AV2VSPixelFormat(AVFilterContext *ctx, enum AVPixelFormat pf){
	switch(pf){
    case AV_PIX_FMT_YUV420P:  return PF_YUV420P;
		case AV_PIX_FMT_YUV422P:	return PF_YUV422P;
		case AV_PIX_FMT_YUV444P:	return PF_YUV444P;
		case AV_PIX_FMT_YUV410P:	return PF_YUV410P;
		case AV_PIX_FMT_YUV411P:	return PF_YUV411P;
		case AV_PIX_FMT_YUV440P:	return PF_YUV440P;
		case AV_PIX_FMT_YUVA420P: return PF_YUVA420P;
		case AV_PIX_FMT_GRAY8:		return PF_GRAY8;
		case AV_PIX_FMT_RGB24:		return PF_RGB24;
		case AV_PIX_FMT_BGR24:		return PF_BGR24;
		case AV_PIX_FMT_RGBA:		  return PF_RGBA;
	default:
		av_log(ctx, AV_LOG_ERROR, "cannot deal with pixel format %i!\n", pf);
		return PF_NONE;
	}
}


/// struct to hold a valid context for logging from within vid.stab lib
typedef struct {
    const AVClass* class;
} TransformLogCtx;

/** wrapper to log vs_log into av_log */
static int av_log_wrapper(int type, const char* tag, const char* format, ...){
    va_list ap;
    TransformLogCtx ctx;
    ctx.class = &transform_class;
    av_log(&ctx,  type, "%s: ", tag);
    va_start (ap, format);
    av_vlog(&ctx, type, format, ap);
    va_end (ap);
    return VS_OK;
}

/** sets the memory allocation function and logging constants to av versions */
static void setMemAndLogFunctions(void){
    vs_malloc  = av_malloc;
    vs_zalloc  = av_mallocz;
    vs_realloc = av_realloc;
    vs_free    = av_free;

    VS_ERROR_TYPE = AV_LOG_ERROR;
    VS_WARN_TYPE  = AV_LOG_WARNING;
    VS_INFO_TYPE  = AV_LOG_INFO;
    VS_MSG_TYPE   = AV_LOG_VERBOSE;

    vs_log   = av_log_wrapper;

    VS_ERROR = 0;
    VS_OK    = 1;
}

/*************************************************************************/

/* Module interface routines and data. */

/*************************************************************************/

static av_cold int init(AVFilterContext *ctx, const char *args)
{

    TransformContext* tc = ctx->priv;

    setMemAndLogFunctions();

    tc->class = &transform_class;
    av_opt_set_defaults(tc); // the default values are overwritten by initMotiondetect later

    av_log(ctx, AV_LOG_VERBOSE, "transform filter: init %s\n", LIBVIDSTAB_VERSION);

    // save args for later, because the initialization of the vid.stab library requires
    //  knowledge about the input.
    if(args)
        tc->args=av_strdup(args);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    TransformContext *tc = ctx->priv;

    av_opt_free(tc);

    cleanupTransformData(&tc->td);
    cleanupTransformations(&tc->trans);

    av_free(tc->args);
    av_free(tc->input); // <- is this already free'd by av_opt_free?
}


static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUV444P,  AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUV411P,  AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUVA420P,
        AV_PIX_FMT_YUV440P,  AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_RGB24, AV_PIX_FMT_BGR24, AV_PIX_FMT_RGBA,
        AV_PIX_FMT_NONE
    };

    ff_set_common_formats(ctx, ff_make_format_list(pix_fmts));
    return 0;
}


static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    TransformContext *tc = ctx->priv;
    FILE* f;
    int returnval;

    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);

    TransformData* td = &(tc->td);

    VSFrameInfo fi_src;
    VSFrameInfo fi_dest;

    if(!initFrameInfo(&fi_src, inlink->w, inlink->h,
                      AV2VSPixelFormat(ctx,inlink->format)) ||
       !initFrameInfo(&fi_dest, inlink->w, inlink->h,
                      AV2VSPixelFormat(ctx, inlink->format))){
        av_log(ctx, AV_LOG_ERROR, "unknown pixel format: %i (%s)",
               inlink->format, desc->name);
        return AVERROR(EINVAL);
    }

    // check
    if(fi_src.bytesPerPixel != av_get_bits_per_pixel(desc)/8 ||
       fi_src.log2ChromaW != desc->log2_chroma_w ||
       fi_src.log2ChromaH != desc->log2_chroma_h){
        av_log(ctx, AV_LOG_ERROR, "pixel-format error: bpp %i<>%i  ",
               fi_src.bytesPerPixel, av_get_bits_per_pixel(desc)/8);
        av_log(ctx, AV_LOG_ERROR, "chroma_subsampl: w: %i<>%i  h: %i<>%i\n",
               fi_src.log2ChromaW, desc->log2_chroma_w,
               fi_src.log2ChromaH, desc->log2_chroma_h);
        return AVERROR(EINVAL);
    }

    if(initTransformData(td, &fi_src, &fi_dest, "transform") != VS_OK){
        av_log(ctx, AV_LOG_ERROR, "initialization of TransformData failed\n");
        return AVERROR(EINVAL);
    }
    td->verbose=1; // TODO: get from somewhere

    // we need to do it after initTransformData because otherwise the values are overwritten
    if ((returnval = (av_set_options_string(tc, tc->args, "=", ":"))) < 0)
        return returnval;

    if(tc->tripod){
        av_log(ctx, AV_LOG_INFO, "Virtual tripod mode: relative=0, smoothing=0");
        td->relative=0;
        td->smoothing=0;
    }

    /* can we get the input name from somewhere? */
    /* if (tc->args != NULL) { */
    /*     if(optstr_lookup(tc->options, "help")) { */
    /*         av_log(ctx, AV_LOG_INFO, transform_help); */
    /*         return AVERROR(EINVAL); */
    /*     } */

    if(configureTransformData(td)!= VS_OK){
      av_log(ctx, AV_LOG_ERROR, "configuration of Tranform failed\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_INFO, "Image Transformation/Stabilization Settings:\n");
    av_log(ctx, AV_LOG_INFO, "    input     = %s\n", tc->input);
    av_log(ctx, AV_LOG_INFO, "    smoothing = %d\n", td->smoothing);
    av_log(ctx, AV_LOG_INFO, "    maxshift  = %d\n", td->maxShift);
    av_log(ctx, AV_LOG_INFO, "    maxangle  = %f\n", td->maxAngle);
    av_log(ctx, AV_LOG_INFO, "    crop      = %s\n", td->crop ? "Black" : "Keep");
    av_log(ctx, AV_LOG_INFO, "    relative  = %s\n", td->relative ? "True": "False");
    av_log(ctx, AV_LOG_INFO, "    invert    = %s\n", td->invert ? "True" : "False");
    av_log(ctx, AV_LOG_INFO, "    zoom      = %f\n", td->zoom);
    av_log(ctx, AV_LOG_INFO, "    optzoom   = %s\n", td->optZoom ? "On" : "Off");
    av_log(ctx, AV_LOG_INFO, "    interpol  = %s\n", interpolTypes[td->interpolType]);
    av_log(ctx, AV_LOG_INFO, "    sharpen   = %f\n", td->sharpen);

    f = fopen(tc->input, "r");
    if (f == NULL) {
        av_log(ctx, AV_LOG_ERROR, "cannot open input file %s!\n", tc->input);
    } else {
        ManyLocalMotions mlms;
        if(readLocalMotionsFile(f,&mlms)==VS_OK){
            // calculate the actual transforms from the localmotions
            if(localmotions2TransformsSimple(td, &mlms,&tc->trans)!=VS_OK)
                av_log(ctx, AV_LOG_ERROR, "calculating transformations failed!\n");
        }else{ // try to read old format
            if (!readOldTransforms(td, f, &tc->trans)) { /* read input file */
                av_log(ctx, AV_LOG_ERROR, "error parsing input file %s!\n", tc->input);
            }
        }
    }
    fclose(f);

    if (preprocessTransforms(td, &tc->trans)!= VS_OK ) {
        av_log(ctx, AV_LOG_ERROR, "error while preprocessing transforms\n");
        return AVERROR(EINVAL);
    }

    // TODO: add sharpening
    return 0;
}


static int filter_frame(AVFilterLink *inlink,  AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    TransformContext *tc = ctx->priv;
    TransformData* td = &(tc->td);

    AVFilterLink *outlink = inlink->dst->outputs[0];
    //const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    //int hsub0 = desc->log2_chroma_w;
    //int vsub0 = desc->log2_chroma_h;
    int direct = 0;
    AVFrame *out;
    VSFrame inframe;
    int plane;

    if (av_frame_is_writable(in)) {
        direct = 1;
        out = in;
    } else {
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    for(plane=0; plane < td->fiSrc.planes; plane++){
        inframe.data[plane] = in->data[plane];
        inframe.linesize[plane] = in->linesize[plane];
    }
    if(out == in){ // inplace
        transformPrepare(td, &inframe, &inframe);
    }else{ // seperate frames
        VSFrame outframe;
        for(plane=0; plane < td->fiDest.planes; plane++){
            outframe.data[plane] = out->data[plane];
            outframe.linesize[plane] = out->linesize[plane];
        }
        transformPrepare(td, &inframe, &outframe);
    }

    if (tc->td.fiSrc.pFormat > PF_PACKED) {
        transformRGB(td, getNextTransform(td, &tc->trans));
    } else {
        transformYUV(td, getNextTransform(td, &tc->trans));
    }
    transformFinish(td);

    if (!direct)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static const AVFilterPad avfilter_vf_transform_inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .get_video_buffer = ff_null_get_video_buffer,
        .filter_frame     = filter_frame,
        .config_props     = config_input,
        .min_perms        = AV_PERM_READ,
    },
    { NULL }
};

static const AVFilterPad avfilter_vf_transform_outputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter avfilter_vf_transform = {
    .name          = "transform",
    .description   = NULL_IF_CONFIG_SMALL("transforms each frame according to transformations\n\
    given in an input file (e.g. translation, rotate) see also filter stabilize."),
    .priv_size     = sizeof(TransformContext),
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,

    .inputs        = avfilter_vf_transform_inputs,
    .outputs       = avfilter_vf_transform_outputs,
    .priv_class    = &transform_class,

};

