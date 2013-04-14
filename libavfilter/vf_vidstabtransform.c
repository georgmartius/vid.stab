/*
 * Copyright (c) 2013 Georg Martius <georg dot martius at web dot de>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define DEFAULT_INPUT_NAME     "transforms.trf"

#include <vid.stab/libvidstab.h>

#include "libavutil/common.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "avfilter.h"
#include "internal.h"

#include "vidstabutils.h"

typedef struct {
    const AVClass* class;

    VSTransformData td;

    VSTransformations trans; // transformations
    char* args;
    char* input;           // name of transform file
    int tripod;
} TransformContext;

#define OFFSET(x) offsetof(TransformContext, x)
#define OFFSETTD(x) (offsetof(TransformContext, td)+offsetof(VSTransformData, x))
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption vidstabtransform_options[]= {
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
                   AV_OPT_TYPE_CONST,  {.i64 = VSKeepBorder }, 0, 0, FLAGS, "crop"},
    {  "black",   "black border",                                                   0,
                   AV_OPT_TYPE_CONST,  {.i64 = VSCropBorder }, 0, 0, FLAGS, "crop"},
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
                   AV_OPT_TYPE_CONST,  {.i64 = VS_Zero  },  0, 0,  FLAGS, "interpol"},
    {  "linear",  "linear (horizontal)",                                            0,
                   AV_OPT_TYPE_CONST,  {.i64 = VS_Linear }, 0, 0,  FLAGS, "interpol"},
    {  "bilinear","bi-linear",                                                      0,
                   AV_OPT_TYPE_CONST,  {.i64 = VS_BiLinear},0, 0,  FLAGS, "interpol"},
    {  "bicubic", "bi-cubic",                                                       0,
                   AV_OPT_TYPE_CONST,  {.i64 = VS_BiCubic },0, 0,  FLAGS, "interpol"},
    {"tripod",    "if 1: virtual tripod mode (equiv. to relative=0:smoothing=0)",   OFFSET(tripod),
                   AV_OPT_TYPE_INT,    {.i64 = 0},        0, 1,    FLAGS},
    {NULL},
};

AVFILTER_DEFINE_CLASS(vidstabtransform);


static av_cold int init(AVFilterContext *ctx, const char *args)
{

    TransformContext* tc = ctx->priv;

    vs_set_mem_and_log_functions();

    tc->class = &vidstabtransform_class;
    av_opt_set_defaults(tc); // the default values are overwritten by initMotiondetect later

    av_log(ctx, AV_LOG_VERBOSE, "vidstabtransform filter: init %s\n", LIBVIDSTAB_VERSION);

    // save args for later, because the initialization of the vid.stab library requires
    //  knowledge about the input.
    tc->args=av_strdup(args);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    TransformContext *tc = ctx->priv;

    av_opt_free(tc);

    vsTransformDataCleanup(&tc->td);
    vsTransformationsCleanup(&tc->trans);

    av_free(tc->args);
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

    VSTransformData* td = &(tc->td);

    VSFrameInfo fi_src;
    VSFrameInfo fi_dest;

    if(!vsFrameInfoInit(&fi_src, inlink->w, inlink->h,
                      av_2_vs_pixel_format(ctx,inlink->format)) ||
       !vsFrameInfoInit(&fi_dest, inlink->w, inlink->h,
                      av_2_vs_pixel_format(ctx, inlink->format))){
        av_log(ctx, AV_LOG_ERROR, "unknown pixel format: %i (%s)",
               inlink->format, desc->name);
        return AVERROR(EINVAL);
    }

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

    if(vsTransformDataInit(td, &fi_src, &fi_dest, "vidstabtransform") != VS_OK){
        av_log(ctx, AV_LOG_ERROR, "initialization of TransformData failed\n");
        return AVERROR(EINVAL);
    }
    td->verbose=1;

    // we need to do it after vsTransformDataInit because otherwise the values are overwritten
    if ((returnval = (av_set_options_string(tc, tc->args, "=", ":"))) < 0)
        return returnval;

    if(tc->tripod){
        av_log(ctx, AV_LOG_INFO, "Virtual tripod mode: relative=0, smoothing=0");
        td->relative=0;
        td->smoothing=0;
    }

    if(vsTransformDataConfigure(td)!= VS_OK){
      av_log(ctx, AV_LOG_ERROR, "configuration of Tranform failed\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_INFO, "Video transformation/stabilization settings (pass 2/2):\n");
    av_log(ctx, AV_LOG_INFO, "    input     = %s\n", tc->input);
    av_log(ctx, AV_LOG_INFO, "    smoothing = %d\n", td->smoothing);
    av_log(ctx, AV_LOG_INFO, "    maxshift  = %d\n", td->maxShift);
    av_log(ctx, AV_LOG_INFO, "    maxangle  = %f\n", td->maxAngle);
    av_log(ctx, AV_LOG_INFO, "    crop      = %s\n", td->crop ? "Black" : "Keep");
    av_log(ctx, AV_LOG_INFO, "    relative  = %s\n", td->relative ? "True": "False");
    av_log(ctx, AV_LOG_INFO, "    invert    = %s\n", td->invert ? "True" : "False");
    av_log(ctx, AV_LOG_INFO, "    zoom      = %f\n", td->zoom);
    av_log(ctx, AV_LOG_INFO, "    optzoom   = %s\n", td->optZoom ? "On" : "Off");
    av_log(ctx, AV_LOG_INFO, "    interpol  = %s\n", getInterpolationTypeName(td->interpolType));
    av_log(ctx, AV_LOG_INFO, "    sharpen   = %f\n", td->sharpen);

    f = fopen(tc->input, "r");
    if (f == NULL) {
        av_log(ctx, AV_LOG_ERROR, "cannot open input file %s\n", tc->input);
        return AVERROR(errno);
    } else {
        VSManyLocalMotions mlms;
        if(vsReadLocalMotionsFile(f,&mlms)==VS_OK){
            // calculate the actual transforms from the localmotions
            if(vsLocalmotions2TransformsSimple(td, &mlms,&tc->trans)!=VS_OK){
                av_log(ctx, AV_LOG_ERROR, "calculating transformations failed\n");
                return AVERROR(EINVAL);
            }
        }else{ // try to read old format
            if (!vsReadOldTransforms(td, f, &tc->trans)) { /* read input file */
                av_log(ctx, AV_LOG_ERROR, "error parsing input file %s\n", tc->input);
                return AVERROR(EINVAL);
            }
        }
    }
    fclose(f);

    if (vsPreprocessTransforms(td, &tc->trans)!= VS_OK ) {
        av_log(ctx, AV_LOG_ERROR, "error while preprocessing transforms\n");
        return AVERROR(EINVAL);
    }

    // TODO: add sharpening, so far the user needs to call the unsharp filter manually
    return 0;
}


static int filter_frame(AVFilterLink *inlink,  AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    TransformContext *tc = ctx->priv;
    VSTransformData* td = &(tc->td);

    AVFilterLink *outlink = inlink->dst->outputs[0];
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
        vsTransformPrepare(td, &inframe, &inframe);
    }else{ // seperate frames
        VSFrame outframe;
        for(plane=0; plane < td->fiDest.planes; plane++){
            outframe.data[plane] = out->data[plane];
            outframe.linesize[plane] = out->linesize[plane];
        }
        vsTransformPrepare(td, &inframe, &outframe);
    }

    vsDoTransform(td, vsGetNextTransform(td, &tc->trans));

    vsTransformFinish(td);

    if (!direct)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static const AVFilterPad avfilter_vf_vidstabtransform_inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .filter_frame     = filter_frame,
        .config_props     = config_input,
    },
    { NULL }
};

static const AVFilterPad avfilter_vf_vidstabtransform_outputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter avfilter_vf_vidstabtransform = {
    .name          = "vidstabtransform",
    .description   = NULL_IF_CONFIG_SMALL("pass 2 of stabilization"
                                          "transforms the frames"
                                          "(see vidstabdetect for pass 1)"),
    .priv_size     = sizeof(TransformContext),
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,

    .inputs        = avfilter_vf_vidstabtransform_inputs,
    .outputs       = avfilter_vf_vidstabtransform_outputs,
    .priv_class    = &vidstabtransform_class,

};

