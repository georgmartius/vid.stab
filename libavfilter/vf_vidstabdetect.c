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

#define DEFAULT_RESULT_NAME     "transforms.trf"

#include <vid.stab/libvidstab.h>

#include "libavutil/common.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "avfilter.h"
#include "internal.h"

#include "vidstabutils.h"

typedef struct {
    const AVClass* class;

    VSMotionDetect md;

    char* args;
    char* result;
    FILE* f;
} StabData;


#define OFFSET(x) offsetof(StabData, x)
#define OFFSETMD(x) (offsetof(StabData, md)+offsetof(VSMotionDetect, x))
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption vidstabdetect_options[]= {
    {"result",      "path to the file used to write the transforms (def:transforms.trf)", OFFSET(result),              AV_OPT_TYPE_STRING, {.str = DEFAULT_RESULT_NAME}},
    {"shakiness",   "how shaky is the video and how quick is the camera?"
                    " 1: little (fast) 10: very strong/quick (slow) (def: 5)",            OFFSETMD(shakiness),         AV_OPT_TYPE_INT,    {.i64 = 5},  1, 10,       FLAGS},
    {"accuracy",    "(>=shakiness) 1: low 15: high (slow) (def: 9)",                      OFFSETMD(accuracy),          AV_OPT_TYPE_INT,    {.i64 = 9 }, 1, 15,       FLAGS},
    {"stepsize",    "region around minimum is scanned with 1 pixel resolution (def: 6)",  OFFSETMD(stepSize),          AV_OPT_TYPE_INT,    {.i64 = 6},  1, 32,       FLAGS},
    {"mincontrast", "below this contrast a field is discarded (0-1) (def: 0.3)",          OFFSETMD(contrastThreshold), AV_OPT_TYPE_DOUBLE, {.dbl =  0.25}, 0.0, 1.0, FLAGS},
    {"show",        "0: draw nothing (def); 1,2: show fields and transforms",             OFFSETMD(show),              AV_OPT_TYPE_INT,    {.i64 =  0}, 0, 2,        FLAGS},
    {"tripod",      "virtual tripod mode (if >0): motion is compared to a reference"
                    " reference frame (frame # is the value) (def: 0)",                   OFFSETMD(virtualTripod),     AV_OPT_TYPE_INT,    {.i64 = 0},  0, INT_MAX,  FLAGS},
    {NULL},
};

AVFILTER_DEFINE_CLASS(vidstabdetect);

static av_cold int init(AVFilterContext *ctx, const char *args)
{
    StabData* sd = ctx->priv;
    vs_set_mem_and_log_functions();

    sd->class = &vidstabdetect_class;
    av_opt_set_defaults(sd); // the default values are overwritten by initMotiondetect later

    av_log(ctx, AV_LOG_VERBOSE, "vidstabdetect filter: init %s\n", LIBVIDSTAB_VERSION);

    // save args for later, because the initialization of the vid.stab library requires
    //  knowledge about the input.
    sd->args=av_strdup(args);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    StabData *sd = ctx->priv;
    VSMotionDetect* md = &(sd->md);

    av_opt_free(sd);
    if (sd->f) {
        fclose(sd->f);
        sd->f = NULL;
    }

    vsMotionDetectionCleanup(md);
    av_free(sd->args);
}

static int query_formats(AVFilterContext *ctx)
{
    // If you add something here also add it to the above mapping function
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
    StabData *sd = ctx->priv;
    int returnval;

    VSMotionDetect* md = &(sd->md);
    VSFrameInfo fi;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);

    vsFrameInfoInit(&fi,inlink->w, inlink->h, av_2_vs_pixel_format(ctx, inlink->format));
    if(fi.bytesPerPixel != av_get_bits_per_pixel(desc)/8){
        av_log(ctx, AV_LOG_ERROR, "pixel-format error: wrong bits/per/pixel, please report a BUG");
        return AVERROR(EINVAL);
    }
    if(fi.log2ChromaW != desc->log2_chroma_w){
        av_log(ctx, AV_LOG_ERROR, "pixel-format error: log2_chroma_w, please report a BUG");
        return AVERROR(EINVAL);
    }

    if(fi.log2ChromaH != desc->log2_chroma_h){
        av_log(ctx, AV_LOG_ERROR, "pixel-format error: log2_chroma_h, please report a BUG");
        return AVERROR(EINVAL);
    }

    if(vsMotionDetectInit(md, &fi, "vidstabdetect") != VS_OK){
        av_log(ctx, AV_LOG_ERROR, "initialization of Motion Detection failed, please report a BUG");
        return AVERROR(EINVAL);
    }

    // we need to do it after vsMotionDetectInit because otherwise the values are overwritten
    if ((returnval = (av_set_options_string(sd, sd->args, "=", ":"))) < 0)
        return returnval;

    if(vsMotionDetectConfigure(md)!= VS_OK){
        av_log(ctx, AV_LOG_ERROR, "configuration of Motion Detection failed, please report a BUG\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_INFO, "Video stabilization settings (pass 1/2):\n");
    av_log(ctx, AV_LOG_INFO, "     shakiness = %d\n", md->shakiness);
    av_log(ctx, AV_LOG_INFO, "      accuracy = %d\n", md->accuracy);
    av_log(ctx, AV_LOG_INFO, "      stepsize = %d\n", md->stepSize);
    av_log(ctx, AV_LOG_INFO, "   mincontrast = %f\n", md->contrastThreshold);
    av_log(ctx, AV_LOG_INFO, "          show = %d\n", md->show);
    av_log(ctx, AV_LOG_INFO, "        result = %s\n", sd->result);

    sd->f = fopen(sd->result, "w");
    if (sd->f == NULL) {
        av_log(ctx, AV_LOG_ERROR, "cannot open transform file %s\n", sd->result);
        return AVERROR(EINVAL);
    }else{
        if(vsPrepareFile(md, sd->f) != VS_OK){
            av_log(ctx, AV_LOG_ERROR, "cannot write to transform file %s\n", sd->result);
            return AVERROR(EINVAL);
        }
    }
    return 0;
}


static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    StabData *sd = ctx->priv;
    VSMotionDetect* md = &(sd->md);
    LocalMotions localmotions;

    AVFilterLink *outlink = inlink->dst->outputs[0];
    int direct = 0;
    AVFrame *out;
    VSFrame frame;
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

    for(plane=0; plane < md->fi.planes; plane++){
        frame.data[plane] = in->data[plane];
        frame.linesize[plane] = in->linesize[plane];
    }
    if(vsMotionDetection(md, &localmotions, &frame) !=  VS_OK){
        av_log(ctx, AV_LOG_ERROR, "motion detection failed");
        return AVERROR(AVERROR_EXTERNAL);
    } else {
        if(vsWriteToFile(md, sd->f, &localmotions) != VS_OK){
            av_log(ctx, AV_LOG_ERROR, "cannot write to transform file");
            return AVERROR(errno);
        }
        vs_vector_del(&localmotions);
    }
    if(md->show>0 && !direct){
        av_image_copy(out->data, out->linesize,
                      (void*)in->data, in->linesize,
                      in->format, in->width, in->height);
    }

    if (!direct)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static const AVFilterPad avfilter_vf_vidstabdetect_inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .filter_frame     = filter_frame,
        .config_props     = config_input,
    },
    { NULL }
};

static const AVFilterPad avfilter_vf_vidstabdetect_outputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter avfilter_vf_vidstabdetect = {
    .name          = "vidstabdetect",
    .description   = NULL_IF_CONFIG_SMALL("pass 1 of 2 for stabilization"
                                          "extracts relative transformations"
                                          "(pass 2 see vidstabtransform)"),
    .priv_size     = sizeof(StabData),
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,

    .inputs        = avfilter_vf_vidstabdetect_inputs,
    .outputs       = avfilter_vf_vidstabdetect_outputs,
    .priv_class    = &vidstabdetect_class,
};
