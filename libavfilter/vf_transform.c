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
 *  ffmpeg -i inp.mpeg ffmpeg -vf transform inp_s.mpeg
 *  all parameters are optional
 */

/*
  TODO: check AVERROR  codes
*/

#define DEFAULT_TRANS_FILE_NAME     "transforms.dat"

#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixfmt.h"
#include "libavcodec/dsputil.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

#include "optstr.h"

#include "vid.stab/libvidstab.h"

#define VS_INPUT_MAXLEN 1024

/* private date structure of this filter*/
typedef struct {
    TransformData td;

    Transformations trans; // transformations
    char* options;
    char input[VS_INPUT_MAXLEN];
} FilterData;


static PixelFormat AV2OurPixelFormat(AVFilterContext *ctx, enum AVPixelFormat pf){
	switch(pf){
    case AV_PIX_FMT_YUV420P: return PF_YUV420P;
		case AV_PIX_FMT_RGB24:		return PF_RGB24;
		case AV_PIX_FMT_BGR24:		return PF_BGR24;
		case AV_PIX_FMT_YUV422P:	return PF_YUV422P;
		case AV_PIX_FMT_YUV444P:	return PF_YUV444P;
		case AV_PIX_FMT_YUV410P:	return PF_YUV410P;
		case AV_PIX_FMT_YUV411P:	return PF_YUV411P;
		case AV_PIX_FMT_GRAY8:		return PF_GRAY8;
		case AV_PIX_FMT_YUVA420P:return PF_YUVA420P;
		case AV_PIX_FMT_RGBA:		return PF_RGBA;
	default:
		av_log(ctx, AV_LOG_ERROR, "cannot deal with pixel format %i!\n", pf);
		return PF_NONE;
	}
}


/*************************************************************************/

/* Module interface routines and data. */

/*************************************************************************/

static av_cold int init(AVFilterContext *ctx, const char *args)
{

    FilterData* fd = ctx->priv;

    if (!fd) {
        av_log(ctx, AV_LOG_INFO, "init: out of memory!\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_INFO, "Transform filter: init %s\n", LIBVIDSTAB_VERSION);

    if(args)
        fd->options=av_strdup(args);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    FilterData *fd = ctx->priv;

    //  avfilter_unref_buffer(fd->ref);

    cleanupTransformData(&fd->td);
    cleanupTransformations(&fd->trans);

    if(fd->options) av_free(fd->options);
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
    FilterData *fd = ctx->priv;
    FILE* f;
//    char* filenamecopy, *filebasename;

    const AVPixFmtDescriptor *desc = &av_pix_fmt_descriptors[inlink->format];

    TransformData* td = &(fd->td);

    DSFrameInfo fi_src;
    DSFrameInfo fi_dest;
    if(!initFrameInfo(&fi_src, inlink->w, inlink->h,
                      AV2OurPixelFormat(ctx,inlink->format)) ||
       !initFrameInfo(&fi_dest, inlink->w, inlink->h,
                      AV2OurPixelFormat(ctx, inlink->format))){
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


    /// TODO: find out input name
//    fd->input = (char*)av_malloc(VS_INPUT_MAXLEN);

//    filenamecopy = strndup(fd->vob->video_in_file);
//    filebasename = basename(filenamecopy);
//    if (strlen(filebasename) < VS_INPUT_MAXLEN - 4) {
//        snprintf(fd->result, VS_INPUT_MAXLEN, "%s.trf", filebasename);
//} else {
//    av_log(ctx, AV_LOG_WARN, "input name too long, using default `%s'",
//                    DEFAULT_TRANS_FILE_NAME);
    snprintf(fd->input, VS_INPUT_MAXLEN, DEFAULT_TRANS_FILE_NAME);
//    }

    if (fd->options != NULL) {
        if(optstr_lookup(fd->options, "help")) {
            av_log(ctx, AV_LOG_INFO, transform_help);
            return AVERROR(EINVAL);
        }

        optstr_get(fd->options, "input",     "%[^:]", fd->input);
        optstr_get(fd->options, "maxshift",  "%d", &td->maxShift);
        optstr_get(fd->options, "maxangle",  "%lf", &td->maxAngle);
        optstr_get(fd->options, "smoothing", "%d", &td->smoothing);
        optstr_get(fd->options, "crop"     , "%d", &td->crop);
        optstr_get(fd->options, "invert"   , "%d", &td->invert);
        optstr_get(fd->options, "relative" , "%d", &td->relative);
        optstr_get(fd->options, "zoom"     , "%lf",&td->zoom);
        optstr_get(fd->options, "optzoom"  , "%d", &td->optZoom);
        optstr_get(fd->options, "interpol" , "%d", (int*)(&td->interpolType));
        optstr_get(fd->options, "sharpen"  , "%lf",&td->sharpen);
        if(optstr_lookup(fd->options, "tripod")){
            av_log(ctx,AV_LOG_INFO, "Virtual tripod mode: relative=False, smoothing=0");
            td->relative=0;
            td->smoothing=0;
        }
    }

    if(configureTransformData(td)!= VS_OK){
    	av_log(ctx, AV_LOG_ERROR, "configuration of Tranform failed\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_INFO, "Image Transformation/Stabilization Settings:\n");
    av_log(ctx, AV_LOG_INFO, "    input     = %s\n", fd->input);
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

    f = fopen(fd->input, "r");
    if (f == NULL) {
        av_log(ctx, AV_LOG_ERROR, "cannot open input file %s!\n", fd->input);
    } else {
        ManyLocalMotions mlms;
        if(readLocalMotionsFile(f,&mlms)==VS_OK){
            // calculate the actual transforms from the localmotions
            if(localmotions2TransformsSimple(td, &mlms,&fd->trans)!=VS_OK)
                av_log(ctx, AV_LOG_ERROR, "calculating transformations failed!\n");
        }else{ // try to read old format
            if (!readOldTransforms(td, f, &fd->trans)) { /* read input file */
                av_log(ctx, AV_LOG_ERROR, "error parsing input file %s!\n", fd->input);
            }
        }
    }
    fclose(f);

    if (preprocessTransforms(td, &fd->trans)!= VS_OK ) {
        av_log(ctx, AV_LOG_ERROR, "error while preprocessing transforms\n");
        return AVERROR(EINVAL);
    }

    // TODO: add sharpening
    return 0;
}


static int filter_frame(AVFilterLink *inlink,  AVFilterBufferRef *in)
{
    AVFilterContext *ctx = inlink->dst;
    FilterData *fd = ctx->priv;
    TransformData* td = &(fd->td);

    AVFilterLink *outlink = inlink->dst->outputs[0];
    //const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    //int hsub0 = desc->log2_chroma_w;
    //int vsub0 = desc->log2_chroma_h;
    int direct = 0;
    AVFilterBufferRef *out;
    DSFrame inframe;
    DSFrame outframe;
    int plane;

    if (in->perms & AV_PERM_WRITE) {
        direct = 1;
        out = in;
    } else {
        out = ff_get_video_buffer(outlink, AV_PERM_WRITE, outlink->w, outlink->h);
        if (!out) {
            avfilter_unref_bufferp(&in);
            return AVERROR(ENOMEM);
        }
        avfilter_copy_buffer_ref_props(out, in);
    }

    for(plane=0; plane < td->fiSrc.planes; plane++){
        inframe.data[plane] = in->data[plane];
        inframe.linesize[plane] = in->linesize[plane];
    }
    for(plane=0; plane < td->fiDest.planes; plane++){
        outframe.data[plane] = out->data[plane];
        outframe.linesize[plane] = out->linesize[plane];
    }


    transformPrepare(td, &inframe, &outframe);

    if (fd->td.fiSrc.pFormat > PF_PACKED) {
        transformRGB(td, getNextTransform(td, &fd->trans));
    } else {
        transformYUV(td, getNextTransform(td, &fd->trans));
    }
    transformFinish(td);

    if (!direct)
        avfilter_unref_bufferp(&in);

    return ff_filter_frame(outlink, out);
}


AVFilter avfilter_vf_transform = {
    .name      = "transform",
    .description = NULL_IF_CONFIG_SMALL("transforms each frame according to transformations\n\
 given in an input file (e.g. translation, rotate) see also filter stabilize."),

    .priv_size = sizeof(FilterData),

    .init = init,
    .uninit = uninit,
    .query_formats = query_formats,

    .inputs    = (const AVFilterPad[]) {
        { .name       = "default",
          .type             = AVMEDIA_TYPE_VIDEO,
          .get_video_buffer = ff_null_get_video_buffer,
          .filter_frame     = filter_frame,
          .config_props     = config_input,
          .min_perms        = AV_PERM_READ | AV_PERM_WRITE,
        },
        { .name = NULL}},
    .outputs   = (const AVFilterPad[]) {
        { .name             = "default",
          .type             = AVMEDIA_TYPE_VIDEO, },
        { .name = NULL}
    },
};



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
