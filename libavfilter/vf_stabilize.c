/*
 *  vf_stabilize.c
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
 *  ffmpeg -i input -vf stabilize=shakiness=5:show=1 dummy.avi
 *  all parameters are optional
 */


/*
  TODO: check AVERROR  codes
*/

#define DEFAULT_TRANS_FILE_NAME     "transforms.dat"
#define VS_INPUT_MAXLEN 1024

#include <math.h> //?
#include <libgen.h> //?

#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixfmt.h"
#include "libavutil/imgutils.h"
#include "libavcodec/dsputil.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"


#include "optstr.h"
#include "vid.stab/libvidstab.h"

/* private date structure of this filter*/
typedef struct _stab_data {
    AVClass av_class;

    MotionDetect md;
    AVFilterBufferRef *ref;    ///< Previous frame
    char* options;

    char* result;
    FILE* f;

} StabData;

static PixelFormat AV2OurPixelFormat(AVFilterContext *ctx, enum AVPixelFormat pf){
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


/*************************************************************************/

/* Module interface routines and data. */

/*************************************************************************/

static av_cold int init(AVFilterContext *ctx, const char *args)
{

    StabData* sd = ctx->priv;

    if (!sd) {
        av_log(ctx, AV_LOG_INFO, "init: out of memory!\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_INFO, "Stabilize: init %s\n", LIBVIDSTAB_VERSION);
    if(args)
        sd->options=av_strdup(args);
    else
        sd->options=0;


    return 0;
}

///1
static av_cold void uninit(AVFilterContext *ctx)
{
    StabData *sd = ctx->priv;

    //  avfilter_unref_buffer(sd->ref);

    MotionDetect* md = &(sd->md);
    if (sd->f) {
        fclose(sd->f);
        sd->f = NULL;
    }

    cleanupMotionDetection(md);
    if (sd->result) {
        av_free(sd->result);
        sd->result = NULL;
    }
    if(sd->options) av_free(sd->options);
}

// AVFILTER_DEFINE_CLASS(stabilize);

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
    StabData *sd = ctx->priv;
//    char* filenamecopy, *filebasename;

    MotionDetect* md = &(sd->md);
    VSFrameInfo fi;
    const AVPixFmtDescriptor *desc = &av_pix_fmt_descriptors[inlink->format];

    initFrameInfo(&fi,inlink->w, inlink->h, AV2OurPixelFormat(ctx, inlink->format));
    // check
    if(fi.bytesPerPixel != av_get_bits_per_pixel(desc)/8)
        av_log(ctx, AV_LOG_ERROR, "pixel-format error: wrong bits/per/pixel");
    if(fi.log2ChromaW != desc->log2_chroma_w)
        av_log(ctx, AV_LOG_ERROR, "pixel-format error: log2_chroma_w");
    if(fi.log2ChromaH != desc->log2_chroma_h)
        av_log(ctx, AV_LOG_ERROR, "pixel-format error: log2_chroma_h");

    if(initMotionDetect(md, &fi, "stabilize") != VS_OK){
        av_log(ctx, AV_LOG_ERROR, "initialization of Motion Detection failed");
        return AVERROR(EINVAL);
    }

    /// TODO: find out input name
    sd->result = av_malloc(VS_INPUT_MAXLEN);
//    filenamecopy = strndup(sd->vob->video_in_file);
//    filebasename = basename(filenamecopy);
//    if (strlen(filebasename) < VS_INPUT_MAXLEN - 4) {
//        snprintf(sd->result, VS_INPUT_MAXLEN, "%s.trf", filebasename);
//} else {
//    av_log(ctx, AV_LOG_WARN, "input name too long, using default `%s'",
//                    DEFAULT_TRANS_FILE_NAME);
    snprintf(sd->result, VS_INPUT_MAXLEN, DEFAULT_TRANS_FILE_NAME);
//    }

    if (sd->options != NULL) {
        if(optstr_lookup(sd->options, "help")) {
            av_log(ctx, AV_LOG_INFO, motiondetect_help);
            return AVERROR(EINVAL);
        }

        optstr_get(sd->options, "result",     "%[^:]", sd->result);
        optstr_get(sd->options, "shakiness",  "%d", &md->shakiness);
        optstr_get(sd->options, "accuracy",   "%d", &md->accuracy);
        optstr_get(sd->options, "stepsize",   "%d", &md->stepSize);
        optstr_get(sd->options, "algo",       "%d", &md->algo);
        optstr_get(sd->options, "mincontrast","%lf",&md->contrastThreshold);
        optstr_get(sd->options, "show",       "%d", &md->show);
    }

    if(configureMotionDetect(md)!= VS_OK){
    	av_log(ctx, AV_LOG_ERROR, "configuration of Motion Detection failed\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_INFO, "Image Stabilization Settings:\n");
    av_log(ctx, AV_LOG_INFO, "     shakiness = %d\n", md->shakiness);
    av_log(ctx, AV_LOG_INFO, "      accuracy = %d\n", md->accuracy);
    av_log(ctx, AV_LOG_INFO, "      stepsize = %d\n", md->stepSize);
    av_log(ctx, AV_LOG_INFO, "          algo = %d\n", md->algo);
    av_log(ctx, AV_LOG_INFO, "   mincontrast = %f\n", md->contrastThreshold);
    av_log(ctx, AV_LOG_INFO, "          show = %d\n", md->show);
    av_log(ctx, AV_LOG_INFO, "        result = %s\n", sd->result);

    sd->f = fopen(sd->result, "w");
    if (sd->f == NULL) {
        av_log(ctx, AV_LOG_ERROR, "cannot open transform file %s!\n", sd->result);
        return AVERROR(EINVAL);
    }else{
        if(prepareFile(md, sd->f) != VS_OK){
            av_log(ctx, AV_LOG_ERROR, "cannot write to transform file %s!\n", sd->result);
            return AVERROR(EINVAL);
        }
    }
    return 0;
}


static int filter_frame(AVFilterLink *inlink, AVFilterBufferRef *in)
{
    AVFilterContext *ctx = inlink->dst;
    StabData *sd = ctx->priv;
    MotionDetect* md = &(sd->md);
    LocalMotions localmotions;


    AVFilterLink *outlink = inlink->dst->outputs[0];
    //const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    //int hsub0 = desc->log2_chroma_w;
    //int vsub0 = desc->log2_chroma_h;
    int direct = 0;
    AVFilterBufferRef *out;
    VSFrame frame;
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

    for(plane=0; plane < md->fi.planes; plane++){
        frame.data[plane] = in->data[plane];
        frame.linesize[plane] = in->linesize[plane];
    }
    if(motionDetection(md, &localmotions, &frame) !=  VS_OK){
        av_log(ctx, AV_LOG_ERROR, "motion detection failed");
        return AVERROR(AVERROR_EXTERNAL);
    } else {
        if(writeToFile(md, sd->f, &localmotions) != VS_OK){
            av_log(ctx, AV_LOG_ERROR, "cannot write to transform file!");
            return AVERROR(EPERM);
        }
        vs_vector_del(&localmotions);
    }
    if(md->show>0 && !direct){
        // copy
        av_image_copy(out->data, out->linesize,
                      (void*)in->data, in->linesize,
                      in->format, in->video->w, in->video->h);
    }

    if (!direct)
        avfilter_unref_bufferp(&in);

    return ff_filter_frame(outlink, out);
}


AVFilter avfilter_vf_stabilize = {
    .name      = "stabilize",
    .description = NULL_IF_CONFIG_SMALL("extracts relative transformations of \n\
    subsequent frames (used for stabilization together with the\n\
    transform filter in a second pass)."),

    .priv_size = sizeof(StabData),

    .init   = init,
    .uninit = uninit,
    .query_formats = query_formats,

    .inputs    = (const AVFilterPad[]) {{ .name       = "default",
                                    .type             = AVMEDIA_TYPE_VIDEO,
                                    .get_video_buffer = ff_null_get_video_buffer,
                                    .filter_frame     = filter_frame,
                                    .config_props     = config_input,
                                    .min_perms        = AV_PERM_READ, },
                                  { .name = NULL}},
    .outputs   = (const AVFilterPad[]) {{ .name       = "default",
                                    .type             = AVMEDIA_TYPE_VIDEO, },
                                  { .name = NULL}},
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
