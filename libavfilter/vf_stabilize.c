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
 *  ffmpeg -vf stabilize=shakiness=5:show=1 -i inp.mpeg -o dummy
 *  all parameters are optional
 */


/*
  TODO: check AVERROR  codes
*/

#define CHROMA_WIDTH(link)  -((-link->w) >> av_pix_fmt_descriptors[link->format].log2_chroma_w)
#define CHROMA_HEIGHT(link) -((-link->h) >> av_pix_fmt_descriptors[link->format].log2_chroma_h)


#define DEFAULT_TRANS_FILE_NAME     "transforms.dat"
#define DS_INPUT_MAXLEN 1024

#include <math.h> //?
#include <libgen.h> //?

#include "avfilter.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavcodec/dsputil.h"

#include "optstr.h"

#include "vid.stab/libdeshake.h"

/* private date structure of this filter*/
typedef struct _stab_data {
    AVClass av_class;

    MotionDetect md;
    AVFilterBufferRef *ref;    ///< Previous frame
    char* options;

    char* result;
    FILE* f;

} StabData;

/*************************************************************************/

/* Module interface routines and data. */

/*************************************************************************/

static av_cold int init(AVFilterContext *ctx, const char *args, void *opaque)
{

    StabData* sd = ctx->priv;

    if (!sd) {
        av_log(ctx, AV_LOG_INFO, "init: out of memory!\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_INFO, "Stabilize: init\n");
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


static int query_formats(AVFilterContext *ctx)
{
    // TODO: check formats and add RGB
    enum PixelFormat pix_fmts[] = {
        PIX_FMT_YUV420P, /* PIX_FMT_YUV422P,  PIX_FMT_YUV444P,  PIX_FMT_YUV410P,
        PIX_FMT_YUV411P,  PIX_FMT_YUV440P,  PIX_FMT_YUVJ420P, PIX_FMT_YUVJ422P,
          PIX_FMT_YUVJ444P, PIX_FMT_YUVJ440P, */PIX_FMT_NONE
    };

    avfilter_set_common_pixel_formats(ctx, avfilter_make_format_list(pix_fmts));

    return 0;
}




static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    StabData *sd = ctx->priv;
//    char* filenamecopy, *filebasename;

    const AVPixFmtDescriptor *desc = &av_pix_fmt_descriptors[inlink->format];
    int bpp = av_get_bits_per_pixel(desc);

    MotionDetect* md = &(sd->md);
    DSFrameInfo fi;
    fi.width=inlink->w;
    fi.height=inlink->h;
    fi.strive=inlink->w;
    // check framesize in ffmpeg

    fi.framesize=(inlink->w*inlink->h*bpp)/8;
    //    PF_RGB=1, PF_YUV = 2
    // TODO: pix format! Also change in my code.
    fi.pFormat = PF_YUV; //420P
    if(initMotionDetect(md, &fi, "stabilize") != DS_OK){
        av_log(ctx, AV_LOG_ERROR, "initialization of Motion Detection failed\n");
        return AVERROR(EINVAL);
    }

    /// TODO: find out input name
    sd->result = av_malloc(DS_INPUT_MAXLEN);
//    filenamecopy = strndup(sd->vob->video_in_file);
//    filebasename = basename(filenamecopy);
//    if (strlen(filebasename) < DS_INPUT_MAXLEN - 4) {
//        snprintf(sd->result, DS_INPUT_MAXLEN, "%s.trf", filebasename);
//} else {
//    av_log(ctx, AV_LOG_WARN, "input name too long, using default `%s'",
//                    DEFAULT_TRANS_FILE_NAME);
    snprintf(sd->result, DS_INPUT_MAXLEN, DEFAULT_TRANS_FILE_NAME);
//    }

    if (sd->options != NULL) {
        ///!CONTINUE
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

    if(configureMotionDetect(md)!= DS_OK){
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
        if(prepareTransformFile(md, sd->f) != DS_OK){
            av_log(ctx, AV_LOG_ERROR, "cannot write to transform file %s!\n", sd->result);
            return AVERROR(EINVAL);
        }
    }

    return 0;
}


static void end_frame(AVFilterLink *link)
{
    AVFilterContext *ctx = link->dst;
    StabData *sd = ctx->priv;
    AVFilterBufferRef *in  = link->cur_buf;
    AVFilterBufferRef *out = ctx->outputs[0]->out_buf;

    MotionDetect* md = &(sd->md);
    LocalMotions localmotions;
    Transform t;

    if(md->show > 0){ // Todo: need function to tell whether read-write access
        memcpy(out->data[0], in->data[0], md->fi.framesize);
        in=out; // work on output and modify it
    }
    if(motionDetection(md, &t, in->data[0]) !=  DS_OK){
    	av_log(ctx, AV_LOG_ERROR, "motion detection failed");
        // Todo: failure
    } else {
        t = simpleMotionsToTransform(md, &localmotions);
        ds_vector_del(&localmotions);
        if(writeTransformToFile(md, sd->f, &t) != DS_OK){
            av_log(ctx, AV_LOG_ERROR, "cannot write to transform file!");
        }
    }

    avfilter_draw_slice(ctx->outputs[0], 0, link->h, 1);
    avfilter_default_end_frame(link);
}


static void draw_slice(AVFilterLink *link, int y, int h, int slice_dir)
{
}


AVFilter avfilter_vf_stabilize = {
    .name      = "stabilize",
    .description = NULL_IF_CONFIG_SMALL("extracts relative transformations of \n\
    subsequent frames (used for stabilization together with the\n\
    transform filter in a second pass)."),

    .priv_size = sizeof(StabData),

    .init = init,
    .uninit = uninit,
    .query_formats = query_formats,

    .inputs    = (const AVFilterPad[]) {{ .name       = "default",
                                    .type             = AVMEDIA_TYPE_VIDEO,
                                    .draw_slice       = draw_slice,
                                    .end_frame        = end_frame,
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
