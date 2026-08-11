/* global_motions.trf round-trip tests.

   Included as a translation unit by tests.c, so it needs no includes of its
   own.

   vsMotionsToTransform() optionally dumps each transform to a FILE*; with
   vidstabtransform's debug=1 that file is global_motions.trf. It is not merely
   a log: it is written in the deprecated transform format that
   vsReadOldTransforms() reads back, and is the supported way to feed a
   camera path computed elsewhere into the second pass (see
   docs/trf-format.md). So the invariant is that the file describes the
   transforms the function actually *returned*.

   That invariant used to be broken for `zoom`: the dump was written before the
   `if(!smoothZoom) t.zoom=0` gate, so with the default smoothZoom=0 the file
   recorded a per-frame zoom estimate the pipeline immediately discarded.
   Replaying such a dump applied a zoom the original run never did -- through
   ffmpeg that cost SSIM 0.93 against the run the file came from, where zeroing
   the column by hand gave 0.999993. See issue #89. */

#define GM_FRAMES    6
/* the transforms are written with "%f", i.e. 6 decimals, so the round trip is
   exact only to within that rounding */
#define GM_TOL       1e-5
/* a zoom estimate has to be at least this large for the smoothZoom=0 case to
   be a meaningful regression test rather than a vacuous one */
#define GM_MIN_ZOOM  0.01

/* Local motions describing a pure zoom about the frame centre: field at
   (fx,fy) moves radially outward by scale*(distance from centre). A plain
   translation would leave the zoom estimate at zero and the test could not
   distinguish the bug from correct behaviour. */
static LocalMotions gmSyntheticMotions(const VSFrameInfo* fi, double scale){
  LocalMotions lms;
  int gx, gy;
  const int cx = fi->width/2, cy = fi->height/2;
  vs_vector_init(&lms, 16);
  for(gy=0; gy<4; gy++){
    for(gx=0; gx<4; gx++){
      LocalMotion lm;
      lm.f.x    = (int16_t)(fi->width  * (gx+1) / 5);
      lm.f.y    = (int16_t)(fi->height * (gy+1) / 5);
      lm.f.size = 32;
      lm.v.x    = (int16_t)lround((lm.f.x - cx) * scale);
      lm.v.y    = (int16_t)lround((lm.f.y - cy) * scale);
      lm.contrast = 0.5;
      lm.match    = 1.0;
      vs_vector_append_dup(&lms, &lm, sizeof(LocalMotion));
    }
  }
  return lms;
}

/* Writes GM_FRAMES transforms through the dump, reads them back with the same
   reader ffmpeg's vidstabtransform uses, and requires the two to agree.
   Returns the number of frames whose written zoom was non-negligible, so the
   caller can assert the input actually exercises the zoom path. */
static int gmCheckRoundTrip(int smoothZoom){
  VSFrameInfo fi;
  VSTransformConfig conf;
  VSTransformData td;
  VSTransformations trans;
  VSTransform returned[GM_FRAMES];
  const char* path = testOut("global_motions_roundtrip.trf");
  FILE* f;
  int i, nonzeroZoom = 0;

  fprintf(stderr, "--- global_motions round trip, smoothZoom=%i ---\n", smoothZoom);
  test_bool(vsFrameInfoInit(&fi, 640, 360, PF_YUV420P) != 0);
  conf = vsTransformGetDefaultConfig("test_globalmotions");
  conf.smoothZoom = smoothZoom;
  test_bool(vsTransformDataInit(&td, &conf, &fi, &fi) == VS_OK);

  /* binary mode on purpose: the contract in serialize.h applies to the ascii
     format too, so that a file written here stays readable on Windows */
  f = fopen(path, "wb");
  test_bool(f != 0);
  for(i=0; i<GM_FRAMES; i++){
    LocalMotions lms = gmSyntheticMotions(&fi, 0.01*(i+1));
    returned[i] = vsMotionsToTransform(&td, &lms, f);
    vs_vector_del(&lms);
    if(fabs(returned[i].zoom) > GM_MIN_ZOOM) nonzeroZoom++;
  }
  fclose(f);

  vsTransformationsInit(&trans);
  f = fopen(path, "rb");
  test_bool(f != 0);
  test_bool(vsReadOldTransforms(&td, f, &trans) == GM_FRAMES);
  fclose(f);

  for(i=0; i<GM_FRAMES && i<trans.len; i++){
    VSTransform d = sub_transforms(&trans.ts[i], &returned[i]);
    int ok = fabs(d.x)     < GM_TOL && fabs(d.y)    < GM_TOL
          && fabs(d.alpha) < GM_TOL && fabs(d.zoom) < GM_TOL;
    if(!ok){
      fprintf(stderr, "frame %i returned: ", i); storeVSTransform(stderr, &returned[i]);
      fprintf(stderr, "          in file: ");    storeVSTransform(stderr, &trans.ts[i]);
    }
    test_bool(ok);
  }

  vsTransformationsCleanup(&trans);
  vsTransformDataCleanup(&td);
  return nonzeroZoom;
}

void test_globalmotions_roundtrip(void){
  int i, zeroed = 1;

  /* smoothZoom=1 keeps the estimate, so this run both checks the round trip
     and establishes that this input really does produce a zoom. Without that
     the smoothZoom=0 case below would pass even if the estimator returned
     zero throughout, which is exactly the way this regression could hide. */
  test_bool(gmCheckRoundTrip(1) > 0);

  /* The default. The gate zeroes the zoom, so the file must record zero too --
     this is the assertion that failed before the fix. */
  test_bool(gmCheckRoundTrip(0) == 0);

  /* and the returned transforms really are zoom-free under the default, so
     "the file matches" cannot be satisfied by both sides being wrong */
  {
    VSFrameInfo fi;
    VSTransformConfig conf = vsTransformGetDefaultConfig("test_globalmotions_gate");
    VSTransformData td;
    test_bool(vsFrameInfoInit(&fi, 640, 360, PF_YUV420P) != 0);
    test_bool(vsTransformDataInit(&td, &conf, &fi, &fi) == VS_OK);
    test_bool(conf.smoothZoom == 0);   /* documents the default this relies on */
    for(i=0; i<GM_FRAMES; i++){
      LocalMotions lms = gmSyntheticMotions(&fi, 0.01*(i+1));
      VSTransform t = vsMotionsToTransform(&td, &lms, 0);
      vs_vector_del(&lms);
      if(t.zoom != 0.0) zeroed = 0;
    }
    test_bool(zeroed);
    vsTransformDataCleanup(&td);
  }
}
