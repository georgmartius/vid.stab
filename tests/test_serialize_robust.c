/*
 * test_serialize_robust.c
 *
 * Regression tests for issue #104: a binary transform file that got truncated
 * (which is what MSVCRT does to a binary stream opened in text mode: reading
 * stops at the first 0x1A / Ctrl-Z byte) must fail gracefully instead of
 * inventing null localmotions and flooding the log, and an ascii transform
 * file with CRLF line endings must still be readable from a binary handle.
 */

#define SR_NLM 20
#define SR_NFRAMES 3

static LocalMotions sr_make_lms(int withCtrlZ){
  /* a contrast value that contains a 0x1A byte - completely ordinary data,
     ~0.15% of all bytes of a real transform file are 0x1A */
  static const unsigned char ctrlz_double[8] =
    {0x1a,0x2b,0x3c,0x4d,0x5e,0x6f,0xd1,0x3f};
  LocalMotions lms;
  int i;
  vs_vector_init(&lms, SR_NLM);
  for(i=0; i<SR_NLM; i++){
    LocalMotion lm;
    memset(&lm,0,sizeof(lm));
    lm.v.x     = i - 10;
    lm.v.y     = 10 - i;
    lm.f.x     = 20 + (i%5)*128;
    lm.f.y     = 20 + (i/5)*64;
    lm.f.size  = 32;
    lm.contrast = 0.5;
    lm.match    = 0.25;
    if(withCtrlZ && i==SR_NLM/2)
      memcpy(&lm.contrast, ctrlz_double, 8);
    vs_vector_append_dup(&lms,&lm,sizeof(LocalMotion));
  }
  return lms;
}

static void sr_write_file(const char* name, int serializationMode, int withCtrlZ){
  VSMotionDetect md;
  int fr;
  FILE* f = fopen(name,"wb");
  memset(&md,0,sizeof(md));
  md.serializationMode = serializationMode;
  md.conf.accuracy = 15;
  md.conf.shakiness = 10;
  md.conf.stepSize = 6;
  md.conf.contrastThreshold = 0.25;
  vsPrepareFile(&md,f);
  for(fr=1; fr<=SR_NFRAMES; fr++){
    LocalMotions lms = sr_make_lms(withCtrlZ);
    md.frameNum = fr;
    vsWriteToFile(&md,f,&lms);
    vs_vector_del(&lms);
  }
  fclose(f);
}

static unsigned char* sr_slurp(const char* name, long* len){
  unsigned char* buf;
  FILE* f = fopen(name,"rb");
  if(!f) return 0;
  fseek(f,0,SEEK_END); *len = ftell(f); fseek(f,0,SEEK_SET);
  buf = vs_malloc(*len);
  if(fread(buf,1,*len,f) != (size_t)*len){ fclose(f); vs_free(buf); return 0; }
  fclose(f);
  return buf;
}

static void sr_spit(const char* name, const unsigned char* buf, long len){
  FILE* f = fopen(name,"wb");
  fwrite(buf,1,len,f);
  fclose(f);
}

static void sr_free_mlms(VSManyLocalMotions* mlms){
  int i;
  for(i=0; i<vs_vector_size(mlms); i++)
    if(VSMLMGet(mlms,i)) vs_vector_del(VSMLMGet(mlms,i));
  vs_vector_del(mlms);
}

int test_serialize_robust(){
  long len, i, first1a = -1;
  unsigned char* buf;
  FILE* f;
  VSManyLocalMotions mlms;

  /* ---- 1. binary file round trips ---- */
  sr_write_file(testOut("srtest_bin.trf"), BINARY_SERIALIZATION_MODE, 1);
  buf = sr_slurp(testOut("srtest_bin.trf"),&len);
  test_bool(buf!=0);
  f = fopen(testOut("srtest_bin.trf"),"rb");
  test_bool(f!=0);
  test_bool(vsReadLocalMotionsFile(f,&mlms)==VS_OK);
  test_bool(vs_vector_size(&mlms)==SR_NFRAMES);
  for(i=0; i<vs_vector_size(&mlms); i++)
    test_bool(vs_vector_size(VSMLMGet(&mlms,i))==SR_NLM);
  sr_free_mlms(&mlms);
  fclose(f);
  fprintf(stderr,"** binary file round trip OKAY\n");
  fprintf(stderr,"** the deliberately damaged files below make the library log"
                 " \"Error: ...\": that is what is being tested\n");

  /* ---- 2. truncation at the first 0x1A (Ctrl-Z), as an MSVCRT text-mode
       handle would do: must not produce bogus (null) localmotions ---- */
  for(i=0; i<len; i++) if(buf[i]==0x1a){ first1a = i; break; }
  test_bool(first1a > 0);
  sr_spit(testOut("srtest_trunc.trf"), buf, first1a);
  fprintf(stderr,"** truncating transform file at first 0x1A: %li of %li bytes\n",
          first1a, len);
  f = fopen(testOut("srtest_trunc.trf"),"rb");
  test_bool(f!=0);
  test_bool(vsReadLocalMotionsFile(f,&mlms)==VS_OK);
  for(i=0; i<vs_vector_size(&mlms); i++){
    LocalMotions* lms = VSMLMGet(&mlms,i);
    int j;
    if(!lms) continue;
    for(j=0; j<vs_vector_size(lms); j++){
      /* every restored localmotion must be one that was really in the file,
         i.e. no null_localmotion() placeholders for unreadable records */
      test_bool(LMGet(lms,j)->f.size == 32);
    }
  }
  sr_free_mlms(&mlms);
  fclose(f);
  fprintf(stderr,"** truncated binary file handled gracefully OKAY\n");

  /* ---- 3. a corrupt (implausibly large) list length must be rejected
       instead of being allocated and parsed record by record ---- */
  {
    int32_t bogus = 100000000;
    unsigned char* copy = vs_malloc(len);
    memcpy(copy,buf,len);
    memcpy(copy+24+4,&bogus,4);  /* header(24) + frameNum(4) -> list length */
    sr_spit(testOut("srtest_biglen.trf"), copy, len);
    vs_free(copy);
    f = fopen(testOut("srtest_biglen.trf"),"rb");
    test_bool(f!=0);
    test_bool(vsReadLocalMotionsFile(f,&mlms)==VS_OK);
    test_bool(vs_vector_size(&mlms)>=1);
    test_bool(VSMLMGet(&mlms,0)==0 || vs_vector_size(VSMLMGet(&mlms,0))==0);
    sr_free_mlms(&mlms);
    fclose(f);
    fprintf(stderr,"** corrupt list length rejected OKAY\n");
  }

  /* ---- 3b. a corrupt (implausibly large) *frame index* must be rejected the
       same way instead of being used to size the frame vector (issue #168).
       Without the bound, index-1 goes straight into vs_vector_set(), whose
       growth loop overflows int and asks the allocator for a negative size.
       On a 64-bit host with overcommit that absurd allocation often succeeds
       and the bug stays invisible; on 32-bit it fails, and the failing resize
       used to wipe the vector - which is how this surfaced as an ARM-only
       test failure. ---- */
  {
    int32_t bogus = INT_MAX-1;
    unsigned char* copy = vs_malloc(len);
    memcpy(copy,buf,len);
    memcpy(copy+24,&bogus,4);   /* header(24) -> frameNum of the first record */
    sr_spit(testOut("srtest_bigindex.trf"), copy, len);
    vs_free(copy);
    f = fopen(testOut("srtest_bigindex.trf"),"rb");
    test_bool(f!=0);
    test_bool(vsReadLocalMotionsFile(f,&mlms)==VS_OK);
    /* the bogus record is dropped, the two well-formed ones that follow it
       are still read - and the vector must not have been reset in between */
    test_bool(vs_vector_size(&mlms)==SR_NFRAMES);
    test_bool(VSMLMGet(&mlms,0)==0);
    for(i=1; i<vs_vector_size(&mlms); i++)
      test_bool(vs_vector_size(VSMLMGet(&mlms,i))==SR_NLM);
    sr_free_mlms(&mlms);
    fclose(f);
    fprintf(stderr,"** corrupt frame index rejected OKAY\n");
  }
  vs_free(buf);

  /* ---- 4. ascii file with CRLF line endings read from a binary handle ---- */
  sr_write_file(testOut("srtest_ascii.trf"), ASCII_SERIALIZATION_MODE, 0);
  buf = sr_slurp(testOut("srtest_ascii.trf"),&len);
  test_bool(buf!=0);
  {
    unsigned char* crlf = vs_malloc(2*len);
    long k = 0;
    for(i=0; i<len; i++){
      if(buf[i]=='\n') crlf[k++] = '\r';
      crlf[k++] = buf[i];
    }
    sr_spit(testOut("srtest_ascii_crlf.trf"), crlf, k);
    vs_free(crlf);
  }
  vs_free(buf);
  f = fopen(testOut("srtest_ascii_crlf.trf"),"rb");
  test_bool(f!=0);
  test_bool(vsReadLocalMotionsFile(f,&mlms)==VS_OK);
  test_bool(vs_vector_size(&mlms)==SR_NFRAMES);
  for(i=0; i<vs_vector_size(&mlms); i++)
    test_bool(vs_vector_size(VSMLMGet(&mlms,i))==SR_NLM);
  sr_free_mlms(&mlms);
  fclose(f);
  fprintf(stderr,"** ascii file with CRLF line endings OKAY\n\n");

  return 1;
}
