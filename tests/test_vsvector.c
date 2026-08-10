/*
 * test_vsvector.c
 *
 * Regression tests for issue #168: vs_vector_set() at a large position must not
 * overflow int while sizing its buffer, and a failing resize must leave the
 * vector's data/buffersize pair consistent rather than data==NULL with a
 * nonzero buffersize (which the next set() would silently re-initialize).
 */

static int vv_count_nonnull(VSVector* v){
  int i, n = 0;
  for(i=0; i<vs_vector_size(v); i++)
    if(vs_vector_get(v,i)) n++;
  return n;
}

int test_vsvector_bounds(){
  VSVector v;
  int payload = 42;
  void* old;

  /* ---- 1. an implausible position is rejected, not turned into an
       allocation. INT_MAX-1 is what a corrupt frame index in a .trf can
       produce; doubling towards it overflows int. ---- */
  vs_vector_init(&v,4);
  test_bool(vs_vector_set(&v, INT_MAX-1, &payload, 0) == VS_ERROR);
  test_bool(vs_vector_set(&v, INT_MAX,   &payload, 0) == VS_ERROR);
  test_bool(vs_vector_set(&v, -1,        &payload, 0) == VS_ERROR);
  fprintf(stderr,"** implausible vector position rejected OKAY\n");

  /* the vector must be untouched by the rejected calls */
  test_bool(vs_vector_size(&v) == 0);
  test_bool(vs_vector_set(&v, 0, &payload, 0) == VS_OK);
  test_bool(vs_vector_get(&v,0) == &payload);
  test_bool(vs_vector_size(&v) == 1);

  /* ---- 2. a rejected set must not destroy already stored elements ---- */
  test_bool(vs_vector_set(&v, 7, &payload, 0) == VS_OK);
  test_bool(vs_vector_size(&v) == 8);
  test_bool(vs_vector_set(&v, INT_MAX-1, &payload, 0) == VS_ERROR);
  test_bool(vs_vector_size(&v) == 8);          /* not reset to 0 */
  test_bool(vs_vector_get(&v,0) == &payload);  /* still there */
  test_bool(vs_vector_get(&v,7) == &payload);
  test_bool(vv_count_nonnull(&v) == 2);
  fprintf(stderr,"** vector survives a rejected set OKAY\n");

  /* ---- 3. the old element is reported so the caller can free it ---- */
  old = (void*)-1;
  test_bool(vs_vector_set(&v, 3, &payload, &old) == VS_OK);
  test_bool(old == 0);                       /* gap element, was empty */
  test_bool(vs_vector_set(&v, 3, &payload, &old) == VS_OK);
  test_bool(old == &payload);                /* now reports what it replaced */
  vs_vector_fini(&v);

  /* ---- 4. set_dup must not leak its copy when the position is rejected.
       Run it often enough that a leaked copy would show up under valgrind /
       ASan; the point here is that it returns an error at all. ---- */
  {
    int i;
    vs_vector_init(&v,4);
    for(i=0; i<1000; i++)
      test_bool(vs_vector_set_dup(&v, INT_MAX-1, &payload, sizeof(payload), 0)
                == VS_ERROR);
    test_bool(vs_vector_size(&v) == 0);
    test_bool(vs_vector_set_dup(&v, 2, &payload, sizeof(payload), 0) == VS_OK);
    test_bool(*(int*)vs_vector_get(&v,2) == 42);
    vs_vector_del(&v);
    fprintf(stderr,"** set_dup rejects without leaking OKAY\n\n");
  }

  return 1;
}
