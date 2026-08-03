#include <assert.h>

#include "testutils.h"
#include "libvidstab.h"
#include "transformtype_operations.h"

void paintRectangle(unsigned char* buffer, const VSFrameInfo* fi, int x, int y, int sizex, int sizey, unsigned char color){
  if(x>=0 && x+sizex < fi->width && y>=0 && y+sizey < fi->height){
    int i,j;
    for(j=y; j < y+sizey; j++){
      for(i=x; i<x+sizex; i++){
  buffer[j*fi->width + i] = color;
      }
    }

  }
}

static uint8_t clip255(int v){
  return (uint8_t)(v<0 ? 0 : (v>255 ? 255 : v));
}

/* BT.601-style fixed point RGB<->YUV conversion. Not required to be bit-exact:
   only used to give synthetic test shapes a consistent, visually distinct
   color across every pixel format, and for the optional PPM dump. */
static void rgbToYuv(uint8_t r, uint8_t g, uint8_t b, uint8_t* y, uint8_t* u, uint8_t* v){
  int yy = (77*r + 150*g + 29*b) >> 8;
  int uu = 128 + ((-43*(int)r - 85*(int)g + 128*(int)b) >> 8);
  int vv = 128 + ((128*(int)r - 107*(int)g - 21*(int)b) >> 8);
  *y = clip255(yy);
  *u = clip255(uu);
  *v = clip255(vv);
}

static void yuvToRgb(uint8_t y, uint8_t u, uint8_t v, uint8_t* r, uint8_t* g, uint8_t* b){
  int d = (int)u - 128;
  int e = (int)v - 128;
  *r = clip255((int)y + ((91881*e) >> 16));
  *g = clip255((int)y - ((22554*d + 46802*e) >> 16));
  *b = clip255((int)y + ((116130*d) >> 16));
}

void setPixelRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y,
                 uint8_t r, uint8_t g, uint8_t b){
  if(x<0 || y<0 || x>=fi->width || y>=fi->height) return;
  if(fi->pFormat < PF_PACKED){
    uint8_t yy,uu,vv;
    rgbToYuv(r,g,b,&yy,&uu,&vv);
    frame->data[0][y*frame->linesize[0] + x] = yy;
    if(fi->planes >= 3){
      int cx = x >> vsGetPlaneWidthSubS(fi,1);
      int cy = y >> vsGetPlaneHeightSubS(fi,1);
      frame->data[1][cy*frame->linesize[1] + cx] = uu;
      frame->data[2][cy*frame->linesize[2] + cx] = vv;
    }
  }else{
    uint8_t* p = frame->data[0] + y*frame->linesize[0] + x*fi->bytesPerPixel;
    switch(fi->pFormat){
     case PF_RGB24: p[0]=r; p[1]=g; p[2]=b; break;
     case PF_BGR24: p[0]=b; p[1]=g; p[2]=r; break;
     case PF_RGBA:  p[0]=r; p[1]=g; p[2]=b; p[3]=255; break;
     default: break;
    }
  }
}

void getPixelRGB(const VSFrame* frame, const VSFrameInfo* fi, int x, int y,
                 uint8_t* r, uint8_t* g, uint8_t* b){
  if(x<0 || y<0 || x>=fi->width || y>=fi->height) { *r=*g=*b=0; return; }
  if(fi->pFormat < PF_PACKED){
    uint8_t yy = frame->data[0][y*frame->linesize[0] + x];
    uint8_t uu = 128, vv = 128;
    if(fi->planes >= 3){
      int cx = x >> vsGetPlaneWidthSubS(fi,1);
      int cy = y >> vsGetPlaneHeightSubS(fi,1);
      uu = frame->data[1][cy*frame->linesize[1] + cx];
      vv = frame->data[2][cy*frame->linesize[2] + cx];
    }
    yuvToRgb(yy,uu,vv,r,g,b);
  }else{
    const uint8_t* p = frame->data[0] + y*frame->linesize[0] + x*fi->bytesPerPixel;
    switch(fi->pFormat){
     case PF_RGB24: *r=p[0]; *g=p[1]; *b=p[2]; break;
     case PF_BGR24: *b=p[0]; *g=p[1]; *r=p[2]; break;
     case PF_RGBA:  *r=p[0]; *g=p[1]; *b=p[2]; break;
     default: *r=*g=*b=0; break;
    }
  }
}

void fillFrameRGB(VSFrame* frame, const VSFrameInfo* fi, uint8_t r, uint8_t g, uint8_t b){
  int x,y;
  for(y=0; y<fi->height; y++)
    for(x=0; x<fi->width; x++)
      setPixelRGB(frame, fi, x, y, r, g, b);
}

void paintCircleRGB(VSFrame* frame, const VSFrameInfo* fi, int cx, int cy, int radius,
                    uint8_t r, uint8_t g, uint8_t b){
  int x,y;
  int r2 = radius*radius;
  for(y=cy-radius; y<=cy+radius; y++){
    for(x=cx-radius; x<=cx+radius; x++){
      int dx=x-cx, dy=y-cy;
      if(dx*dx+dy*dy <= r2)
        setPixelRGB(frame, fi, x, y, r, g, b);
    }
  }
}

void paintSquareRGB(VSFrame* frame, const VSFrameInfo* fi, int x, int y, int size,
                    uint8_t r, uint8_t g, uint8_t b){
  int i,j;
  for(j=y; j<y+size; j++)
    for(i=x; i<x+size; i++)
      setPixelRGB(frame, fi, i, j, r, g, b);
}

int storePPMImage(const char* filename, const VSFrame* frame, const VSFrameInfo* fi){
  FILE* f = fopen(filename, "wb");
  int x,y;
  if(!f){
    vs_log_error("TEST", "Can't open image file '%s'", filename);
    return 0;
  }
  fprintf(f, "P6\n# CREATOR test suite of vid.stab\n%i %i\n255\n", fi->width, fi->height);
  for(y=0; y<fi->height; y++){
    for(x=0; x<fi->width; x++){
      uint8_t rgb[3];
      getPixelRGB(frame, fi, x, y, &rgb[0], &rgb[1], &rgb[2]);
      if(fwrite(rgb, 3, 1, f) != 1){
        vs_log_error("TEST", "Can't write to image file '%s'", filename);
        fclose(f);
        return 0;
      }
    }
  }
  fclose(f);
  return 1;
}

/// corr: correlation length of noise
void fillArrayWithNoise(unsigned char* buffer, int length, float corr){
  unsigned char avg=randPixel();
  int i=0;
  if(corr<1) corr=1;
  float alpha = 1.0/corr;
  for(i=0; i < length; i++){
    buffer[i] = avg;
    avg = avg * (1.0-alpha) + randPixel()*alpha;
  }
}

VSTransform getTestFrameTransform(int i){
  VSTransform t = null_transform();
  t.x = ( (i%2)==0 ? -1 : 1)  *i*5;
  t.y = ( (i%3)==0 ?  1 : -1) *i*5;
  t.alpha = (i<3 ? 0 : 1) * (i)*1*M_PI/(180.0);
  t.zoom = 0;
  return t;
}

static int readNumber (const char* filename, FILE *f)
{
  int c,n=0;
  for(;;) {
    c = fgetc(f);
    if (c==EOF)
      vs_log_error("TEST", "unexpected end of file in '%s'", filename);
    if (c >= '0' && c <= '9') n = n*10 + (c - '0');
    else {
      ungetc (c,f);
      return n;
    }
  }
}


static void skipWhiteSpace (const char* filename, FILE *f)
{
  int c,d;
  for(;;) {
    c = fgetc(f);
    if (c==EOF)
      vs_log_error("TEST", "unexpected end of file in '%s'", filename);

    // skip comments
    if (c == '#') {
      do {
  d = fgetc(f);
  if (d==EOF)
    vs_log_error("TEST", "unexpected end of file in '%s'", filename);
      } while (d != '\n');
      continue;
    }

    if (c > ' ') {
      ungetc (c,f);
      return;
    }
  }
}

int loadPGMImage(const char* filename, VSFrame* frame, VSFrameInfo* fi)
{
  FILE *f = fopen (filename,"rb");
  if (!f) {
    vs_log_error("TEST", "Can't open image file '%s'", filename);
    return 0;
  }

  // read in header
  if (fgetc(f) != 'P' || fgetc(f) != '2')
    vs_log_error("TEST","image file ist not binary PGM (no P5 header) '%s'", filename);
  skipWhiteSpace (filename,f);

  // read in image parameters
  fi->width = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  fi->height = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  int max_value = readNumber (filename,f);

  // check values
  if (fi->width < 1 || fi->height < 1)
    vs_log_error("TEST", "bad image file '%s'", filename);
  if (max_value != 255)
    vs_log_error("TEST", "image file '%s' must have color range 255", filename);

  // read either nothing, LF (10), or CR,LF (13,10)
  int c = fgetc(f);
  if (c == 10) {
    // LF
  }
  else if (c == 13) {
    // CR
    c = fgetc(f);
    if (c != 10) ungetc (c,f);
  }
  else ungetc (c,f);


  // read in rest of data
  vsFrameAllocate(frame,fi);
  if (fread( frame->data[0], fi->width*fi->height, 1, f) != 1){
    vs_log_error("TEST", "Can't read data from image file '%s'", filename);
    return 0;
  }
  fclose (f);
  return 1;
}


int storePGMImage(const char* filename, const uint8_t* data, VSFrameInfo fi ) {
  FILE *f = fopen (filename,"wb");
  if (!f) {
    vs_log_error("TEST", "Can't open image file '%s'",  filename);
    return 0;
  }

  // write header
  fprintf(f,"P5\n");
  fprintf(f,"# CREATOR test suite of vid.stab\n");
  fprintf(f,"%i %i\n", fi.width, fi.height);
  fprintf(f,"255\n");

  // write data
  if (fwrite( data, fi.width*fi.height, 1, f) != 1){
    vs_log_error("TEST", "Can't write to image file '%s'", filename);
    return 0;
  }
  fclose (f);
  return 1;
}

