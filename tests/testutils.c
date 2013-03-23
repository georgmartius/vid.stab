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

Transform getTestFrameTransform(int i){
  Transform t = null_transform();
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
	allocateFrame(frame,fi);
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

