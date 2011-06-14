################################################################################
# Optimized functions 
################################################################################

# Image Difference
#
#  for (j = 0; j < field->size; j++) {
#    for (k = 0; k < field->size * bytesPerPixel; k++) {
#      sum += abs((int) *p1 - (int) *p2);
#      p1++;
#      p2++;
#    }
#    p1 += (width - field->size) * bytesPerPixel;
#    p2 += (width - field->size) * bytesPerPixel;
#  }

.function image_difference_optimized
.flags 2d
.source 1 s1 char
.source 1 s2 char
.accumulator 4 sum int
accsadubl sum, s1, s2
