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
.source 1 s1 uint8_t
.source 1 s2 uint8_t
.accumulator 4 sum int
accsadubl sum, s1, s2



# Image Contrast
#  SUM
# p = pstart;
# for (j = 0; j < field->size; j++) {      
#   for (k = 0; k < field->size; k++, p++) {
#     sum+=*p;
#   }
#   p += (width - field->size);
# }
# mean=sum/numpixel;
# p = pstart;  
#  VARIANCE
# for (j = 0; j < field->size; j++) {      
#   for (k = 0; k < field->size; k++, p++) {
#     var+=abs(*p-mean);
#   }
#   p += (width - field->size);
# }


# Image Contrast functions
# Sum of all pixels (used to calculate mean)
.function image_sum_optimized
.flags 2d
.accumulator 4 sum int
.source 1 s uint8_t
.temp 2 t1 
.temp 4 t2
convubw t1 s
convuwl t2 t1
accl sum, t2

# this implementation appears to be slower
# .function image_sum_optimized
# .flags 2d
# .accumulator 4 sum int
# .source 1 s uint8_t
# .const 1 c1 0
# accsadubl sum, s, c1

# Variance of the image in Manhattan-Norm (absolute value)
.function image_variance_optimized
.flags 2d
.accumulator 4 var int
.source 1 s uint8_t
.param 1 mean uint8_t
accsadubl var, s, mean





