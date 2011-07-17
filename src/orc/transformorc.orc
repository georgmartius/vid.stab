################################################################################
# Optimized functions 
################################################################################

# Hint: use only one space between opcode and operands (and also between them)

# Rotation and Translation of one line

# for (x = 0; x < td->fiDest.width; x++) {
# 	int32_t x_d1 = (xs[x] - c_d_x);
# 	x_ss[x]  =  zcos_a * x_d1 + zsin_a * y_d1 + c_tx;
# 	y_ss[x]  = -zsin_a * x_d1 + zcos_a * y_d1 + c_ty;
# }


.function transform_one_line_optimized
.dest 4 x_ss int32_t    # fp16
.dest 4 y_ss int32_t    # fp16
.source 4 xs int32_t
.param 4 y_d1 int32_t
.param 4 c_d_x int32_t
.param 4 c_tx int32_t   # fp16
.param 4 c_ty int32_t   # fp16
.param 4 zcos_a int32_t # fp16
.param 4 zsin_a int32_t # fp16
.temp 4 x_d1
.temp 4 tmp1
.temp 4 tmp2

subl x_d1, xs, c_d_x
mulll tmp1 zcos_a x_d1
mulll tmp2 zsin_a y_d1
addl tmp1 tmp1 tmp2
addl x_ss tmp1 c_tx
mulll tmp1 zcos_a y_d1
mulll tmp2 zsin_a x_d1
subl tmp1 tmp1 tmp2
addl y_ss tmp1 c_ty


.function transform_one_line_optimized1
.dest 4 x_ss int32_t    # fp16
.source 4 xs int32_t
.param 4 y_d1 int32_t
.param 4 c_d_x int32_t
.param 4 c_tx int32_t   # fp16
.param 4 c_ty int32_t   # fp16
.param 4 zcos_a int32_t # fp16
.param 4 zsin_a int32_t # fp16
.param 4 sin_y int32_t # fp16
.param 4 cos_y int32_t # fp16
.temp 4 x_d1
.temp 4 tmp1
.temp 4 tmp2
subl x_d1, xs, c_d_x
mulll tmp1, x_d1, zcos_a
addl tmp1, tmp1, sin_y
addl x_ss, tmp1, c_tx
mulll tmp1, x_d1, zsin_a
mulll tmp1, tmp1, -1
addl tmp2, tmp1, cos_y
# addl y_ss, tmp1, c_ty



