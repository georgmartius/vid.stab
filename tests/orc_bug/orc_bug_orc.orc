.function test_orc
.dest 4 x_ss int32_t    
.dest 4 y_ss int32_t    
.source 4 xs int32_t
.param 4 y_d1 int32_t
.param 4 c_d_x int32_t
.param 4 c_tx int32_t   
.param 4 c_ty int32_t   
.param 4 zcos_a int32_t 
.param 4 zsin_a int32_t 
.temp 4 x_d1
.temp 4 tmp1
.temp 4 tmp2
subl x_d1, xs, c_d_x
mulll tmp1 zcos_a x_d1
mulll tmp2 zsin_a y_d1
addl tmp1 tmp1 tmp2
addl x_ss tmp1 c_tx
mulll tmp1, x_d1, zsin_a # with this line I get a segfault
addl y_ss, tmp1, c_ty
