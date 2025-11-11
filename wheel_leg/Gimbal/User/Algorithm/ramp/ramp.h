//
// Created by xhuanc on 2021/11/24.
//

#ifndef DEMO1_RAMP_H
#define DEMO1_RAMP_H
#include "struct_typedef.h"

typedef  struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
}__packed ramp_function_source_t;

void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);

#endif //DEMO1_RAMP_H
