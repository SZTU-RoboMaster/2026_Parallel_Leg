//
// Created by Administrator on 25-10-29.
//

#ifndef RAMP_H
#define RAMP_H


typedef  struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float increase_rate; //增加步长
    float decrease_rate; //减速步长
}ramp_function_source_t;

void ramp_init(ramp_function_source_t *ramp_source_type, float increase_rate, float max, float min, float decrease_rate);
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);

#endif //RAMP_H
