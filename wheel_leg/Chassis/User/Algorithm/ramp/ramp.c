//
// Created by Administrator on 25-10-29.
//

//
// Created by xhuanc on 2021/11/24.
//

#include "ramp.h"
#include "math.h"

void ramp_init(ramp_function_source_t *ramp_source_type, float increase_rate, float max, float min, float decrease_rate)
{
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
    ramp_source_type->decrease_rate = decrease_rate;
    ramp_source_type->increase_rate = increase_rate;
}

void ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;

    if(input > 0)
    {
        if (fabsf(ramp_source_type->input) > ramp_source_type->out)
        {
            ramp_source_type->out += ramp_source_type->increase_rate;
            if (fabsf(ramp_source_type->input - ramp_source_type->out) < ramp_source_type->increase_rate)
            {
                ramp_source_type->out = ramp_source_type->input;
            }
        }
        else if (fabsf(ramp_source_type->input) < ramp_source_type->out)
        {
            ramp_source_type->out -= ramp_source_type->decrease_rate;
            if (fabsf(ramp_source_type->input - ramp_source_type->out) < ramp_source_type->decrease_rate)
            {
                ramp_source_type->out = ramp_source_type->input;
            }
        }
    }
    else if(input < 0)
    {
        if(ramp_source_type->input < ramp_source_type->out) {
            ramp_source_type->out -= ramp_source_type->increase_rate;
            if (fabsf(ramp_source_type->input - ramp_source_type->out) < ramp_source_type->increase_rate)
            {
                ramp_source_type->out = ramp_source_type->input;
            }
        }
        else if(fabsf(ramp_source_type->input) < ramp_source_type->out)
        {
            ramp_source_type->out += ramp_source_type->increase_rate;
            if (fabsf(ramp_source_type->input - ramp_source_type->out) < ramp_source_type->decrease_rate)
            {
                ramp_source_type->out = ramp_source_type->input;
            }
        }
    }
    else
    {
        if(ramp_source_type->out > 0)
        {
            ramp_source_type->out -= ramp_source_type->decrease_rate;
            if(ramp_source_type->out < 0)
            {
                ramp_source_type->out = 0;
            }
        }
        else
        {
            ramp_source_type->out += ramp_source_type->decrease_rate;
            if(ramp_source_type->out > 0)
            {
                ramp_source_type->out = 0;
            }
        }
    }

}