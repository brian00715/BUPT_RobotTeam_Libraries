#ifndef __SW_UTILS_H_
#define __SW_UTILS_H_

#include "utils.h"
#include "stdint.h"
#include "main.h"
#include "cmd.h"

/** @brief 数组求平均值*/
#define AR_AVE(AR, N, RESULT)   \
    RESULT = 0;                 \
    for (int i = 0; i < N; i++) \
    {                           \
        RESULT += AR[i];        \
    }                           \
    RESULT = RESULT / N;

int16_t Sum_int16ar(int16_t ar[], int n);
float Ave_int16ar(int16_t ar[], int n);
float Ave_floatar(float ar[], int n);
void PrintHallSwitchState();

#endif