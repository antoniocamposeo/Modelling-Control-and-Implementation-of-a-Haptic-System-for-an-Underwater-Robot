/**
 * @file LowPassFilter.h
 * @author Talha Sarı (talha.sari@outlook.com.tr)
 * @brief A Low-pass Filter Implementation in C
 * @version v1.0
 * @date 2021-04-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

/* Macros */
#define PI              3.141592
#define __DELTAT(fs)    (1.0F / fs)
#define __ALPHA(fc, T)  ((2.0F * PI * T * fc) / ((2.0F * PI * T * fc) + 1.0F))

/* Filter HandleTypeDef */
typedef struct LowPassFilter
{
    float input;
    float alpha;
    float output[2];
} LowPassFilter;

/* Functions */
void LowPassFilter_Init(LowPassFilter *filter, float cutoffFreq, float samplePeriod);
float LowPassFilter_Update(LowPassFilter *filter, float input);

#endif /* LOW_PASS_FILTER_H */