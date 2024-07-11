/*
 * digital_filter.h
 *
 *  Created on: Feb 8, 2024
 *  Modified: July 9, 2024
 *      Author: Angus McLennan
 */

#ifndef DIGITAL_FILTER_H
#define DIGITAL_FILTER_H

#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <vector>

#define MAX_MEDIAN_FILTER_SIZE 10

class MedianFilter
{
public:
	MedianFilter() : size(0), currentIndex(0), filledUp(false), updateFrequency(0.0f), lastUpdateTime(0.0f) {}
    MedianFilter(int size, float updateFrequency);
    void update(float newValue, float updateTime_s);
    float getMedianValue();
    bool isFilledUp();

private:
    float values[MAX_MEDIAN_FILTER_SIZE];
    int size;
    int currentIndex;
    bool filledUp;
    float updateFrequency; // In Hz
    float lastUpdateTime;
};

class ExpLowPassFilter
{
public:
	ExpLowPassFilter() : updateFrequency(0.0f), cutoffFrequency(0.0f), prevOutput(0.0f), initialized(0), alpha(0.0f) {}
    ExpLowPassFilter(float updateFrequency, float cutoffFrequency, float initialInput);
    float update(float input);

private:
    float updateFrequency; // Hz
    float cutoffFrequency; // Hz
    float prevOutput;      // Previous output state
    bool initialized;      // Flag to indicate if the filter has been initialized
    float alpha;
};

#endif // DIGITAL_FILTER_HP
