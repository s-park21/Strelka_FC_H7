/*
 * digital_filter.cpp
 *
 *  Created on: Feb 8, 2024
 *  Modified: July 8, 2024
 *      Author: Angus McLennan
 */

#include "Rocket_State_Controller/digital_filter.h"

// MedianFilter class implementation
MedianFilter::MedianFilter(int size, float updateFrequency)
{
    this->size = size > MAX_MEDIAN_FILTER_SIZE ? MAX_MEDIAN_FILTER_SIZE : size;
    this->currentIndex = 0;
    this->filledUp = false;
    this->updateFrequency = updateFrequency;
    this->lastUpdateTime = 0.0;
}

void MedianFilter::update(float newValue, float updateTime_s)
{
    if (updateTime_s - this->lastUpdateTime >= 1 / this->updateFrequency)
    {
        this->lastUpdateTime = updateTime_s;
        this->values[this->currentIndex] = newValue;
        this->currentIndex++;
        if (this->currentIndex >= this->size)
        {
            this->currentIndex = 0;
            this->filledUp = true;
        }
    }
}

float MedianFilter::getMedianValue()
{
    std::vector<float> sortedValues(this->values, this->values + (this->filledUp ? this->size : this->currentIndex));
    std::sort(sortedValues.begin(), sortedValues.end());

    size_t size = sortedValues.size();
    size_t middleIndex = size / 2;

    if (size % 2 == 0)
    {
        return (sortedValues[middleIndex - 1] + sortedValues[middleIndex]) / 2.0;
    }
    else
    {
        return sortedValues[middleIndex];
    }
}

bool MedianFilter::isFilledUp()
{
	return filledUp;
}

// ExpLowPassFilter class implementation
ExpLowPassFilter::ExpLowPassFilter(float updateFrequency, float cutoffFrequency, float initialInput)
{
    this->updateFrequency = updateFrequency;
    this->cutoffFrequency = cutoffFrequency;
    this->prevOutput = initialInput;
    this->alpha = 1.0 - exp(-2.0 * M_PI * this->cutoffFrequency / this->updateFrequency);
    this->initialized = true;
}

float ExpLowPassFilter::update(float input)
{
    float output = this->alpha * input + (1 - this->alpha) * this->prevOutput;
    this->prevOutput = output;
    return output;
}
