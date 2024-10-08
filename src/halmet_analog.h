#ifndef __SRC_HALMET_ANALOG_H__
#define __SRC_HALMET_ANALOG_H__

#include <WString.h>

#include <Adafruit_ADS1X15.h>

#include <sensesp/system/valueproducer.h>

sensesp::FloatProducer* AnalogResistanceSender(Adafruit_ADS1115* ads1115,
                                               int channel, const String& name);

#endif
