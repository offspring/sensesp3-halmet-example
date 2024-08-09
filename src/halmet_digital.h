#ifndef __SRC_HALMET_DIGITAL_H__
#define __SRC_HALMET_DIGITAL_H__

#include <WString.h>

#include <sensesp/system/valueproducer.h>

sensesp::FloatProducer* TachoDigitalSender(int pin, const String& path_prefix,
                                           const String& sk_name,
                                           int sort_order_base);
sensesp::BoolProducer* AlarmDigitalSender(int pin, const String& name,
                                          int sort_order_base);

#endif
