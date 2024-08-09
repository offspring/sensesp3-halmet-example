#ifndef HALMET_SRC_N2K_SENDERS_H_
#define HALMET_SRC_N2K_SENDERS_H_

#ifdef ENABLE_NMEA2000_OUTPUT
#include "expiring_value.h"

#include <N2kMessages.h>
#include <NMEA2000.h>

#include <sensesp/system/configurable.h>
#include <sensesp/system/lambda_consumer.h>
#include <sensesp/system/local_debug.h>

#include <cstdint>

namespace halmet {

/**
 * @brief Base class for NMEA 2000 senders.
 *
 */
class N2kSender : public sensesp::Configurable {
 public:
  N2kSender(const String& config_path) : sensesp::Configurable{config_path} {}

  virtual void enable() = 0;

  void disable() {
    if (this->sender_reaction_ != nullptr) {
      reactesp::ReactESP::app->remove(this->sender_reaction_);
      this->sender_reaction_ = nullptr;
    }
  }

 protected:
  sensesp::RepeatReaction* sender_reaction_ = nullptr;
};

/**
 * @brief Transmit NMEA 2000 PGN 127488: Engine Parameters, Rapid Update
 *
 */
class N2kEngineParameterRapidSender : public N2kSender {
 public:
  N2kEngineParameterRapidSender(const String& config_path,
                                uint8_t engine_instance, tNMEA2000* nmea2000,
                                bool enable = true)
      : N2kSender{config_path},
        engine_instance_{engine_instance},
        nmea2000_{nmea2000},
        repeat_interval_{100},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{1000},          // In ms. When the inputs expire.
        engine_speed_{N2kDoubleNA, expiry_, N2kDoubleNA},
        engine_boost_pressure_{N2kDoubleNA, expiry_, N2kDoubleNA},
        engine_tilt_trim_{N2kInt8NA, expiry_, N2kInt8NA} {
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    reactesp::ReactESP::app->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      // At the moment, the PGN is sent regardless of whether all the
      // values are invalid or not.
      SetN2kEngineParamRapid(
          N2kMsg, this->engine_instance_, this->engine_speed_.get(),
          this->engine_boost_pressure_.get(), this->engine_tilt_trim_.get());
      this->nmea2000_->SendMsg(N2kMsg);
    });
  }

  sensesp::LambdaConsumer<double> engine_speed_consumer_{
      [this](double value) { this->engine_speed_.update(value); }};

  sensesp::LambdaConsumer<double> engine_boost_pressure_consumer_{
      [this](double value) { this->engine_boost_pressure_.update(value); }};

  sensesp::LambdaConsumer<int8_t> engine_tilt_trim_consumer_{
      [this](int8_t value) { this->engine_tilt_trim_.update(value); }};

  String get_config_schema() override {
    return R"###({
    "type": "object",
    "properties": {
      "engine_instance": { "title": "Engine instance", "type": "integer" }
    }
  })###";
  }

  bool set_configuration(const JsonObject& config) override {
    const String expected[] = {"engine_instance"};
    for (const auto& str : expected) {
      if (!config.containsKey(str)) {
        debugE(
            "N2kEngineParameterRapidSender: Missing configuration key "
            "%s",
            str.c_str());
        return false;
      }
    }
    engine_instance_ = config["engine_instance"];
    return true;
  }

  void get_configuration(JsonObject& config) override {
    config["engine_instance"] = engine_instance_;
  }

 protected:
  uint32_t repeat_interval_;
  uint32_t expiry_;
  tNMEA2000* nmea2000_;

  uint8_t engine_instance_;
  sensesp::ExpiringValue<double> engine_speed_;
  sensesp::ExpiringValue<double> engine_boost_pressure_;
  sensesp::ExpiringValue<int8_t> engine_tilt_trim_;
};

/**
 * @brief Transmit NMEA 2000 PGN 127489: Engine Parameters, Dynamic
 *
 */
class N2kEngineParameterDynamicSender : public N2kSender {
 public:
  N2kEngineParameterDynamicSender(const String& config_path,
                                  uint8_t engine_instance, tNMEA2000* nmea2000,
                                  bool enable = true)
      : N2kSender{config_path},
        engine_instance_{engine_instance},
        nmea2000_{nmea2000},
        repeat_interval_{500},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{5000},          // In ms. When the inputs expire.
        oil_pressure_{N2kDoubleNA, expiry_, N2kDoubleNA},
        oil_temperature_{N2kDoubleNA, expiry_, N2kDoubleNA},
        coolant_temperature_{N2kDoubleNA, expiry_, N2kDoubleNA},
        alternator_voltage_{N2kDoubleNA, expiry_, N2kDoubleNA},
        fuel_rate_{N2kDoubleNA, expiry_, N2kDoubleNA},
        total_engine_hours_{N2kDoubleNA, expiry_, N2kDoubleNA},
        coolant_pressure_{N2kDoubleNA, expiry_, N2kDoubleNA},
        fuel_pressure_{N2kDoubleNA, expiry_, N2kDoubleNA},
        engine_load_{N2kInt8NA, expiry_, N2kInt8NA},
        engine_torque_{N2kInt8NA, expiry_, N2kInt8NA},
        // Status bits 1
        check_engine_{false, expiry_, false},
        over_temperature_{false, expiry_, false},
        low_oil_pressure_{false, expiry_, false},
        low_oil_level_{false, expiry_, false},
        low_fuel_pressure_{false, expiry_, false},
        low_system_voltage_{false, expiry_, false},
        low_coolant_level_{false, expiry_, false},
        water_flow_{false, expiry_, false},
        water_in_fuel_{false, expiry_, false},
        charge_indicator_{false, expiry_, false},
        preheat_indicator_{false, expiry_, false},
        high_boost_pressure_{false, expiry_, false},
        rev_limit_exceeded_{false, expiry_, false},
        egr_system_{false, expiry_, false},
        throttle_position_sensor_{false, expiry_, false},
        emergency_stop_{false, expiry_, false},
        warning_level_1_{false, expiry_, false},
        warning_level_2_{false, expiry_, false},
        // Status bits 2
        low_oi_power_reduction_{false, expiry_, false},
        maintenance_needed_{false, expiry_, false},
        engine_comm_error_{false, expiry_, false},
        sub_or_secondary_throttle_{false, expiry_, false},
        neutral_start_protect_{false, expiry_, false},
        engine_shutting_down_{false, expiry_, false} {
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_reaction_ == nullptr) {
      this->sender_reaction_ =
          reactesp::ReactESP::app->onRepeat(repeat_interval_, [this]() {
            tN2kMsg N2kMsg;
            SetN2kEngineDynamicParam(
                N2kMsg, this->engine_instance_, this->oil_pressure_.get(),
                this->oil_temperature_.get(), this->coolant_temperature_.get(),
                this->alternator_voltage_.get(), this->fuel_rate_.get(),
                this->total_engine_hours_.get(), this->coolant_pressure_.get(),
                this->fuel_pressure_.get(), this->engine_load_.get(),
                this->engine_torque_.get(), this->get_engine_status_1(),
                this->get_engine_status_2());
            this->nmea2000_->SendMsg(N2kMsg);
          });
    }
  }

// Define a macro for defining the consumer functions for each parameter.
#define DEFINE_CONSUMER(name, type)               \
  sensesp::LambdaConsumer<type> name##_consumer_{ \
      [this](type value) { this->name##_.update(value); }};
  DEFINE_CONSUMER(oil_pressure, double)
  DEFINE_CONSUMER(oil_temperature, double)
  DEFINE_CONSUMER(coolant_temperature, double)
  DEFINE_CONSUMER(alternator_voltage, double)
  DEFINE_CONSUMER(fuel_rate, double)
  DEFINE_CONSUMER(total_engine_hours, double)
  DEFINE_CONSUMER(coolant_pressure, double)
  DEFINE_CONSUMER(fuel_pressure, double)
  DEFINE_CONSUMER(engine_load, int8_t)
  DEFINE_CONSUMER(engine_torque, int8_t)
  // Status bits 1
  DEFINE_CONSUMER(check_engine, bool)
  DEFINE_CONSUMER(over_temperature, bool)
  DEFINE_CONSUMER(low_oil_pressure, bool)
  DEFINE_CONSUMER(low_oil_level, bool)
  DEFINE_CONSUMER(low_fuel_pressure, bool)
  DEFINE_CONSUMER(low_system_voltage, bool)
  DEFINE_CONSUMER(low_coolant_level, bool)
  DEFINE_CONSUMER(water_flow, bool)
  DEFINE_CONSUMER(water_in_fuel, bool)
  DEFINE_CONSUMER(charge_indicator, bool)
  DEFINE_CONSUMER(preheat_indicator, bool)
  DEFINE_CONSUMER(high_boost_pressure, bool)
  DEFINE_CONSUMER(rev_limit_exceeded, bool)
  DEFINE_CONSUMER(egr_system, bool)
  DEFINE_CONSUMER(throttle_position_sensor, bool)
  DEFINE_CONSUMER(emergency_stop, bool)
  // Status bits 2
  DEFINE_CONSUMER(warning_level_1, bool)
  DEFINE_CONSUMER(warning_level_2, bool)
  DEFINE_CONSUMER(low_oi_power_reduction, bool)
  DEFINE_CONSUMER(maintenance_needed, bool)
  DEFINE_CONSUMER(engine_comm_error, bool)
  DEFINE_CONSUMER(sub_or_secondary_throttle, bool)
  DEFINE_CONSUMER(neutral_start_protect, bool)
  DEFINE_CONSUMER(engine_shutting_down, bool)
#undef DEFINE_CONSUMER

  String get_config_schema() override {
    return R"###({
    "type": "object",
    "properties": {
      "engine_instance": { "title": "Engine instance", "type": "integer" }
    }
  })###";
  }

  bool set_configuration(const JsonObject& config) override {
    const String expected[] = {"engine_instance"};
    for (const auto& str : expected) {
      if (!config.containsKey(str)) {
        debugE(
            "N2kEngineParameterRapidSender: Missing configuration key "
            "%s",
            str.c_str());
        return false;
      }
    }
    engine_instance_ = config["engine_instance"];
    return true;
  }

  void get_configuration(JsonObject& config) override {
    config["engine_instance"] = engine_instance_;
  }

 protected:
  sensesp::Reaction* sender_reaction_ = nullptr;

  tN2kEngineDiscreteStatus1 get_engine_status_1() {
    tN2kEngineDiscreteStatus1 status1 = 0;

    // Get the status from each of the sensor checks
#if 1  // HACK HACK
    uint16_t status_bits = 0;
    status_bits |= 0x0001 * check_engine_.get();
    status_bits |= 0x0002 * over_temperature_.get();
    status_bits |= 0x0004 * low_oil_pressure_.get();
    status_bits |= 0x0008 * low_oil_level_.get();
    status_bits |= 0x0010 * low_fuel_pressure_.get();
    status_bits |= 0x0020 * low_system_voltage_.get();
    status_bits |= 0x0040 * low_coolant_level_.get();
    status_bits |= 0x0080 * water_flow_.get();
    status_bits |= 0x0100 * water_in_fuel_.get();
    status_bits |= 0x0200 * charge_indicator_.get();
    status_bits |= 0x0400 * preheat_indicator_.get();
    status_bits |= 0x0800 * high_boost_pressure_.get();
    status_bits |= 0x1000 * rev_limit_exceeded_.get();
    status_bits |= 0x2000 * egr_system_.get();
    status_bits |= 0x4000 * throttle_position_sensor_.get();
    status_bits |= 0x8000 * emergency_stop_.get();
    status1.Status = status_bits;
#else
    status1.Bits.CheckEngine = check_engine_.get();
    status1.Bits.OverTemperature = over_temperature_.get();
    status1.Bits.LowOilPressure = low_oil_pressure_.get();
    status1.Bits.LowOilLevel = low_oil_level_.get();
    status1.Bits.LowFuelPressure = low_fuel_pressure_.get();
    status1.Bits.LowSystemVoltage = low_system_voltage_.get();
    status1.Bits.LowCoolantLevel = low_coolant_level_.get();
    status1.Bits.WaterFlow = water_flow_.get();
    status1.Bits.WaterInFuel = water_in_fuel_.get();
    status1.Bits.ChargeIndicator = charge_indicator_.get();
    status1.Bits.PreheatIndicator = preheat_indicator_.get();
    status1.Bits.HighBoostPressure = high_boost_pressure_.get();
    status1.Bits.RevLimitExceeded = rev_limit_exceeded_.get();
    status1.Bits.EGRSystem = egr_system_.get();
    status1.Bits.ThrottlePositionSensor = throttle_position_sensor_.get();
    status1.Bits.EngineEmergencyStopMode = emergency_stop_.get();

    // Set CheckEngine if any other status bit is set
    status1.Bits.CheckEngine =
        status1.Bits.OverTemperature || status1.Bits.LowOilPressure ||
        status1.Bits.LowOilLevel || status1.Bits.LowFuelPressure ||
        status1.Bits.LowSystemVoltage || status1.Bits.LowCoolantLevel ||
        status1.Bits.WaterFlow || status1.Bits.WaterInFuel ||
        status1.Bits.ChargeIndicator || status1.Bits.PreheatIndicator ||
        status1.Bits.HighBoostPressure || status1.Bits.RevLimitExceeded ||
        status1.Bits.EGRSystem || status1.Bits.ThrottlePositionSensor ||
        status1.Bits.EngineEmergencyStopMode;
#endif
    return status1;
  }

  tN2kEngineDiscreteStatus2 get_engine_status_2() {
    tN2kEngineDiscreteStatus2 status2 = 0;

#if 1  // HACK HACK
    uint16_t status_bits = 0;
    status_bits |= 0x0001 * warning_level_1_.get();
    status_bits |= 0x0002 * warning_level_2_.get();
    status_bits |= 0x0004 * low_oi_power_reduction_.get();
    status_bits |= 0x0008 * maintenance_needed_.get();
    status_bits |= 0x0010 * engine_comm_error_.get();
    status_bits |= 0x0020 * sub_or_secondary_throttle_.get();
    status_bits |= 0x0040 * neutral_start_protect_.get();
    status_bits |= 0x0080 * engine_shutting_down_.get();
    status2.Status = status_bits;
#else
    status2.Bits.WarningLevel1 = warning_level_1_.get();
    status2.Bits.WarningLevel2 = warning_level_2_.get();
    status2.Bits.LowOiPowerReduction = low_oi_power_reduction_.get();
    status2.Bits.MaintenanceNeeded = maintenance_needed_.get();
    status2.Bits.EngineCommError = engine_comm_error_.get();
    status2.Bits.SubOrSecondaryThrottle = sub_or_secondary_throttle_.get();
    status2.Bits.NeutralStartProtect = neutral_start_protect_.get();
    status2.Bits.EngineShuttingDown = engine_shutting_down_.get();
#endif
    return status2;
  }

  uint32_t repeat_interval_;
  uint32_t expiry_;
  tNMEA2000* nmea2000_;

  uint8_t engine_instance_;
  // Data to be transmitted
  sensesp::ExpiringValue<double> oil_pressure_;
  sensesp::ExpiringValue<double> oil_temperature_;
  sensesp::ExpiringValue<double> coolant_temperature_;
  sensesp::ExpiringValue<double> alternator_voltage_;
  sensesp::ExpiringValue<double> fuel_rate_;
  sensesp::ExpiringValue<double> total_engine_hours_;
  sensesp::ExpiringValue<double> coolant_pressure_;
  sensesp::ExpiringValue<double> fuel_pressure_;
  sensesp::ExpiringValue<int8_t> engine_load_;
  sensesp::ExpiringValue<int8_t> engine_torque_;
  // Engine status 1 fields
  sensesp::ExpiringValue<bool> check_engine_;
  sensesp::ExpiringValue<bool> over_temperature_;
  sensesp::ExpiringValue<bool> low_oil_pressure_;
  sensesp::ExpiringValue<bool> low_oil_level_;
  sensesp::ExpiringValue<bool> low_fuel_pressure_;
  sensesp::ExpiringValue<bool> low_system_voltage_;
  sensesp::ExpiringValue<bool> low_coolant_level_;
  sensesp::ExpiringValue<bool> water_flow_;
  sensesp::ExpiringValue<bool> water_in_fuel_;
  sensesp::ExpiringValue<bool> charge_indicator_;
  sensesp::ExpiringValue<bool> preheat_indicator_;
  sensesp::ExpiringValue<bool> high_boost_pressure_;
  sensesp::ExpiringValue<bool> rev_limit_exceeded_;
  sensesp::ExpiringValue<bool> egr_system_;
  sensesp::ExpiringValue<bool> throttle_position_sensor_;
  sensesp::ExpiringValue<bool> emergency_stop_;
  // Engine status 2 fields
  sensesp::ExpiringValue<bool> warning_level_1_;
  sensesp::ExpiringValue<bool> warning_level_2_;
  sensesp::ExpiringValue<bool> low_oi_power_reduction_;
  sensesp::ExpiringValue<bool> maintenance_needed_;
  sensesp::ExpiringValue<bool> engine_comm_error_;
  sensesp::ExpiringValue<bool> sub_or_secondary_throttle_;
  sensesp::ExpiringValue<bool> neutral_start_protect_;
  sensesp::ExpiringValue<bool> engine_shutting_down_;
};

const char kN2kFluidLevelTankTypes[][12] = {
    "Fuel", "Water", "Gray water", "Live well", "Oil", "Black water"};

/**
 * @brief Transmit NMEA 2000 PGN 127505: Fluid Level
 *
 */
class N2kFluidLevelSender : public N2kSender {
 public:
  N2kFluidLevelSender(const String& config_path, uint8_t tank_instance,
                      tN2kFluidType tank_type, double tank_capacity,
                      tNMEA2000* nmea2000, bool enable = true)
      : N2kSender{config_path},
        tank_instance_{tank_instance},
        tank_type_{tank_type},
        tank_capacity_{tank_capacity},
        nmea2000_{nmea2000},
        repeat_interval_{2500},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{10000}           // In ms. When the inputs expire.
  {
    tank_level_ =
        sensesp::ExpiringValue<double>(N2kDoubleNA, expiry_, N2kDoubleNA);
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_reaction_ == nullptr) {
      this->sender_reaction_ =
          reactesp::ReactESP::app->onRepeat(repeat_interval_, [this]() {
            tN2kMsg N2kMsg;
            // At the moment, the PGN is sent regardless of whether all
            // the values are invalid or not.
            SetN2kFluidLevel(N2kMsg, this->tank_instance_, this->tank_type_,
                             this->tank_level_.get(), this->tank_capacity_);
            this->nmea2000_->SendMsg(N2kMsg);
          });
    }
  }

  sensesp::LambdaConsumer<double> tank_level_consumer_{[this](double value) {
    // Internal tank level is a ratio, NMEA 2000 wants a percentage.
    this->tank_level_.update(100. * value);
  }};

  String get_config_schema() override {
    String schema_template = R"###({
  "type": "object",
  "properties": {
    "tank_instance": {
      "title": "Tank instance",
      "type": "integer"
    },
    "tank_type": {
      "title": "Tank type",
      "type": "array",
      "uniqueItems": true,
      "format": "select",
      "items": {
        "type": "string",
        "enum": [
          $FLUID_TYPES$
        ]
      }
    },
    "tank_capacity": {
      "title": "Tank capacity (liters)",
      "type": "number"
    }
  }
})###";

    String fluid_types;
    for (int ii = 0; ii < sizeof(kN2kFluidLevelTankTypes) / 12; ii++) {
      fluid_types += "\"" + String(kN2kFluidLevelTankTypes[ii]) + "\"";
      if (ii < sizeof(kN2kFluidLevelTankTypes) / 12 - 1) {
        fluid_types += ",";
      }
    }
    schema_template.replace("$FLUID_TYPES$", fluid_types);
    return schema_template;
  };

  bool set_configuration(const JsonObject& config) override {
    const String expected[] = {"tank_instance", "tank_type", "tank_capacity"};
    for (const auto& str : expected) {
      if (!config.containsKey(str)) {
        debugE("N2kFluidLevelSender: Missing configuration key %s",
               str.c_str());
        return false;
      }
    }
    tank_instance_ = config["tank_instance"];
    const String& tank_type_str = config["tank_type"];
    for (int ii = 0; ii < sizeof(kN2kFluidLevelTankTypes) / 12; ii++) {
      if (tank_type_str == kN2kFluidLevelTankTypes[ii]) {
        tank_type_ = (tN2kFluidType)ii;
        break;
      }
    }
    tank_capacity_ = config["tank_capacity"];
    return true;
  }

  void get_configuration(JsonObject& config) override {
    config["tank_instance"] = tank_instance_;
    config["tank_type"] = kN2kFluidLevelTankTypes[tank_type_];
    config["tank_capacity"] = tank_capacity_;
  }

 protected:
  uint32_t repeat_interval_;
  uint32_t expiry_;
  tNMEA2000* nmea2000_;

  uint8_t tank_instance_;
  tN2kFluidType tank_type_;
  double tank_capacity_;                       // in liters
  sensesp::ExpiringValue<double> tank_level_;  // in percent
};

const char kN2kTemperatureSourceTypes[][35] = {
    "Sea Temperature",
    "Outside Temperature",
    "Inside Temperature",
    "Engine Room Temperature",
    "Main Cabin Temperature",
    "Live Well Temperature",
    "Bait Well Temperature",
    "Refrigeration Temperature",
    "Heating System Temperature",
    "Dew Point Temperature",
    "Apparent Wind Chill Temperature",
    "Theoretical Wind Chill Temperature",
    "Heat Index Temperature",
    "Freezer Temperature",
    "Exhaust Gas Temperature",
    "Shaft Seal Temperature"};

/**
 * @brief Transmit NMEA 2000 PGN 130316: Temperature, Extended Range
 *
 */
class N2kTemperatureExtSender : public N2kSender {
 public:
  N2kTemperatureExtSender(const String& config_path,
                          uint8_t temperature_instance,
                          tN2kTempSource temperature_source,
                          tNMEA2000* nmea2000, bool enable = true)
      : N2kSender{config_path},
        temperature_instance_{temperature_instance},
        temperature_source_{temperature_source},
        nmea2000_{nmea2000},
        repeat_interval_{2000},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{10000}           // In ms. When the inputs expire.
  {
    temperature_ =
        sensesp::ExpiringValue<double>(N2kDoubleNA, expiry_, N2kDoubleNA);
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_reaction_ == nullptr) {
      this->sender_reaction_ =
          reactesp::ReactESP::app->onRepeat(repeat_interval_, [this]() {
            tN2kMsg N2kMsg;
            // At the moment, the PGN is sent regardless of whether all
            // the values are invalid or not.
            SetN2kTemperatureExt(N2kMsg, 255, this->temperature_instance_,
                                 this->temperature_source_,
                                 this->temperature_.get(), N2kDoubleNA);
            this->nmea2000_->SendMsg(N2kMsg);
          });
    }
  }

  sensesp::LambdaConsumer<double> temperature_consumer_{
      [this](double value) { this->temperature_.update(value); }};

  String get_config_schema() override {
    String schema_template = R"###({
  "type": "object",
  "properties": {
    "temperature_instance": {
      "title": "Temperature instance",
      "type": "integer"
    },
    "temperature_source": {
      "title": "Temperature source",
      "type": "array",
      "uniqueItems": true,
      "format": "select",
      "items": {
        "type": "string",
        "enum": [
          $TEMP_TYPES$
        ]
      }
    }
  }
})###";

    String fluid_types;
    for (int ii = 0; ii < sizeof(kN2kTemperatureSourceTypes) / 35; ii++) {
      fluid_types += "\"" + String(kN2kTemperatureSourceTypes[ii]) + "\"";
      if (ii < sizeof(kN2kTemperatureSourceTypes) / 35 - 1) {
        fluid_types += ",";
      }
    }
    schema_template.replace("$TEMP_TYPES$", fluid_types);
    return schema_template;
  };

  bool set_configuration(const JsonObject& config) override {
    const String expected[] = {"temperature_instance", "temperature_source"};
    for (const auto& str : expected) {
      if (!config.containsKey(str)) {
        debugE("N2kTemperatureExtSender: Missing configuration key %s",
               str.c_str());
        return false;
      }
    }
    temperature_instance_ = config["temperature_instance"];
    const String& temperature_source_str = config["temperature_source"];
    for (int ii = 0; ii < sizeof(kN2kTemperatureSourceTypes) / 35; ii++) {
      if (temperature_source_str == kN2kTemperatureSourceTypes[ii]) {
        temperature_source_ = (tN2kTempSource)ii;
        break;
      }
    }
    return true;
  }

  void get_configuration(JsonObject& config) override {
    config["temperature_instance"] = temperature_instance_;
    config["temperature_source"] =
        kN2kTemperatureSourceTypes[temperature_source_];
  }

 protected:
  uint32_t repeat_interval_;
  uint32_t expiry_;
  tNMEA2000* nmea2000_;

  uint8_t temperature_instance_;
  tN2kTempSource temperature_source_;
  sensesp::ExpiringValue<double> temperature_;  // in percent
};

}  // namespace halmet

#endif

#endif  // HALMET_SRC_N2K_SENDERS_H_
