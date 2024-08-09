// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

// Uncomment this line to disable debugs output.
//#define SERIAL_DEBUG_DISABLED

// Comment out this line to disable NMEA 2000 output.
//#define ENABLE_NMEA2000_OUTPUT

// Comment out this line to disable Signal K support. At the moment, disabling
// Signal K support also disables all WiFi functionality.
// #define ENABLE_SIGNALK

#include "any_transform.h"
#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#ifdef ENABLE_NMEA2000_OUTPUT
#include "n2k_senders.h"
#endif

#include <Arduino.h>
#include <WString.h>
#include <WiFi.h>
#include <Wire.h>

#include <Adafruit_ADS1X15.h>
#include <Adafruit_SSD1306.h>

#ifdef ENABLE_NMEA2000_OUTPUT
#include <N2kTypes.h>
#include <NMEA2000.h>
#include <NMEA2000_esp32.h>
#endif

#include <ReactESP.h>
#ifdef ENABLE_SIGNALK
#include <sensesp_app.h>
#include <sensesp_app_builder.h>
#define BUILDER_CLASS sensesp::SensESPAppBuilder
#define APP_CLASS sensesp::SensESPApp
#else
#include <sensesp_minimal_app.h>
#include <sensesp_minimal_app_builder.h>
#define BUILDER_CLASS sensesp::SensESPMinimalAppBuilder
#define APP_CLASS sensesp::SensESPMinimalApp
#endif
#ifdef ENABLE_SIGNALK
#include <sensesp/signalk/signalk_output.h>
#else
#include <sensesp/net/discovery.h>
#include <sensesp/net/http_server.h>
#include <sensesp/net/networking.h>
#endif
#include <sensesp/sensors/analog_input.h>
#include <sensesp/sensors/digital_input.h>
#include <sensesp/sensors/sensor.h>
#include <sensesp/system/lambda_consumer.h>
#include <sensesp/system/local_debug.h>
#include <sensesp/system/system_status_led.h>
#include <sensesp/transforms/curveinterpolator.h>
#include <sensesp/transforms/lambda_transform.h>
#include <sensesp/transforms/linear.h>
#include <sensesp/transforms/time_counter.h>
#include <sensesp/ui/ui_controls.h>
#include <sensesp_onewire/onewire_temperature.h>

using namespace halmet;

namespace {

/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

/////////////////////////////////////////////////////////////////////
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
constexpr int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
constexpr int kTestOutputFrequency = 380;
#endif

// Default fuel tank size, in m3
constexpr float kTankDefaultSize = 120. / 1000;

constexpr char kFillLevelCurveDescription[] =
    "Piecewise linear conversion of the resistance to a "
    "fill level ratio between 0 and 1.</p>"
    "<p>Input values are resistances in ohms, outputs are the corresponding "
    "fill level ratios (between 0 and 1).";

/////////////////////////////////////////////////////////////////////
// Declare global app to keep state.

static reactesp::ReactESP app;

}  // namespace

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  sensesp::SetupLogging(ESP_LOG_WARN);
#endif

  // initialize the I2C bus
  auto* i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto* ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  const bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcSetup(0, kTestOutputFrequency, 13);
  // Attach the channel to the GPIO pin to be controlled
  ledcAttachPin(kTestOutputPin, 0);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(0, 4096);
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality
  tNMEA2000* nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20240801",    // Manufacturer's Model serial code (max 32 chars)
      104,           // Manufacturer's product code
      "HALMET",      // Manufacturer's Model ID (max 33 chars)
      "2024.08.01",  // Manufacturer's Software version code (max 40 chars)
      "20240801"     // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  //: http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
  //: https://manualzz.com/doc/12647142/nmea2000-class-and-function-codes

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046                     // Manufacturer code
  );

  const unsigned long ReceivedMessages[] PROGMEM = {
      126992L,  // System Time
      129033L,  // Local Time Offset
      0         // End of list
  };
  nmea2000->ExtendReceiveMessages(ReceivedMessages);

  // Initial N2k node address
  // NOTE: For certified NMEA 2000 devices it is mandatory save changed
  // address to e.g. EEPROM, for use in next startup.
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 71);
  nmea2000->EnableForward(false);
#ifndef SERIAL_DEBUG_DISABLED
#if 1  // NOTE: Used for debugging
  nmea2000->SetMsgHandler([](const tN2kMsg& N2kMs) { N2kMs.Print(&Serial); });
#endif
#endif
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will
  // do
  reactesp::ReactESP::app->onRepeat(
      1, [nmea2000]() { nmea2000->ParseMessages(); });
#endif

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;
  APP_CLASS* sensesp_app =
      (&builder)
          // EDIT: Set a custom hostname for the app.
          ->set_hostname("halmet")
          //->enable_ota("gishaaquav1O")
          // EDIT: Optionally, hard-code the WiFi and Signal K
          // server settings. This is normally not needed.
          //->set_wifi("My WiFi SSID", "my_wifi_password")
          //->set_sk_server("192.168.10.3", 80)
          ->get_app();

#ifndef ENABLE_SIGNALK
  // Initialize components that would normally be present in SensESPApp
  auto* networking = new sensesp::Networking(
      "/System/WiFi Settings",                  // config_path
      sensesp::SensESPBaseApp::get_hostname(),  // client_ssid
      "thisisfine");                            // client_password
  auto* mdns_discovery = new sensesp::MDNSDiscovery();
  auto* http_server = new sensesp::HTTPServer();
  auto* system_status_led = new sensesp::SystemStatusLed(LED_BUILTIN);
#endif

  // Initialize the OLED display
  Adafruit_SSD1306* display = nullptr;
  const bool display_present = InitializeSSD1306(
      &display, i2c, sensesp::SensESPBaseApp::get_hostname().c_str());

  /////////////////////////////////////////////////////////////////////
  // NMEA 2000 sender objects

  auto* enable_n2k_output = new sensesp::CheckboxConfig(
      false, "NMEA 2000 Output Enabled", "/NMEA 2000/NMEA 2000 Enabled");
  enable_n2k_output->set_description(
      "Enable NMEA 2000 output. If disabled, no NMEA 2000 "
      "messages will be sent, regardless of other settings.");
  enable_n2k_output->set_sort_order(500);

#ifdef ENABLE_NMEA2000_OUTPUT
  N2kEngineParameterDynamicSender* n2k_engine_dynamic_sender = nullptr;

  if (enable_n2k_output->get_value()) {
    // Create the NMEA 2000 sender objects when enabled

    n2k_engine_dynamic_sender = new N2kEngineParameterDynamicSender(
        "/NMEA 2000/Engine Dynamic", 0, nmea2000);
    n2k_engine_dynamic_sender->set_sort_order(520);
  }
#endif

  ///////////////////////////////////////////////////////////////////
  // Analog inputs

  // Analog input A1

  auto* enable_tank_volume =
      new sensesp::CheckboxConfig(true, "Enable A1 Input", "/Tank A1/Enabled");
  enable_tank_volume->set_description(
      "Enable analog tank level input A1. Requires a reboot to take "
      "effect.");
  enable_tank_volume->set_sort_order(1000);

  if (enable_tank_volume->get_value()) {
    // Connect the tank senders.
    auto* a1_tank_resistance = AnalogResistanceSender(ads1115, 0, "A1");
    // Resistance converted to relative value 0..1
    auto* tank_a1_level =
        new sensesp::CurveInterpolator(nullptr, "/Tank A1/Level Curve");
    tank_a1_level->set_input_title("Sender Resistance (ohms)")
        ->set_output_title("Fill Level (ratio)")
        ->set_description(kFillLevelCurveDescription)
        ->set_sort_order(1100);
    if (tank_a1_level->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      tank_a1_level->clear_samples();
      tank_a1_level->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
      tank_a1_level->add_sample(sensesp::CurveInterpolator::Sample(900., 0.5));
      tank_a1_level->add_sample(sensesp::CurveInterpolator::Sample(1800., 1));
    }
    a1_tank_resistance->connect_to(tank_a1_level);

    // Level converted to remaining volume in m3
    auto* tank_a1_volume =
        new sensesp::Linear(kTankDefaultSize, 0, "/Tank A1/Total Volume");
    tank_a1_volume->set_description("Total volume of tank A1 in m3");
    tank_a1_volume->set_sort_order(1200);
    tank_a1_level->connect_to(tank_a1_volume);

#ifdef ENABLE_SIGNALK
    auto* analog_a1_resistance_sk_output = new sensesp::SKOutputFloat(
        "tanks.fuel.A1.senderResistance", "/Tank A1/Sender Resistance",
        new sensesp::SKMetadata("ohm", "Input A1 sender resistance",
                                "Input A1 sender resistance"));
    analog_a1_resistance_sk_output->set_sort_order(1300);
    a1_tank_resistance->connect_to(analog_a1_resistance_sk_output);

    auto* tank_a1_level_sk_output = new sensesp::SKOutputFloat(
        "tanks.fuel.A1.currentLevel", "/Tank A1/Current Level",
        new sensesp::SKMetadata("ratio", "Tank A1 level", "Tank A1 level"));
    tank_a1_level_sk_output->set_sort_order(1400);
    tank_a1_level->connect_to(tank_a1_level_sk_output);

    auto* tank_a1_volume_sk_output = new sensesp::SKOutputFloat(
        "tanks.fuel.A1.currentVolume", "/Tank A1/Current Volume",
        new sensesp::SKMetadata("m3", "Tank A1 volume",
                                "Calculated tank A1 remaining volume"));
    tank_a1_volume_sk_output->set_sort_order(1500);
    tank_a1_volume->connect_to(tank_a1_volume_sk_output);
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
    if (enable_n2k_output->get_value()) {
      // Create the NMEA 2000 sender objects when enabled
      auto* n2k_a1_tank_level_output = new N2kFluidLevelSender(
          "/Tank A1/NMEA 2000", 0, N2kft_Fuel, 200, nmea2000);
      n2k_a1_tank_level_output->set_sort_order(1600);

      // Connect outputs to the N2k senders.
      tank_a1_volume->connect_to(
          &(n2k_a1_tank_level_output->tank_level_consumer_));
    }
#endif

#if 0
      tank_a1_level->attach([tank_a1_level]() {
        debugD("tank_a1_level: %f", tank_a1_level->get());
      });
      tank_a1_volume->attach([tank_a1_volume]() {
        debugD("tank_a1_volume: %f", tank_a1_volume->get());
      });
#endif

    if (display_present) {
      tank_a1_volume->connect_to(
          new sensesp::LambdaConsumer<float>([display](float value) {
            PrintValue(display, 2, "Tank A1", 100 * value);
          }));
    }
  }

  // Analog input A2

  auto* a2_input_enable =
      new sensesp::CheckboxConfig(false, "Enable A2 Input", "/Tank A2/Enabled");
  a2_input_enable->set_description(
      "Enable analog tank level input A2. Requires a reboot to take "
      "effect.");
  a2_input_enable->set_sort_order(2000);

  if (a2_input_enable->get_value()) {
    // Connect the tank senders.
    auto* analog_a2_resistance = AnalogResistanceSender(ads1115, 1, "A2");
    // Resistance converted to relative value 0..1
    auto* tank_a2_level =
        (new sensesp::CurveInterpolator(nullptr, "/Tank A2/Level Curve"))
            ->set_input_title("Sender Resistance (ohms)")
            ->set_output_title("Fill Level (ratio)");
    tank_a2_level->set_description(kFillLevelCurveDescription);
    tank_a2_level->set_sort_order(2100);
    if (tank_a2_level->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      tank_a2_level->clear_samples();
      tank_a2_level->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
      tank_a2_level->add_sample(sensesp::CurveInterpolator::Sample(900., 0.5));
      tank_a2_level->add_sample(sensesp::CurveInterpolator::Sample(1800., 1));
    }
    analog_a2_resistance->connect_to(tank_a2_level);
    // Level converted to remaining volume in m3
    auto* tank_a2_volume =
        new sensesp::Linear(kTankDefaultSize, 0, "/Tank A2/Total Volume");
    tank_a2_volume->set_description("Total volume of tank A2 in m3");
    tank_a2_volume->set_sort_order(2200);
    tank_a2_level->connect_to(tank_a2_volume);

#ifdef ENABLE_SIGNALK
    auto* analog_a2_resistance_sk_output = new sensesp::SKOutputFloat(
        "tanks.fuel.A2.senderResistance", "/Tank A2/Sender Resistance",
        new sensesp::SKMetadata("ohm", "Input A2 sender resistance",
                                "Input A2 sender resistance"));
    analog_a2_resistance_sk_output->set_sort_order(2300);
    analog_a2_resistance->connect_to(analog_a2_resistance_sk_output);
    auto* tank_a2_level_sk_output = new sensesp::SKOutputFloat(
        "tanks.fuel.A2.currentLevel", "/Tank A2/Current Level",
        new sensesp::SKMetadata("ratio", "Tank A2 level", "Tank A2 level"));
    tank_a2_level_sk_output->set_sort_order(2400);
    tank_a2_level->connect_to(tank_a2_level_sk_output);
    auto* tank_a2_volume_sk_output = new sensesp::SKOutputFloat(
        "tanks.fuel.A2.currentVolume", "/Tank A2/Current Volume",
        new sensesp::SKMetadata("m3", "Tank A2 volume",
                                "Calculated tank A2 remaining volume"));
    tank_a2_volume_sk_output->set_sort_order(2500);
    tank_a2_volume->connect_to(tank_a2_volume_sk_output);
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
    if (enable_n2k_output->get_value()) {
      // Create the NMEA 2000 sender objects when enabled
      auto* n2k_a2_tank_level_output = new N2kFluidLevelSender(
          "/Tank A2/NMEA 2000", 1, N2kft_Water, 200, nmea2000);
      n2k_a2_tank_level_output->set_sort_order(2600);

      // Connect outputs to the N2k senders.
      tank_a2_volume->connect_to(
          &(n2k_a2_tank_level_output->tank_level_consumer_));
    }
#endif
  }

  // Analog input A3

  auto* a3_input_enable =
      new sensesp::CheckboxConfig(false, "Enable A3 Input", "/Tank A3/Enabled");
  a3_input_enable->set_description(
      "Enable analog tank level input A3. Requires a reboot to take "
      "effect.");
  a3_input_enable->set_sort_order(3000);

  if (a3_input_enable->get_value()) {
    // Connect the tank senders.
    auto* analog_a3_resistance = AnalogResistanceSender(ads1115, 2, "A3");
    // Resistance converted to relative value 0..1
    auto* tank_a3_level =
        (new sensesp::CurveInterpolator(nullptr, "/Tank A3/Level Curve"))
            ->set_input_title("Sender Resistance (ohms)")
            ->set_output_title("Fill Level (ratio)");
    tank_a3_level->set_description(kFillLevelCurveDescription);
    tank_a3_level->set_sort_order(3100);
    if (tank_a3_level->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      tank_a3_level->clear_samples();
      tank_a3_level->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
      tank_a3_level->add_sample(sensesp::CurveInterpolator::Sample(900., 0.5));
      tank_a3_level->add_sample(sensesp::CurveInterpolator::Sample(1800., 1));
    }
    analog_a3_resistance->connect_to(tank_a3_level);
    // Level converted to remaining volume in m3
    auto* tank_a3_volume =
        new sensesp::Linear(kTankDefaultSize, 0, "/Tank A3/Total Volume");
    tank_a3_volume->set_description("Total volume of tank A3 in m3");
    tank_a3_volume->set_sort_order(3200);
    tank_a3_level->connect_to(tank_a3_volume);

#ifdef ENABLE_SIGNALK
    auto* analog_a3_resistance_sk_output = new sensesp::SKOutputFloat(
        "tanks.fuel.A3.senderResistance", "/Tank A3/Sender Resistance",
        new sensesp::SKMetadata("ohm", "Input A3 sender resistance",
                                "Input A3 sender resistance"));
    analog_a3_resistance_sk_output->set_sort_order(3300);
    analog_a3_resistance->connect_to(analog_a3_resistance_sk_output);
    auto* tank_a3_level_sk_output = new sensesp::SKOutputFloat(
        "tanks.fuel.A3.currentLevel", "/Tank A3/Current Level",
        new sensesp::SKMetadata("ratio", "Tank A3 level", "Tank A3 level"));
    tank_a3_level_sk_output->set_sort_order(3400);
    tank_a3_level->connect_to(tank_a3_level_sk_output);
    auto* tank_a3_volume_sk_output = new sensesp::SKOutputFloat(
        "tanks.fuel.A3.currentVolume", "/Tank A3/Current Volume",
        new sensesp::SKMetadata("m3", "Tank A3 volume",
                                "Calculated tank A3 remaining volume"));
    tank_a3_volume_sk_output->set_sort_order(3500);
    tank_a3_volume->connect_to(tank_a3_volume_sk_output);
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
    if (enable_n2k_output->get_value()) {
      // Create the NMEA 2000 sender objects when enabled
      auto* n2k_a3_tank_level_output = new N2kFluidLevelSender(
          "/Tank A3/NMEA 2000", 2, N2kft_GrayWater, 200, nmea2000);
      n2k_a3_tank_level_output->set_sort_order(3600);

      // Connect outputs to the N2k senders.
      tank_a3_volume->connect_to(
          &(n2k_a3_tank_level_output->tank_level_consumer_));
    }
#endif
  }

  // Analog input A4 (OIL PRESSURE)

  auto* a4_input_enable = new sensesp::CheckboxConfig(false, "Enable A4 Input",
                                                      "/Pressure A4/Enabled");
  a4_input_enable->set_description(
      "Enable analog pressure input A4. Requires a reboot to take effect.");
  a4_input_enable->set_sort_order(4000);

  if (a4_input_enable->get_value()) {
    // Connect the pressure sender.
    auto* a4_analog_resistance = AnalogResistanceSender(ads1115, 3, "A4");
    // Resistance converted to pressure in bar
    auto* a4_pressure_sender =
        (new sensesp::CurveInterpolator(nullptr, "/Pressure A4/Pressure"))
            ->set_input_title("Sender Resistance (ohms)")
            ->set_output_title("Pressure (Pa)");
    a4_pressure_sender->set_description(
        "Piecewise linear conversion of the resistance of the A4 sender "
        "to a "
        "pressure in Pascal. Input is resistance, output is pressure in "
        "Pascal.");
    a4_pressure_sender->set_sort_order(4100);
    if (a4_pressure_sender->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      a4_pressure_sender->clear_samples();
      a4_pressure_sender->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
      a4_pressure_sender->add_sample(
          sensesp::CurveInterpolator::Sample(900., 150000));
      a4_pressure_sender->add_sample(
          sensesp::CurveInterpolator::Sample(1800., 300000));
    }
    a4_analog_resistance->connect_to(a4_pressure_sender);

    const auto* low_pressure_limit =
        new sensesp::ParamInfo[1]{{"low_pressure_limit", "Low Pressure Limit"}};

    const auto alarm_pressure_low_comparator =
        [](float pressure, float limit) -> bool { return pressure < limit; };

    auto* sender_a4_low_pressure_alarm =
        new sensesp::LambdaTransform<float, bool, float>(
            alarm_pressure_low_comparator,
            100000,              // Default value for parameter
            low_pressure_limit,  // Parameter UI description
            "/Pressure A4/Low Pressure Alarm");
    sender_a4_low_pressure_alarm->set_description(
        "Alarm if the pressure falls below the set limit. Value in "
        "Pascal.");
    sender_a4_low_pressure_alarm->set_sort_order(4150);
    a4_pressure_sender->connect_to(sender_a4_low_pressure_alarm);

#ifdef ENABLE_SIGNALK
    auto* analog_a4_resistance_sk_output = new sensesp::SKOutputFloat(
        "propulsion.main.oilPressureSenderResistance",
        "/Pressure A4/Sender Resistance",
        new sensesp::SKMetadata("ohm", "Input A4 sender resistance",
                                "Input A4 sender resistance"));
    analog_a4_resistance_sk_output->set_sort_order(4200);
    a4_analog_resistance->connect_to(analog_a4_resistance_sk_output);

    auto* sender_a4_pressure_sk_output = new sensesp::SKOutputFloat(
        "propulsion.main.oilPressure", "/Pressure A4/Current Pressure",
        new sensesp::SKMetadata("Pa", "Oil Pressure",
                                "Main Engine Oil Pressure"));
    sender_a4_pressure_sk_output->set_sort_order(4300);
    a4_pressure_sender->connect_to(sender_a4_pressure_sk_output);
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
    if (n2k_engine_dynamic_sender) {
      // Connect the pressure output to N2k dynamic sender
      a4_pressure_sender->connect_to(
          &(n2k_engine_dynamic_sender->oil_pressure_consumer_));

      // Connect the low pressure alarm to N2k dynamic sender
      sender_a4_low_pressure_alarm->connect_to(
          &(n2k_engine_dynamic_sender->low_oil_pressure_consumer_));
    }
#endif
  }

  // Store alarm states in an array for local display output
  static bool alarm_states[4] = {false, false, false, false};

  ///////////////////////////////////////////////////////////////////
  // Digital input D1 (ac tachometer)

  auto* d1_rpm_output_enable = new sensesp::CheckboxConfig(
      true, "Enable RPM Output", "/Tacho D1/Enabled");
  d1_rpm_output_enable->set_description(
      "Enable RPM input D1. Requires a reboot to take effect.");
  d1_rpm_output_enable->set_sort_order(5000);

  if (d1_rpm_output_enable->get_value()) {
    // Connect the tacho senders. Engine name is "main".
    auto* d1_tacho_frequency =
        TachoDigitalSender(kDigitalInputPin1, "Tacho D1", "main", 5100);

#ifdef ENABLE_SIGNALK
    d1_tacho_frequency->connect_to(new sensesp::SKOutput<float>(
        "propulsion.main.revolutions", "",
        new sensesp::SKMetadata("Hz", "Main Engine Revolutions")));
#endif
    auto* engine_hours =
        new sensesp::TimeCounter<float>("/Tacho D1/Engine Hours");
    engine_hours->set_description(
        "Engine hours based on the D1 tacho input, in seconds.");
    engine_hours->set_sort_order(5400);
    d1_tacho_frequency->connect_to(engine_hours);

#ifdef ENABLE_SIGNALK
    // create and connect the engine hours output object
    engine_hours->connect_to(new sensesp::SKOutput<float>(
        "propulsion.main.runTime", "",
        new sensesp::SKMetadata("s", "Main Engine running time")));
#endif
    // create a propulsion state lambda transform
    auto* propulsion_state =
        new sensesp::LambdaTransform<float, String>([](bool freq) {
          if (freq > 0) {
            return "started";
          } else {
            return "stopped";
          }
        });

    // connect the tacho frequency to the propulsion state lambda transform
    d1_tacho_frequency->connect_to(propulsion_state);
#ifdef ENABLE_SIGNALK
    // create and connect the propulsion state output object
    propulsion_state->connect_to(new sensesp::SKOutput<String>(
        "propulsion.main.state", "",
        new sensesp::SKMetadata("", "Main Engine State")));
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
    if (enable_n2k_output->get_value()) {
      // Create the NMEA 2000 sender objects when enabled
      auto* n2k_engine_rapid_sender = new N2kEngineParameterRapidSender(
          "/NMEA 2000/Engine Rapid Update", 0, nmea2000);
      n2k_engine_rapid_sender->set_description(
          "PGN 127488 (Engine Rapid Update) parameters.");
      n2k_engine_rapid_sender->set_sort_order(510);

      // Connect outputs to the N2k senders.
      d1_tacho_frequency->connect_to(
          &(n2k_engine_rapid_sender->engine_speed_consumer_));
    }
#endif

    if (display_present) {
      d1_tacho_frequency->connect_to(
          new sensesp::LambdaConsumer<float>([display](float value) {
            PrintValue(display, 3, "RPM D1", 60 * value);
          }));
    }
  }

  ///////////////////////////////////////////////////////////////////
  // Digital input D2 (alarm low_oil_level)

  auto* d2_alarm_input = AlarmDigitalSender(kDigitalInputPin2, "D2", 2100);
  d2_alarm_input->connect_to(new sensesp::LambdaConsumer<bool>(
      [](bool value) { alarm_states[1] = value; }));

#ifdef ENABLE_NMEA2000_OUTPUT
  if (n2k_engine_dynamic_sender) {
    d2_alarm_input->connect_to(
        &(n2k_engine_dynamic_sender->low_oil_level_consumer_));
  }
#endif

  ///////////////////////////////////////////////////////////////////
  // Digital input D3 (alarm warning_level_1)

  auto* d3_alarm_input = AlarmDigitalSender(kDigitalInputPin3, "D3", 2200);
  // In this example, d3_alarm_input is active low, so invert the value.
  auto* d3_alarm_inverted =
      d3_alarm_input->connect_to(new sensesp::LambdaTransform<bool, bool>(
          [](bool value) { return !value; }));
  d3_alarm_inverted->connect_to(new sensesp::LambdaConsumer<bool>(
      [](bool value) { alarm_states[2] = value; }));

#ifdef ENABLE_NMEA2000_OUTPUT
  if (n2k_engine_dynamic_sender) {
    // NOTE: This is just an example -- normally temperature alarms would not be
    // active-low (inverted).
    d3_alarm_inverted->connect_to(
        &(n2k_engine_dynamic_sender->warning_level_1_consumer_));
  }
#endif

  ///////////////////////////////////////////////////////////////////
  // Digital input D4 (alarm warning_level_2)

  auto* d4_alarm_input = AlarmDigitalSender(kDigitalInputPin4, "D4", 2300);
  d4_alarm_input->connect_to(new sensesp::LambdaConsumer<bool>(
      [](bool value) { alarm_states[3] = value; }));

#ifdef ENABLE_NMEA2000_OUTPUT
  if (n2k_engine_dynamic_sender) {
    // NOTE: This is just an example -- normally alarms would not be
    // active-low (inverted).
    d4_alarm_input->connect_to(
        &(n2k_engine_dynamic_sender->warning_level_2_consumer_));
  }
#endif

  ///////////////////////////////////////////////////////////////////
  // 1-Wire Temperature Sensors

  auto* dts = new sensesp::DallasTemperatureSensors(kOneWirePin);

  // Any alarm of the 3 1-Wire temperature sensors
  auto* any_temperature_alarm = new sensesp::AnyTransform<3>();

  ///////////////////////////////////////////////////////////////////
  // 1-Wire temperature sensor 1 (Engine Oil Temperature)

#if 1  // OPTIONAL
  auto* main_engine_oil_temperature =
      new sensesp::OneWireTemperature(dts, 1000, "/Temperature 1/OneWire");
  main_engine_oil_temperature->set_description(
      "Engine oil temperature sensor on the 1-Wire bus.");
  main_engine_oil_temperature->set_sort_order(6000);

#ifdef ENABLE_SIGNALK
  // connect the sensors to Signal K output paths
  auto* main_engine_oil_temperature_metadata =
      new sensesp::SKMetadata("K",                       // units
                              "Engine Oil Temperature",  // display name
                              "Engine Oil Temperature",  // description
                              "Oil Temperature",         // short name
                              10.                        // timeout, in seconds
      );
  auto* oil_temp_sk_output = new sensesp::SKOutput<float>(
      "propulsion.main.oilTemperature", "/Temperature 1/SK Path",
      main_engine_oil_temperature_metadata);
  oil_temp_sk_output->set_sort_order(6100);
  main_engine_oil_temperature->connect_to(oil_temp_sk_output);
#endif

  const auto* oil_temperature_limit = new sensesp::ParamInfo[1]{
      {"oil_temperature_limit", "Oil Temperature Limit"}};

  const auto alarm_temp_high_comparator = [](float temperature,
                                             float limit) -> bool {
    return temperature > limit;
  };

  constexpr float kInitialOilTemperatureAlarm = 383;

  auto* sender_oil_temp_alarm =
      new sensesp::LambdaTransform<float, bool, float>(
          alarm_temp_high_comparator, kInitialOilTemperatureAlarm,
          oil_temperature_limit, "/Temperature 1/Oil Temperature Alarm");
  sender_oil_temp_alarm->set_description(
      "Alarm if the oil temperature exceeds the set limit. Value in Kelvin.");
  sender_oil_temp_alarm->set_sort_order(6200);

  main_engine_oil_temperature->connect_to(sender_oil_temp_alarm);

  sender_oil_temp_alarm->connect_to(any_temperature_alarm);

#ifdef ENABLE_NMEA2000_OUTPUT
  if (n2k_engine_dynamic_sender) {
    // Connect the oil temperature output to N2k dynamic sender
    main_engine_oil_temperature->connect_to(
        &(n2k_engine_dynamic_sender->oil_temperature_consumer_));
  }
#endif
#endif

  ///////////////////////////////////////////////////////////////////
  // 1-Wire temperature sensor 2 (Engine Coolant Temperature)

#if 0  // OPTIONAL
  auto* main_engine_coolant_temperature =
      new sensesp::OneWireTemperature(dts, 1000, "/Temperature 2/OneWire");
  main_engine_coolant_temperature->set_description(
      "Engine coolant temperature sensor on the 1-Wire bus.");
  main_engine_coolant_temperature->set_sort_order(7000);

#ifdef ENABLE_SIGNALK
  auto* main_engine_coolant_temperature_metadata =
      new sensesp::SKMetadata("K",                           // units
                              "Engine Coolant Temperature",  // display name
                              "Engine Coolant Temperature",  // description
                              "Coolant Temperature",         // short name
                              10.  // timeout, in seconds
      );
  auto* main_engine_coolant_temperature_sk_output =
      new sensesp::SKOutput<float>("propulsion.main.coolantTemperature",
                                   "/Temperature 2/Coolant Temperature SK Path",
                                   main_engine_coolant_temperature_metadata);
  main_engine_coolant_temperature_sk_output->set_sort_order(7100);
  main_engine_coolant_temperature->connect_to(
      main_engine_coolant_temperature_sk_output);

  auto* main_engine_temperature_metadata =
      new sensesp::SKMetadata("K",                   // units
                              "Engine Temperature",  // display name
                              "Engine Temperature",  // description
                              "Temperature",         // short name
                              10.                    // timeout, in seconds
      );
  auto* main_engine_temperature_sk_output = new sensesp::SKOutput<float>(
      "propulsion.main.temperature", "/Temperature 2/Temperature SK Path",
      main_engine_temperature_metadata);
  main_engine_temperature_sk_output->set_sort_order(7200);
  // transmit coolant temperature as overall engine temperature as well
  main_engine_coolant_temperature->connect_to(
      main_engine_temperature_sk_output);
#endif

  const auto* coolant_temperature_limit = new sensesp::ParamInfo[1]{
      {"coolant_temperature_limit", "Coolant Temperature Limit"}};

  const auto alarm_coolant_temp_high_comparator = [](float temperature,
                                              float limit) -> bool {
    return temperature > limit;
  };

  auto* sender_coolant_temp_alarm =
      new sensesp::LambdaTransform<float, bool, float>(
          alarm_coolant_temp_high_comparator,
          373,                        // Default value for parameter
          coolant_temperature_limit,  // Parameter UI description
          "/Temperature 2/Coolant Temperature Alarm");
  sender_coolant_temp_alarm->set_description(
      "Alarm if the coolant temperature exceeds the set limit. Value in "
      "Kelvin.");
  sender_coolant_temp_alarm->set_sort_order(6200);

  main_engine_coolant_temperature->connect_to(sender_coolant_temp_alarm);

  sender_coolant_temp_alarm->connect_to(any_temperature_alarm);

#ifdef ENABLE_NMEA2000_OUTPUT
  if (n2k_engine_dynamic_sender) {
    // Connect the coolant temperature output to N2k dynamic sender
    main_engine_coolant_temperature->connect_to(
        &(n2k_engine_dynamic_sender->temperature_consumer_));
  }
#endif
#endif

  ///////////////////////////////////////////////////////////////////
  // 1-Wire temperature sensor 3 (Wet Exhaust Temperature)

#if 0  // OPTIONAL
  auto* main_engine_exhaust_temperature =
      new sensesp::OneWireTemperature(dts, 1000, "/Temperature 3/OneWire");
  main_engine_exhaust_temperature->set_sort_order(8000);
  main_engine_exhaust_temperature->set_description(
      "Engine wet exhaust temperature sensor on the 1-Wire bus.");
  main_engine_exhaust_temperature->set_sort_order(8100);

#ifdef ENABLE_SIGNALK
  auto* main_engine_exhaust_temperature_metadata =
      new sensesp::SKMetadata("K",                        // units
                              "Wet Exhaust Temperature",  // display name
                              "Wet Exhaust Temperature",  // description
                              "Exhaust Temperature",      // short name
                              10.                         // timeout, in seconds
      );
  auto* main_engine_exhaust_temperature_sk_path = new sensesp::SKOutput<float>(
      "propulsion.main.wetExhaustTemperature", "/Temperature 3/SK Path",
      main_engine_exhaust_temperature_metadata);
  main_engine_exhaust_temperature_sk_path->set_sort_order(8200);
  // propulsion.*.wetExhaustTemperature is a non-standard path
  main_engine_exhaust_temperature->connect_to(
      main_engine_exhaust_temperature_sk_path);
#endif

  const auto* exhaust_temperature_limit = new sensesp::ParamInfo[1]{
      {"exhaust_temperature_limit", "Exhaust Temperature Limit"}};

  const auto alarm_exhaust_temp_high_comparator = [](float temperature,
                                              float limit) -> bool {
    return temperature > limit;
  };

  auto* sender_exhaust_temp_alarm =
      new sensesp::LambdaTransform<float, bool, float>(
          alarm_exhaust_temp_high_comparator,
          333,                        // Default value for parameter
          exhaust_temperature_limit,  // Parameter UI description
          "/Temperature 3/Coolant Temperature Alarm");
  sender_exhaust_temp_alarm->set_description(
      "Alarm if the coolant temperature exceeds the set limit. Value in "
      "Kelvin.");
  sender_exhaust_temp_alarm->set_sort_order(8300);

  main_engine_exhaust_temperature->connect_to(sender_exhaust_temp_alarm);

  sender_exhaust_temp_alarm->connect_to(any_temperature_alarm);

#ifdef ENABLE_NMEA2000_OUTPUT
  if (enable_n2k_output->get_value()) {
    // Create the NMEA 2000 sender objects when enabled
    auto* n2k_exhaust_temp_sender = new N2kTemperatureExtSender(
        "/Temperature 3/NMEA 2000", 0, N2kts_ExhaustGasTemperature, nmea2000);
    n2k_exhaust_temp_sender->set_sort_order(8400);

    // Connect the coolant temperature output to N2k dynamic sender
    main_engine_exhaust_temperature->connect_to(
        &(n2k_exhaust_temp_sender->temperature_consumer_));
  }
#endif
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
  if (n2k_engine_dynamic_sender) {
    // Connect the any temperature alarm to N2k dynamic sender
    any_temperature_alarm->connect_to(
        &(n2k_engine_dynamic_sender->over_temperature_consumer_));
  }
#endif

  ///////////////////////////////////////////////////////////////////
  // Display setup

  // Connect the outputs to the display
  if (display_present) {
    reactesp::ReactESP::app->onRepeat(1000, [display]() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    // Create a poor man's "christmas tree" display for the alarms
    reactesp::ReactESP::app->onRepeat(1000, [display]() {
      constexpr auto alarm_states_sz =
          sizeof(alarm_states) / sizeof(alarm_states[0]);
      char state_string[alarm_states_sz + 1];
      for (int ii = 0; ii < alarm_states_sz; ii++) {
        state_string[ii] = alarm_states[ii] ? '*' : '_';
      }
      state_string[alarm_states_sz] = '\0';
      PrintValue(display, 4, "Alarm", state_string);
    });
  }
}

void loop() { app.tick(); }
