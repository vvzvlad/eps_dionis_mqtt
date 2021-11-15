#include "EspMQTTClient.h"
#include "Wire.h"
#include <ArduinoJson.h>

/*!<indirect memory table: https://wiki.analog.com/resources/tools-software/sigmastudio/usingsigmastudio/indirectparamaccess#writing_the_parameter) */
#define DSP_ADDRESS                   0x38 /*!< i2c slave address for DSP */
#define DSP_VOLUME_ADDRESS            24592 /*!< volume address in memory (indirect memory table) */
#define DSP_VOLUME_CHANNELS_ADDRESS   24586 /*!< volume channels address in memory (indirect memory table) */
#define DSP_START_ADDRESS             24583 /*!< start address in memory (indirect memory table) */
#define DSP_NUM_TRIGGER_ADDRESS       24584 /*!< num address in memory (indirect memory table) */

#define PERIODIC_MESSAGE_INTERNAL     30*1000 /*!< periodicity of messages in milliseconds */
#define MAX_DB_VALUE                  0 /*!< maximum value of dB */
#define MIN_DB_VALUE                  -50 /*!< minimum value of dB */

uint8_t mqtt_mutex = 0;
uint8_t global_volume_percent = 0;
unsigned long current_ms = 0;
unsigned long previous_ms = 0;

EspMQTTClient mqtt_client(
  "IoT_Dobbi",            //WiFi name
  "canned-ways-incense",  //WiFi password
  "192.168.88.111",       //MQTT server address
  "", "",                 //MQTT username and password
  "ESP8266_Dionis",       //Client name
  1883                    //MQTT port
);

//---------------------------------------------//

int16_t log_interpolate(int16_t in) {
    int16_t out = 0;
    if (in == 0) out = 0;
    else if (in > 0) out = 0.7*in;
    else if (in >= 81) out = 2.3*in-130;
    else if (in >= 100) out = 100;
    //printf("%d\t%d\n", in, out);
    return out;
}

inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline int16_t convert_percents_to_decibels(int16_t percents) {
  return map(percents, 0, 100, MIN_DB_VALUE, MAX_DB_VALUE);
}

inline float exp10f( float x ) { return powf( 10.f, x ); }
inline int32_t float_to_dsp(float x) { return (int32_t)( x * 0x1p24f ); }
inline int32_t dB_to_dsp(float x) { return float_to_dsp( exp10f( x / 20.f ) ); }


void dsp_write_value(uint16_t address, uint32_t value)
{
  uint8_t address_u8[] = { (address >> 8) & 0xFF, (address >> 0) & 0xFF };
  uint8_t data_u8[] = { (value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, (value >> 0) & 0xFF};
  //printf("I2C write 0x%02x %02x %02x %02x on 0x%02x %02x...\n", data_u8[0],data_u8[1],data_u8[2],data_u8[3], address_u8[0],address_u8[1]);
  Wire.beginTransmission(DSP_ADDRESS);
  Wire.write(address_u8[0]);
  Wire.write(address_u8[1]);
  Wire.write(data_u8[0]);
  Wire.write(data_u8[1]);
  Wire.write(data_u8[2]);
  Wire.write(data_u8[3]);
  Wire.endTransmission();
}

void set_volume(uint8_t volume_percent)
{
  int16_t volume_percent_log = log_interpolate((int16_t)volume_percent);
  int16_t db_volume = convert_percents_to_decibels(volume_percent_log);
  int32_t t824_volume = dB_to_dsp((float)db_volume);
  //printf("Setting volume: %d -> %d lg -> %d db,\n", volume_percent, volume_percent_log, db_volume);

  Serial.println("Start I2C write to DSP..");
  dsp_write_value(DSP_VOLUME_ADDRESS, t824_volume);
  dsp_write_value(DSP_START_ADDRESS, DSP_VOLUME_ADDRESS);
  dsp_write_value(DSP_NUM_TRIGGER_ADDRESS, 1);
}

void set_volume_channels(float a_ch_db, float b_ch_db, float c_ch_db, float d_ch_db, float ab_ch_db, float cd_ch_db)
{
  Serial.println("Start I2C write to DSP..");
  dsp_write_value(DSP_VOLUME_CHANNELS_ADDRESS+0, dB_to_dsp(a_ch_db));
  dsp_write_value(DSP_VOLUME_CHANNELS_ADDRESS+1, dB_to_dsp(b_ch_db));
  dsp_write_value(DSP_VOLUME_CHANNELS_ADDRESS+2, dB_to_dsp(c_ch_db));
  dsp_write_value(DSP_VOLUME_CHANNELS_ADDRESS+3, dB_to_dsp(d_ch_db));
  dsp_write_value(DSP_VOLUME_CHANNELS_ADDRESS+4, dB_to_dsp(ab_ch_db));
  dsp_write_value(DSP_VOLUME_CHANNELS_ADDRESS+5, dB_to_dsp(cd_ch_db));
  dsp_write_value(DSP_START_ADDRESS, DSP_VOLUME_CHANNELS_ADDRESS);
  dsp_write_value(DSP_NUM_TRIGGER_ADDRESS, 6);
  //Serial.println("End sending to 0x32");
}

void volume_message_received(const String& topic, const String& message) {
  global_volume_percent = strtol(message.c_str(), NULL, 10);
  if (global_volume_percent > 100) { global_volume_percent = 100; }
  if (global_volume_percent < 0) { global_volume_percent = 0; }
  //printf("Recieved payload from MQTT %s, parsed volume %d\n", message.c_str(), volume);
  if (mqtt_mutex == 0) {
    mqtt_mutex = 1;
    set_volume(global_volume_percent);
    mqtt_mutex = 0;
  }
  else{
    Serial.println("MQTT mutex is locked");
  }
}

void volume_channels_message_received(const String& topic, const String& message) {
  DynamicJsonDocument volume_channels(1024);
  DeserializationError error = deserializeJson(volume_channels, message); //{"A":"-12.50","B":"-7.5","C":"-3.75","D":"-15","AB":"-3.75","CD":"-16.25"}

  //printf("Recieved payload from MQTT %s\n Parsed volumes: %f,%f,%f,%f,%f,%f\n", message.c_str(), a_volume, b_volume, c_volume, d_volume, ab_volume, cd_volume);
  if (mqtt_mutex == 0) {
    mqtt_mutex = 1;
    set_volume_channels(volume_channels["A"].as<float>(),
                          volume_channels["B"].as<float>(),
                          volume_channels["C"].as<float>(),
                          volume_channels["D"].as<float>(),
                          volume_channels["AB"].as<float>(),
                          volume_channels["CD"].as<float>() );
    mqtt_mutex = 0;
  }
  else
  {
    Serial.println("MQTT mutex is locked");
  }
}

void onConnectionEstablished()
{
  mqtt_client.subscribe("Dionis/volume/set", volume_message_received);
  mqtt_client.subscribe("Dionis/volume_channels/set", volume_channels_message_received);

  mqtt_client.publish("Dionis/status", "MQTTtoI2C module started");
  mqtt_client.publish("Dionis/volume", String(global_volume_percent));
}

void i2c_scanner()
{
  Wire.begin();
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
        Serial.print("Found I2C Device: ");
        Serial.print(" (0x");
        Serial.print(i, HEX);
        Serial.println(")");
        delay(1);
      }
  }
  Serial.print("\r\n");
}

//---------------------------------------------//

void setup()
{
  Serial.begin(115200);
  mqtt_client.enableDebuggingMessages();
  i2c_scanner();
}


void loop()
{
  mqtt_client.loop();
  current_ms = millis();

  if (current_ms - previous_ms >= PERIODIC_MESSAGE_INTERNAL) {
    previous_ms = current_ms;
    Serial.println("Publishing to MQTT periodic message");
    mqtt_client.publish("Dionis/volume", String(global_volume_percent));
  }
}
