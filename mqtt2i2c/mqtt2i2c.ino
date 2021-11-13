#include "EspMQTTClient.h"
#include "Wire.h"

/*!<indirect memory table: https://wiki.analog.com/resources/tools-software/sigmastudio/usingsigmastudio/indirectparamaccess#writing_the_parameter) */
#define DSP_ADDRESS 0x38 /*!< slave address for DSP */
#define DSP_VOLUME_ADDRESS 0x6010 /*!< volume address in memory (indirect memory table) */
#define DSP_START_ADDRESS 0x6007 /*!< start address in memory (indirect memory table) */
#define DSP_NUM_ADDRESS 0x6008 /*!< num address in memory (indirect memory table) */
#define PERIDIC_MESSAGE_INTERNAL 30*1000 /*!< periodicity of internal messages in milliseconds */

uint8_t mqtt_mutex = 0;
uint8_t global_volume_percent = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

EspMQTTClient mqtt_client(
  "IoT_Dobbi",
  "canned-ways-incense",
  "192.168.88.111",
  "",
  "",
  "ESP8266_Dionis",
  1883
);

void setup()
{
  Serial.begin(115200);
  mqtt_client.enableDebuggingMessages();

  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
      {
      Serial.print("Found I2C Device: ");
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      delay(1);
      }
  }
  Serial.print("\r\n");
}

void write_value(uint16_t address, uint32_t value)
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

int16_t log_interpolate(int16_t in) {
    int16_t out = 0;
    if (in == 0) out = 0;
    if (in > 0) out = 0.7*in;
    if (in >= 81) out = 2.3*in-130;
    if (in >= 100) out = 100;
    //printf("%d\t%d\n", in, out);
    return out;
}

int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int16_t convert_percents_to_decibels(int16_t percents) {
  return map(percents, 0, 100, -50, 0);
}

float exp10f( float x ) {
    return powf( 10.f, x );
}

int32_t float_to_dsp(float x) {
    return (int32_t)( x * 0x1p24f );
}

int32_t dB_to_dsp(float x) {
    return float_to_dsp( exp10f( x / 20.f ) );
}


void set_volume(uint8_t volume_percent)
{

  int16_t volume_percent_log = log_interpolate((int16_t)volume_percent);
  int16_t db_volume = convert_percents_to_decibels(volume_percent_log);
  int32_t t824_volume = dB_to_dsp((float)db_volume);
  printf("Setting volume: %d -> %d lg -> %d db,\n", volume_percent, volume_percent_log, db_volume);

  //printf("Value %.2f, fixpoint %ld, hex %02X %02X %02X %02X\n", db_volume, t824_volume, bytes[0], bytes[1], bytes[2], bytes[3]);
  //printf("Send to 0x32 volume command %d, hex converted 0x%08x...\n", volume_percent, volume);

  Serial.println("Start sending to 0x32");
  write_value(DSP_VOLUME_ADDRESS, t824_volume);
  write_value(DSP_START_ADDRESS, DSP_VOLUME_ADDRESS);
  write_value(DSP_NUM_ADDRESS, 1);
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
  else
  {
    Serial.println("MQTT mutex is locked");
  }

}

void onConnectionEstablished()
{
  mqtt_client.subscribe("Dionis/volume/set", volume_message_received);
  mqtt_client.publish("Dionis/status", "Wifi module started");
  mqtt_client.publish("Dionis/volume", String(global_volume_percent));
}

void loop()
{
  mqtt_client.loop();
  currentMillis = millis();

  if (currentMillis - previousMillis >= PERIDIC_MESSAGE_INTERNAL) {
    previousMillis = currentMillis;
    Serial.println("Publishing to MQTT periodic message");
    mqtt_client.publish("Dionis/volume", String(global_volume_percent));
  }

}
