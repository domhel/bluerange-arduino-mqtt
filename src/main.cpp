#include <NeoPixelBus.h>

const uint16_t PixelCount = 50; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 2;     // make sure to set this to the correct pin, ignored for Esp8266

#define colorSaturation 128

NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> strip(PixelCount, PixelPin);

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

#define MQTT_MAX_PACKET_SIZE 1024

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#define BLUERANGE
//#define MQTT_DASHBOARD


const int mqtt_port = 8883;

#ifdef BLUERANGE
#elif defined(MQTT_DASHBOARD)
const char *mqtt_server = "mqtt-dashboard.com";
const char *mqtt_topic = "testtopic/#";
#else
const char *mqtt_server = "";
const char *mqtt_user = "";
const char *mqtt_password = "";
const char *mqtt_topic = "";
#endif

WiFiClientSecure esp_client;
PubSubClient mqtt_client(esp_client);

void setup_wifi()
{
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


void setup_mqtt()
{
  // Set the MQTT server and SSL/TLS options
  mqtt_client.setServer(mqtt_server, mqtt_port);

  // Set the callback function for incoming messages
  mqtt_client.setCallback(mqtt_callback);

  // Ensure a secure connection
  esp_client.setInsecure(); // This line ensures a secure connection without certificate validation
}

void reconnect_mqtt()
{
  while (!mqtt_client.connected())
  {
    Serial.print("Attempting MQTT connection...");

#ifdef BLUERANGE
    if (mqtt_client.connect("ESP8266Client", mqtt_user, mqtt_password))
#elif defined(MQTT_DASHBOARD)
    if (client.connect("ESP8266ClientABC"))
#endif
    {
      Serial.println("connected");
      const bool res = mqtt_client.subscribe(mqtt_topic);
      Serial.printf("Subscribed to %s: %s\n", mqtt_topic, res ? "OK" : "Failed");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  setup_leds();
  setup_wifi();
  setup_mqtt();
}

void loop()
{

  if (!mqtt_client.connected())
  {
    reconnect_mqtt();
  }
  mqtt_client.loop();
}

void setup_leds()
{
  strip.Begin();
  strip.Show();
}

void on_motion_changed(uint32_t motion_value) {
  if (motion_value) {
    set_all_pixels(blue);
  } else {
    set_all_pixels(black);
  }
}

void set_all_pixels(RgbColor& color) {
  for (uint32_t i = 0; i < PixelCount; i++)
  {
    strip.SetPixelColor(i, color);
  }
  strip.Show();
}
