#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>

const char *ssid = "g5Net";
const char *password = "g5IotNet";

#define DHTPIN D4
#define DHTTYPE DHT11
#define TOPIC "g5/sensor"
#define BROKER_IP "192.168.43.14"
#define BROKER_PORT 2883

DHT dht(DHTPIN, DHTTYPE);

WiFiClient espClient;
PubSubClient client(espClient);

void wifiConnect()
{
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connected to the WiFi network");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
}

void mqttConnect()
{
  client.setServer(BROKER_IP, BROKER_PORT);
  while (!client.connected())
  {
    Serial.print("MQTT connecting ...");

    if (client.connect("ESP32Client1"))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");

      delay(5000); //* Wait 5 seconds before retrying
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  delay(4000);
  wifiConnect();
  mqttConnect();

  //client.subscribe(TOPIC);
  //client.setCallback(mqttCallback);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // put your main code here, to run repeatedly:
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  String jsonData = "{\"temperature\":" + String(t) + ",\"humidity\":" + String(h) + "}";

  client.publish(TOPIC, jsonData.c_str());
  delay(3000);
}