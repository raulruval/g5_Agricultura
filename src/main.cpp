#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"

const char *ssid = "g5Net";
const char *password = "g5IotNet";

#define DHTPIN D4
#define DHTTYPE DHT11
#define TOPIC "g5/sensor"
#define BROKER_IP "192.168.1.144"
#define BROKER_PORT 2883

DHT dht(DHTPIN, DHTTYPE);
#define LIGHTSENSORPIN A1

WiFiClient espClient;
PubSubClient client(espClient);
QueueHandle_t xMutex;

float temperatura;
float humedad;
float humedadSuelo;
float fotoreceptor;

void IRAM_ATTR on_handleInterrupt()
{
}

void IRAM_ATTR off_handleInterrupt()
{
}

/*
  Tarea para la lectura de la temperatura global y escogida y arrancado de termostato
*/
void tareaTemperatura(void *param)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      temperatura = dht.readTemperature();
      if (isnan(temperatura))
      {
        Serial.println(F("Error de lectura del sensor DHT!"));
        return;
      }

      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 1000);
    }
  }
  vTaskDelete(NULL);
}

/*
  Tarea para la lectura de la luz ambiente
*/
void tareaFotoreceptor(void *param)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      fotoreceptor = analogRead(LIGHTSENSORPIN);
      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 1000);
    }
  }
  vTaskDelete(NULL);
}

/*
  Tarea para la lectura de la humedad ambiente
*/
void tareaHumedad(void *param)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      humedad = dht.readHumidity();
      if (isnan(humedad))
      {
        Serial.println(F("Error de lectura del sensor DHT!"));
        return;
      }
      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 1000);
    }
  }
  vTaskDelete(NULL);
}

/*
  Tarea para la lectura de la humedad del suelo
*/
void tareaHumedadSuelo(void *param)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      humedadSuelo = 15.0;
      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 1000);
    }
  }
  vTaskDelete(NULL);
}

/*
  Tarea para activar el servo
*/
void tareaServo(void *param)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      if (temperatura > 30 || humedad < 30  ){
        printf("Accionar servo");
      }
      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 1000);
    }
  }
  vTaskDelete(NULL);
}

/*
  Tarea para activar el servo
*/
void tareaEnvio(void *param)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      String jsonData = "{\"temperatura\":" + String(temperatura) + ",\"luz\":" + String(fotoreceptor) + ",\"humedad\":" + String(humedad) + ",\"humedadSuelo\":" + String(humedadSuelo) + "}";
      client.publish(TOPIC, jsonData.c_str());
      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 1500);
    }
  }
  vTaskDelete(NULL);
}

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

  xMutex = xSemaphoreCreateMutex();
  if (xMutex != NULL)
  {
    xTaskCreatePinnedToCore(tareaTemperatura, "Tarea lectura temperatura", 1500, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(tareaFotoreceptor, "Tarea lectura luz", 1500, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(tareaHumedad, "Tarea lectura humedad", 1500, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(tareaHumedadSuelo, "Tarea lectura humedad del suelo", 1500, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(tareaServo, "Tarea accionador servo", 1500, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(tareaEnvio, "Tarea para enviar", 1500, NULL, 1, NULL, 0);
  }

}

void loop()
{
}