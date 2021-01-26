#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "driver/mcpwm.h"
#include <nodeRed.pb.h>
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>

const char *ssid = "g5Net2";
const char *password = "g5IotNet";

#define DHTPIN D4
#define DHTTYPE DHT11
#define TOPIC "g5/sensor"
#define TOPICNORERED "g5/nodered"
#define BROKER_IP "192.168.1.144"
#define BROKER_PORT 2883
#define LIGHTSENSORPIN A1
#define SENSORSUELO A3

DHT dht(DHTPIN, DHTTYPE);

WiFiClient espClient;
PubSubClient client(espClient);
QueueHandle_t xMutex;
nodeRed message = nodeRed_init_zero;

float temperatura = 0.0;
float humedad = 0.0;
float humedadSuelo = 0.0;
float fotoreceptor = 0.0;

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
      float fotoreceptor = analogRead(LIGHTSENSORPIN);
      if (isnan(fotoreceptor))
      {
        Serial.println(F("Error de lectura del sensor Light A1! (Luz)"));
        return;
      }
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
      //El sensor registra valores normales de humedad, no es necesaria conversión. Valor registrado = 33~34
      //humedadSuelo = SENSORSUELO; Si se introduce con analogRead, obtenemos el valor en millares y decimales. Se puede utilizar si queremos se más precisos
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
      if (temperatura > 30 || humedad < 30)
      {
        printf("Accionar servo");
        /* Movimiento del servo cada 2 segundos en diferentes ángulos
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2.5);
        //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 500);  //0.5ms - 0
        delay(2000);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 7.5);
        //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500); //1.5ms - 90
        delay(2000);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 12.5);
        //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2500); //2.5ms - 180
        delay(2000);*/
      }
      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 1000);
    }
  }
  vTaskDelete(NULL);
}

/*
  Tarea para Enviar la información a la arquitectura
*/
void tareaEnvio(void *param)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      String jsonData = "{\"temperatura\":" + String(temperatura) + ",\"fotoreceptor\":" + String(fotoreceptor) + ",\"humedad\":" + String(humedad) + ",\"humedadSuelo\":" + String(humedadSuelo) + "}";
      client.publish(TOPIC, jsonData.c_str());

      // client.publish(TOPICNORERED, jsonData.c_str());
      message.temperatura = temperatura;
      message.humedad = humedad;
      message.humedadSuelo = humedadSuelo;
      message.fotoreceptor = fotoreceptor;

      uint8_t buffer[200];
      pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
      bool status = pb_encode(&stream, nodeRed_fields, &message);
      if (!status)
        Serial.println("Error encode");
      client.publish(TOPIC, buffer, stream.bytes_written);
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
  pinMode(LIGHTSENSORPIN, INPUT);
  Serial.begin(115200);
  dht.begin();
  wifiConnect();
  mqttConnect();

  //Inicialización del servo
  /**mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_14);

  mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);*/

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