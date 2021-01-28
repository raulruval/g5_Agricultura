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
#include "map"

const char *ssid = "g5Net3";
const char *password = "g5IotNet";

#define DHTPIN D4
#define DHTTYPE DHT11
#define TOPIC "g5/sensor"
#define TOPICNODERED "g5/nodered"
#define BROKER_IP "192.168.140.178"
#define BROKER_PORT 2883
#define LIGHTSENSORPIN A1
#define SENSORSUELO A3
#define BTN_MOTOR D2

DHT dht(DHTPIN, DHTTYPE);
std::map<String, float> Map;

WiFiClient espClient;
PubSubClient client(espClient);
xQueueHandle queue;

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
  Tarea para activar el servo
*/
void servo(bool activate)
{
  if (activate)
  {
    Serial.println("Accionar servo");
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
  else
  {
    Serial.println("Parar servo");
  }
}

void IRAM_ATTR motor_interrupcion()
{
  if (digitalRead(BTN_MOTOR) == 0)
  {
    servo(true);
  }
}

/*
  Tarea para la lectura de la temperatura global y escogida y arrancado de termostato
*/
void tareaTemperatura(void *param)
{
  for (;;)
  {
    Map["t"] = dht.readTemperature();
    xQueueSend(queue, (void *)&Map, (TickType_t)0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
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
    Map["f"] = analogRead(LIGHTSENSORPIN);
    xQueueSend(queue, (void *)&Map, (TickType_t)0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
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
    Map["h"] = dht.readHumidity();
    xQueueSend(queue, (void *)&Map, (TickType_t)0);
    vTaskDelay(6000 / portTICK_PERIOD_MS);
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
    Map["hs"] = 15.0;
    //El sensor registra valores normales de humedad, no es necesaria conversión. Valor registrado = 33~34
    //humedadSuelo = SENSORSUELO; Si se introduce con analogRead, obtenemos el valor en millares y decimales. Se puede utilizar si queremos se más precisos
    xQueueSend(queue, (void *)&Map, (TickType_t)0);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
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
    xQueueReceive(queue, &Map, (TickType_t)(1000 / portTICK_PERIOD_MS));
    String jsonData = "{\"temperatura\":" + String(Map["t"]) + ",\"fotoreceptor\":" + String(Map["f"]) + ",\"humedad\":" + String(Map["h"]) + ",\"humedadSuelo\":" + String(Map["hs"]) + "}";
    Serial.printf("\nEnviando información...");
    
    client.publish(TOPIC, jsonData.c_str());

    // client.publish(TOPICNORERED, jsonData.c_str());
    nodeRed message = nodeRed_init_zero;
    message.temperatura = Map["t"];
    message.humedad = Map["h"];
    message.humedadSuelo = Map["hs"];
    message.fotoreceptor = Map["f"];
    message.has_temperatura = true;
    message.has_humedad = true;
    message.has_humedadSuelo = true;
    message.has_fotoreceptor = true;

    uint8_t buffer[200];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    bool status = pb_encode(&stream, nodeRed_fields, &message);
    if (!status)
      Serial.println("Error encode");
    client.publish(TOPICNODERED, buffer, stream.bytes_written);
    vTaskDelay(7000 / portTICK_PERIOD_MS);
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
      delay(5000);
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(LIGHTSENSORPIN, INPUT);
  pinMode(BTN_MOTOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_MOTOR), &motor_interrupcion, FALLING);
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

  queue = xQueueCreate(10, sizeof(Map));
  if (queue != NULL)
  {
    xTaskCreate(tareaTemperatura, "Tarea lectura temperatura", CONFIG_SYSTEM_EVENT_TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(tareaFotoreceptor, "Tarea lectura luz", CONFIG_SYSTEM_EVENT_TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(tareaHumedad, "Tarea lectura humedad", CONFIG_SYSTEM_EVENT_TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(tareaHumedadSuelo, "Tarea lectura humedad del suelo", CONFIG_SYSTEM_EVENT_TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(tareaEnvio, "Tarea para enviar", CONFIG_SYSTEM_EVENT_TASK_STACK_SIZE, NULL, 5, NULL);
  }
}

void loop()
{
}