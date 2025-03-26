//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 01 ESP32 Cam QR Code Scanner
/*
 * Reference :
 * - ESP32-CAM QR Code Reader (off-line) : https://www.youtube.com/watch?v=ULZL37YqJc8
 * - ESP32QRCodeReader_Page : https://github.com/fustyles/Arduino/tree/master/ESP32-CAM_QRCode_Recognition/ESP32QRCodeReader_Page
 * 
 * The source of the "quirc" library I shared on this project: https://github.com/fustyles/Arduino/tree/master/ESP32-CAM_QRCode_Recognition/ESP32QRCodeReader_Page
 */

/* ======================================== Including the libraries. */
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "quirc.h"
#include "ID_tag.h"   
#include <PubSubClient.h>  
#include <WiFi.h>
#include <Arduino.h>
#include <cstring>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L0X.h"
/* ======================================== */

// creating a task handle
TaskHandle_t QRCodeReader_Task; 
TaskHandle_t reconnect_Task; 
TaskHandle_t envio_Task; 
TaskHandle_t flash_Task;
/* ======================================== GPIO of camera model Al Thinker ESP32-CAM */

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


/* ======================================== */ // Definindo Pinos do LED do flash e do Buzzer
#define LED_OnBoard 4
#define LED_err 12
#define Buzzer 14
#define SDA_PIN 15 // SDA Connected to GPIO 14
#define SCL_PIN 13 // SCL Connected to GPIO 15

/* ======================================== Replace with your network credentials */
const char* ssid = "Automacao";
const char* password = "127.0.0.1...";
/* ======================================== */

/*======================================== Definição do endereço IP do servidor MQTT*/
const char* mqtt_server = "172.16.9.148";

WiFiClient espClient; // Cliente Wi-Fi para comunicação com o broker MQTT
PubSubClient client(espClient); // Cliente MQTT usando o cliente Wi-Fi
long lastMsg = 0; //Variável para controlar o tempo entre as mensagens publicadas
char msg[50];  // Buffer para armazenar a mensagem
char msg_ant[50];// Buffer para mensagem anterior
/* ======================================== */

/* ======================================== Variables declaration */
struct QRCodeData
{
  bool valid;
  int dataType;
  uint8_t payload[1024];
  int payloadLen;
};

struct quirc *q = NULL;
uint8_t *image = NULL;  
camera_fb_t * fb = NULL;
struct quirc_code code;
struct quirc_data data;
quirc_decode_error_t err;
struct QRCodeData qrCodeData;

/* ======================================== */

// VL53L0X (Using I2C) - SENSOR
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool estadoanterior;
/* ________________________________________________________________________________ VOID SETTUP() */
void setup() {
  // put your setup code here, to run once:
  pinMode(LED_OnBoard, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(LED_err, OUTPUT);
  digitalWrite(LED_err, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);

  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
 

  /* ---------------------------------------- Init serial communication speed (baud rate). */
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  /* ---------------------------------------- */

  /* ---------------------------------------- */
  client.setServer(mqtt_server, 1883);   // Configura o broker MQTT (porta 1883)
  //client.setCallback(callback);           // Define a função de callback para tratar mensagens recebidas
  /* ---------------------------------------- Camera configuration. */
  Serial.println("Start configuring and initializing the camera...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
  
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  
  Serial.println("Configure and initialize the camera successfully.");
  Serial.println();
  /* ---------------------------------------- */
  /* ---------------------------------------- Connect to Wi-Fi. */
  WiFi.mode(WIFI_STA);
  Serial.println("------------");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int connecting_process_timed_out = 20; //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(LED_OnBoard, HIGH);
    vTaskDelay(250);
    digitalWrite(LED_OnBoard, LOW);
    vTaskDelay(250);
    if(connecting_process_timed_out > 0) connecting_process_timed_out--;
    if(connecting_process_timed_out == 0) {
      vTaskDelay(1000);
      ESP.restart();
    }
  }

  /* ---------------------------------------- Inicializa a comunicação I2C do Sensor*/
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
  /* ---------------------------------------- create "QRCodeReader_Task" using the xTaskCreatePinnedToCore() function */
  xTaskCreatePinnedToCore(
             QRCodeReader,          /* Task function. */
             "QRCodeReader_Task",   /* name of task. */
             10000,                 /* Stack size of task */
             NULL,                  /* parameter of the task */
             5,                     /* priority of the task */
             &QRCodeReader_Task,    /* Task handle to keep track of created task */
             0);                    /* pin task to core 0 */
  /* ---------------------------------------- */
  xTaskCreatePinnedToCore(
             reconnect_mqtt,          /* Task function. */
             "reconnect_mqtt",   /* name of task. */
             2048,                 /* Stack size of task */
             NULL,                  /* parameter of the task */
             2,                     /* priority of the task */
             &reconnect_Task,    /* Task handle to keep track of created task */
             1);                    /* pin task to core 1 */
  /* ---------------------------------------- */
    /* ---------------------------------------- */

  xTaskCreatePinnedToCore(
             Flash,          /* Task function. */
             "Flash",   /* name of task. */
             2048,                 /* Stack size of task */
             NULL,                  /* parameter of the task */
             2,                     /* priority of the task */
             &flash_Task,    /* Task handle to keep track of created task */
             0);                    /* pin task to core 1 */
  /* ---------------------------------------- */ 

}

/* ________________________________________________________________________________ */
void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(10);
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ The function to be executed by "QRCodeReader_Task" */
// This function is to instruct the camera to take or capture a QR Code image, then it is processed and translated into text.
void QRCodeReader( void * pvParameters ){
  /* ---------------------------------------- */
  Serial.println("QRCodeReader is ready.");
  Serial.print("QRCodeReader running on core ");
  Serial.println(xPortGetCoreID());
  Serial.println();
  /* ---------------------------------------- */

  /* ---------------------------------------- Loop to read QR Code in real time. */
  while(1){
      q = quirc_new();
      if (q == NULL){
        Serial.print("can't create quirc object\r\n");  
        continue;
      }
    
      fb = esp_camera_fb_get();
      if (!fb)
      {
        Serial.println("Camera capture failed");
        continue;
      }   
      
      quirc_resize(q, fb->width, fb->height);
      image = quirc_begin(q, NULL, NULL);
      memcpy(image, fb->buf, fb->len);
      quirc_end(q);
      
      int count = quirc_count(q);
      if (count > 0) {
        quirc_extract(q, 0, &code);
        err = quirc_decode(&code, &data);
    
        if (err){
          Serial.println("Decoding FAILED");
          //digitalWrite(LED_OnBoard, HIGH);
        } else {
          Serial.printf("Decoding successful:\n");
          //digitalWrite(LED_OnBoard, LOW);
          dumpData(&data);
        } 
        Serial.println();
      } 
      
      esp_camera_fb_return(fb);
      fb = NULL;
      image = NULL;  
      quirc_destroy(q);
  }
  /* ---------------------------------------- */
}

//UTILIZADA APENAS PARA RECEBER MENSAGENS
/* ________________________________________________________________________________ 
// Função de callback que é chamada sempre que uma nova mensagem chega a um tópico inscrito
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");  // Imprime o tópico
  Serial.print(topic);
  Serial.print(". Message: ");
  
  String messageTemp = "";  // String temporária para armazenar a mensagem recebida

  // Imprime e armazena os dados da mensagem
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);  // Imprime cada byte da mensagem
    messageTemp += (char)message[i]; // Adiciona o byte à string messageTemp
  }
  Serial.println();  // Nova linha após imprimir a mensagem
}
*/

/* ________________________________________________________________________________ Function to display the results of reading the QR Code on the serial monitor. */
void dumpData(const struct quirc_data *data)
{
  Serial.printf("Version: %d\n", data->version);
  Serial.printf("ECC level: %c\n", "MLHQ"[data->ecc_level]);
  Serial.printf("Mask: %d\n", data->mask);
  Serial.printf("Length: %d\n", data->payload_len);
  Serial.printf("Payload: %s\n", data->payload);
  
  sprintf(msg, "%s", data->payload);

  if(memcmp(msg, msg_ant, sizeof(msg)) != 0){
    //Sinal luminoso e sonoro
    sinal_envio();
    //Cria task de envio
    xTaskCreatePinnedToCore(
            enviar_mensagem_mqtt,          /* Task function. */
            "enviar_mensagem_mqtt",   /* name of task. */
            10000,                 /* Stack size of task */
            msg,                  /* parameter of the task */
            1,                     /* priority of the task */
            &envio_Task,    /* Task handle to keep track of created task */
            1);                    /* pin task to core 1 */
  }
}
/* ________________________________________________________________________________ */
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// Função para reconectar ao broker MQTT caso a conexão seja perdida
void reconnect_mqtt ( void * pvParameters ) {
  // Tenta se reconectar até que a conexão seja bem-sucedida
  while(1){
    while (!client.connected()) 
    {
      Serial.print("Attempting MQTT connection...");
      // Tenta conectar com o ID "ESP8266Client"
      if (client.connect("ESP8266", "rafael", "senha")) 
      {
        Serial.println("connected");     // Se a conexão for bem-sucedida
      } 
      else 
      {
        // Caso falhe a conexão, imprime o código de erro e tenta novamente após 5 segundos
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        vTaskDelay( 5000 / portTICK_PERIOD_MS );  // Espera 5 segundos antes de tentar novamente
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void sinal_envio(){
    digitalWrite(LED_err, HIGH);
    digitalWrite(Buzzer, HIGH);
    vTaskDelay(50);
    digitalWrite(LED_err, LOW);
    digitalWrite(Buzzer, LOW);
    digitalWrite(LED_OnBoard, LOW);
}

void enviar_mensagem_mqtt(void * pvParameters){
    char *msg_rec = (char *) pvParameters;
    if(client.connected())
    {
      Serial.println("Dados enviados");
      Serial.println(msg_rec);
      client.publish("rastreio/esp32/camera1/localizacao",  msg_rec);
      //Copia para a variável o valor da nova mensagem para o buffer anterior para proxima comparação
      memcpy(msg_ant, msg, sizeof(msg));
    }
    else
    {
      Serial.println("Erro conexão");
    }
    vTaskDelete(NULL);
}

void Flash ( void * pvParameters ) {
  // Tenta se reconectar até que a conexão seja bem-sucedida
  while(1){
    bool estado;
    VL53L0X_RangingMeasurementData_t measure;
    const int distancia = 250;
    lox.rangingTest(&measure, false);
    estado = measure.RangeMilliMeter < distancia;
    //lox.getRangingMeasurement(&measure, false);
    if (measure.RangeStatus != 4) {
      if (estado == 1 && estadoanterior == 0){
        digitalWrite(LED_OnBoard, HIGH);
        }
      } else {
        digitalWrite(LED_OnBoard, LOW);
      }
    if (!estado){
      digitalWrite(LED_OnBoard, LOW);
    }
    estadoanterior = estado;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
