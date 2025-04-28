/* ======================================== Inclusão de Bibliotecas */
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
/* ======================================== Configuração de GPIOs da câmera Thinker ESP32-CAM */

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


/* ======================================== */ // Definindo diretivas dos pinos
#define LED_OnBoard 4 // Flash conectado internamente ao GPIO 4
#define LED_err 12    // Led azul conectado ao GPIO 12
#define Buzzer 14     // Buzzer conectado ao GPIO 14
#define SDA_PIN 15    // SDA conectado ao GPIO 15
#define SCL_PIN 13    // SCL conectado ao GPIO 13

/* ======================================== SSID e Senha da Wifi */
const char* ssid = "Automacao";
const char* password = "127.0.0.1...";
/* ======================================== */

/*======================================== Definição do endereço IP do servidor MQTT*/
const char* mqtt_server = "172.16.10.175";

WiFiClient espClient; // Cliente Wi-Fi para comunicação com o broker MQTT
PubSubClient client(espClient); // Cliente MQTT usando o cliente Wi-Fi
long lastMsg = 0; //Variável para controlar o tempo entre as mensagens publicadas
char msg[50];  // Buffer para armazenar a mensagem
char msg_ant[50];// Buffer para mensagem anterior
/* ======================================== */

/* ======================================== Declaração de variáveis globais */
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
/* ________________________________________________________________________________ VOID SETUP() */
void setup() {
  // Modos de operação dos GPIOs:
  pinMode(LED_OnBoard, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(LED_err, OUTPUT);
  digitalWrite(LED_err, LOW);

  Wire.begin(SDA_PIN, SCL_PIN); // Ligação dos pinos I2C 

  // Desativar detector de queda de energia.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
 

  /* ---------------------------------------- Definição da comunicação serial */
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  /* ---------------------------------------- */

  /* ---------------------------------------- */
  client.setServer(mqtt_server, 1883);   // Configura o broker MQTT (porta 1883)
  //client.setCallback(callback);           // Define a função de callback para tratar mensagens recebidas 

  /* ---------------------------------------- Camera configuration. */
  Serial.println("Iniciando configuração da camera...");
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

  esp_err_t err = esp_camera_init(&config);  // Inicializa o objeto da câmera e o paramêtro err recebe o status de erro caso ocorra um problema na inicialização

  //Se houver um problema durante a inicialização informa a mensagem e reinicia o ESP.
  if (err != ESP_OK) {
    Serial.printf("Erro na inicialização da câmera 0x%x", err); 
    ESP.restart();
  }
  
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  
  Serial.println("Camera configurada com sucesso.");
  Serial.println();
  /* ---------------------------------------- */
  /* ---------------------------------------- Conectando à Wifi. */
  WiFi.mode(WIFI_STA);
  Serial.println("------------");
  Serial.print("Conectando-se a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int connecting_process_timed_out = 20; //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;

  while (WiFi.status() != WL_CONNECTED) {
    //Flash pisca até que a Wifi esteja conectada
    Serial.print(".");
    digitalWrite(LED_OnBoard, HIGH);
    vTaskDelay(250);
    digitalWrite(LED_OnBoard, LOW);
    vTaskDelay(250);
    if(connecting_process_timed_out > 0) connecting_process_timed_out--;
    // Se o tempo de timeout for superado sem conexão o Esp será reiniciado
    if(connecting_process_timed_out == 0) {
      vTaskDelay(1000);
      ESP.restart();
    }
  }

  /* ---------------------------------------- Inicializa a comunicação I2C do Sensor*/
  Serial.println("Adafruit VL53L0X Testando...");
  if (!lox.begin()) {
    Serial.println(F("Falha ao iniciar o VL53L0X"));
    while (1)
      ;
  }

  /* ---------------------------------------- Criando a task "QRCodeReader_Task" usando o a função xTaskCreatePinnedToCore() */
  // Essa task é responsável pela leitura em tempo de execução dos códigos QR
  xTaskCreatePinnedToCore(
             QRCodeReader,          /* Função da Task. */
             "QRCodeReader_Task",   /* Nome da Task. */
             10000,                 /* Memória destinada a Task */
             NULL,                  /* Parâmetro para Task */
             5,                     /* Nível de prioridade da Task */
             &QRCodeReader_Task,    /* Handle da Task */
             0);                    /* Núcleo onde a task é executada 0 ou 1 */
  /* ---------------------------------------- */

  /* ---------------------------------------- Criando a task "reconnect_Task" usando o a função xTaskCreatePinnedToCore() */
  // Essa task é responsável pela reconecção ao Broker MQTT em caso de problemas
  xTaskCreatePinnedToCore(
             reconnect_mqtt,     /* Função da Task. */
             "reconnect_mqtt",   /* Nome da Task. */
             2048,               /* Memória destinada a Task */
             NULL,               /* Parâmetro para Task */
             2,                  /* Nível de prioridade da Task */
             &reconnect_Task,    /* Handle da Task */
             1);                 /* Núcleo onde a task é executada 0 ou 1 */
  /* ---------------------------------------- */

  /* ---------------------------------------- */
  /* ---------------------------------------- Criando a task "flash_Task" usando o a função xTaskCreatePinnedToCore() */
  // Essa task é responsável por ligar o Flash quando o sensor detecta um objeto proximo.
  xTaskCreatePinnedToCore(
             Flash,          /* Função da Task. */
             "Flash",        /* Nome da Task. */
             2048,           /* Memória destinada a Task */
             NULL,           /* Parâmetro para Task */
             2,              /* Nível de prioridade da Task */
             &flash_Task,    /* Handle da Task */
             0);             /* Núcleo onde a task é executada 0 ou 1 */
  /* ---------------------------------------- */ 

}

/* ________________________________________________________________________________ */
void loop() {
  // Delay para que não ocorram problemas com o watchdog timer
  vTaskDelay(10);
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Função executada na task de leitura de QRcode */
void QRCodeReader( void * pvParameters ){
  /* ---------------------------------------- */
  Serial.println("QRCodeReader está pronta");
  Serial.print("QRCodeReader executando no core ");
  Serial.println(xPortGetCoreID());
  Serial.println();
  /* ---------------------------------------- */

  /* ---------------------------------------- Loop da leitura em tempo real */
  while(1){
      q = quirc_new();
      if (q == NULL){
        Serial.print("Erro ao criar o objeto quirc \r\n");  
        continue;
      }
    
      fb = esp_camera_fb_get();
      if (!fb)
      {
        Serial.println("Captura da Camera falhou");
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
          Serial.println("Erro na decodificação");
        } else {
          Serial.printf("A decodificação foi um sucesso:\n");
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

/* ________________________________________________________________________________ Função para exibir os resultados no Monitor e também inicializa a Task de envio MQTT*/
void dumpData(const struct quirc_data *data)
{
  Serial.printf("Version: %d\n", data->version);
  Serial.printf("ECC level: %c\n", "MLHQ"[data->ecc_level]);
  Serial.printf("Mask: %d\n", data->mask);
  Serial.printf("Length: %d\n", data->payload_len);
  Serial.printf("Payload: %s\n", data->payload);
  
  sprintf(msg, "%s", data->payload);
  // A task só inicializa caso a nova mensagem seja diferente da mensagem anterior. A mesma mensagem não será enviada mais de uma vez, para que não ocorram dados duplicados.
  if(memcmp(msg, msg_ant, sizeof(msg)) != 0){
    //Sinal luminoso e sonoro
    sinal_envio();
    //Cria task de envio
    xTaskCreatePinnedToCore(
            enviar_mensagem_mqtt,     /* Função da Task. */
            "enviar_mensagem_mqtt",   /* Nome da Task. */
            10000,                    /* Memória destinada a Task */
            msg,                      /* Parâmetro para Task */
            1,                        /* Nível de prioridade da Task */
            &envio_Task,              /* Handle da Task */
            1);                       /* Núcleo onde a task é executada 0 ou 1 */
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
      if (client.connect("ESP8266", "cepedi_pos", "cepedi123")) 
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
// Função criada para emitir sinal sonoro e luminoso ao enviar mensagem MQTT
void sinal_envio(){
    digitalWrite(LED_err, HIGH);
    digitalWrite(Buzzer, HIGH);
    vTaskDelay(50);
    digitalWrite(LED_err, LOW);
    digitalWrite(Buzzer, LOW);
    digitalWrite(LED_OnBoard, LOW);
}

// Função responsável por enviar mensagem MQTT após uma leitura
void enviar_mensagem_mqtt(void * pvParameters){
    char *msg_rec = (char *) pvParameters;
    // Se o cliente estiver conectado a mensagem será enviada, senão será enviada a menasgem de erro de conexão
    if(client.connected()) 
    {
      Serial.println("Dados enviados");
      Serial.println(msg_rec);
      client.publish("rastreio/esp32/camera1/localizacao",  msg_rec); // Define o tópico no qual será enviada a mensagem
      //Copia para a variável o valor da nova mensagem para o buffer anterior para proxima comparação
      memcpy(msg_ant, msg, sizeof(msg));
    }
    else
    {
      Serial.println("Erro conexão");
    }
    vTaskDelete(NULL);
}

// Função Flash responsável por ligar o flash na presença de um objeto.
void Flash ( void * pvParameters ) {
  // Tenta se reconectar até que a conexão seja bem-sucedida
  while(1){
    bool estado;
    VL53L0X_RangingMeasurementData_t measure; // Dados do sensor Laser
    const int distancia = 250; // Distância mínima para que o sistema ative o flash
    lox.rangingTest(&measure, false);
    estado = measure.RangeMilliMeter < distancia;
    // Lógica de borda de subida para ligar o flash, o flash só é desligado na precisa de um NOVO objeto.
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
