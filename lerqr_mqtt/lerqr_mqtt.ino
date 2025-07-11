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
TaskHandle_t monitoramento_Sensor_Task;
TaskHandle_t monitoramento_Botao_Task;
TaskHandle_t erro_na_linha_Task;

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
#define botao1 12    // botao conectado ao GPIO 12
#define botao2 2    // botao conectado ao GPIO 2
#define Buzzer 14     // Buzzer conectado ao GPIO 14
#define SDA_PIN 15    // SDA conectado ao GPIO 15
#define SCL_PIN 13    // SCL conectado ao GPIO 13


const int numero_posto = 2;
char topico_dispositivo[40];
char topico_sistema[35];
char id_posto[15];

bool estadoBotao = false;     // Estado atual do botão (ligado/desligado)
bool ultimoEstadoBotao = LOW; // Último estado lido do botão
unsigned long ultimaLeitura = 0;
bool estadoBotao2 = false;     // Estado atual do botão (ligado/desligado)
bool ultimoEstadoBotao2 = LOW; // Último estado lido do botão
unsigned long ultimaLeitura2 = 0;
const unsigned long debounceDelay = 50; // Tempo de debounce (em milissegundos)


/* ======================================== SSID e Senha da Wifi */
const char* ssid = "UBQ-Automacao";
const char* password = "127.0.0.1...";
/* ======================================== */

/*======================================== Definição do endereço IP do servidor MQTT*/
const char* mqtt_server = "172.16.10.175";

void callback(char* topic, byte* message, unsigned int length);
void envia_dispositivo(char* msg);
void realiza_leitura();
void para_leitura();
void beep();

WiFiClient espClient; // Cliente Wi-Fi para comunicação com o broker MQTT
PubSubClient client(mqtt_server, 1883, callback, espClient); // Cliente MQTT usando o cliente Wi-Fi
long lastMsg = 0; //Variável para controlar o tempo entre as mensagens publicadas
char msg[50];  // Buffer para armazenar a mensagem
//char msg_ant[50];// Buffer para mensagem anterior
/* ======================================== */

/* ======================================== Declaração de variáveis globais */
struct QRCodeData
{
  bool valid;
  int dataType;
  uint8_t payload[1024];
  int payloadLen;
};


/* ======================================== */

// VL53L0X (Using I2C) - SENSOR
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool estado;
bool estadoanterior;
/* ________________________________________________________________________________ VOID SETUP() */
void setup() {
  snprintf(topico_dispositivo, sizeof(topico_dispositivo), "rastreio/esp32/posto_%d/dispositivo", numero_posto);
  snprintf(topico_sistema, sizeof(topico_sistema), "rastreio/esp32/posto_%d/sistema", numero_posto);
  snprintf(id_posto, sizeof(id_posto), "posto_%d", numero_posto);
  // Modos de operação dos GPIOs:
  pinMode(LED_OnBoard, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(botao1, INPUT_PULLUP);
  pinMode(botao2, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN); // Ligação dos pinos I2C 

  // Desativar detector de queda de energia.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
 
  //

  /* ---------------------------------------- Definição da comunicação serial */
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  /* ---------------------------------------- */

  /* ---------------------------------------- */

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
  config.fb_location = CAMERA_FB_IN_PSRAM;

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
        50000,                 /* Memória destinada a Task */
        NULL,                  /* Parâmetro para Task */
        1,                     /* Nível de prioridade da Task */
        &QRCodeReader_Task,    /* Handle da Task */
        0);                    /* Núcleo onde a task é executada 0 ou 1 */
/* ---------------------------------------- */

  /* ---------------------------------------- Criando a task "reconnect_Task" usando o a função xTaskCreatePinnedToCore() */
  // Essa task é responsável pela reconecção ao Broker MQTT em caso de problemas
  xTaskCreatePinnedToCore(
             reconnect_mqtt,     /* Função da Task. */
             "reconnect_mqtt",   /* Nome da Task. */
             4096,               /* Memória destinada a Task */
             NULL,               /* Parâmetro para Task */
             2,                  /* Nível de prioridade da Task */
             &reconnect_Task,    /* Handle da Task */
             1);                 /* Núcleo onde a task é executada 0 ou 1 */
  /* ---------------------------------------- */

  /* ---------------------------------------- */
  /* ---------------------------------------- Criando a task "flash_Task" usando o a função xTaskCreatePinnedToCore() */
  // Essa task é responsável por ligar o Flash quando o sensor detecta um objeto proximo.
  xTaskCreatePinnedToCore(
             monitoramento_Sensor,          /* Função da Task. */
             "monitoramento_Sensor",        /* Nome da Task. */
             4096,           /* Memória destinada a Task */
             NULL,           /* Parâmetro para Task */
             3,              /* Nível de prioridade da Task */
             &monitoramento_Sensor_Task,    /* Handle da Task */
             0);             /* Núcleo onde a task é executada 0 ou 1 */
  /* ---------------------------------------- */ 
  /* ---------------------------------------- */
  /* ---------------------------------------- Criando a task "flash_Task" usando o a função xTaskCreatePinnedToCore() */
  // Essa task é responsável por ligar o Flash quando o sensor detecta um objeto proximo.
  xTaskCreatePinnedToCore(
             monitoramento_Botao,          /* Função da Task. */
             "monitoramento_Botao",        /* Nome da Task. */
             4096,           /* Memória destinada a Task */
             NULL,           /* Parâmetro para Task */
             3,              /* Nível de prioridade da Task */
             &monitoramento_Botao_Task,    /* Handle da Task */
             0);             /* Núcleo onde a task é executada 0 ou 1 */
  /* ---------------------------------------- */ 
}

/* ________________________________________________________________________________ */
void loop() {
  vTaskDelay(10);
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Função executada na task de leitura de QRcode */
void QRCodeReader( void * pvParameters ){
  /* ---------------------------------------- */
  Serial.println("QRCodeReader está pronta");
  /* ----------------------------------------  da leitura em tempo real */
  while(1){
      struct quirc *q = NULL;
      uint8_t *image = NULL;  
      camera_fb_t * fb = NULL;
      struct quirc_code code;
      struct quirc_data data;
      quirc_decode_error_t err;
      struct QRCodeData qrCodeData;
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
          Serial.printf("A decodificação foi um sucesso\n");
          dumpData(&data);
          //xSemaphoreTake(xSemaphore, portMAX_DELAY);
          //xSemaphoreTake(xSemaphore, portMAX_DELAY);
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

/* ________________________________________________________________________________ Função para exibir os resultados no Monitor e também inicializa a Task de envio MQTT*/
void dumpData(const struct quirc_data *data)
{

  sprintf(msg, "%s", data->payload);
  // A task só inicializa caso a nova mensagem seja diferente da mensagem anterior. A mesma mensagem não será enviada mais de uma vez, para que não ocorram dados duplicados.
  sinal_envio();
  envia_dispositivo(msg);
  para_leitura();
}

/* ________________________________________________________________________________ */
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// Função para reconectar ao broker MQTT caso a conexão seja perdida
void reconnect_mqtt ( void * pvParameters ) {
  // Tenta se reconectar até que a conexão seja bem-sucedida
  while(1){
    client.loop();
    while (!client.connected()) 
    {
      Serial.print("Attempting MQTT connection...");
      // Tenta conectar com o ID "camera1", informação de usuário e senha para o broker mqtt
      if (client.connect(id_posto, id_posto, "cepedi123"))  
      {
        client.subscribe(topico_sistema);
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
    digitalWrite(Buzzer, HIGH);
    vTaskDelay(50);
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
      Serial.print("Tópico: ");
      Serial.println(topico_dispositivo);
      Serial.print("Mensagem: ");
      Serial.println(msg_rec);

      client.publish(topico_dispositivo, msg_rec); // Define o tópico no qual será enviada a mensagem
    }
    else
    {
      Serial.println("Erro conexão");
    }
    vTaskDelete(NULL);
}

// Variáveis de debounce
bool estadoEstavel = false;             // Estado final confiável
bool leituraAnterior = false;          // Última leitura do sensor
unsigned long tempoUltimaLeitura = 0;  // Quando a mudança começou
const int tempoDebounce = 200;         // Tempo mínimo (ms) para validar mudança

void beep(){
    digitalWrite(Buzzer, HIGH);
    vTaskDelay(50);
    digitalWrite(Buzzer, LOW);
}

void monitoramento_Sensor ( void * pvParameters ) {
  while(1){
    VL53L0X_RangingMeasurementData_t measure;
    const int distancia = 300;
    lox.rangingTest(&measure, false);
    bool leituraAtual = measure.RangeMilliMeter < distancia;

    // Se a leitura mudou em relação à anterior
    if (leituraAtual != leituraAnterior) {
      leituraAnterior = leituraAtual;
      tempoUltimaLeitura = millis();  // Marca o tempo da mudança
    }

    // Se o tempo de estabilidade foi atingido
    if ((millis() - tempoUltimaLeitura) > tempoDebounce) {
      // E se o estado ainda não foi atualizado
      if (leituraAtual != estadoEstavel) {
        // Aqui temos uma mudança de borda real, com debounce
        bool estadoAnterior = estadoEstavel;
        estadoEstavel = leituraAtual;

        if (estadoEstavel && !estadoAnterior) {
          Serial.println("BS");
          realiza_leitura();
          envia_dispositivo("BS");
        } 
        else if (!estadoEstavel && estadoAnterior) {
          Serial.println("BD");
          envia_dispositivo("BD");
          para_leitura();
        }
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // 10 ms de intervalo
  }
}

void monitoramento_Botao ( void * pvParameters ) {
  while(1){
    int leitura = digitalRead(botao1);

    // Verifica se o botão mudou de estado
    if (leitura != ultimoEstadoBotao) {
      ultimaLeitura = millis(); // Reinicia o temporizador
    }

    // Se passou o tempo de debounce e o botão continua no mesmo estado
    if ((millis() - ultimaLeitura) > debounceDelay) {
      // Verifica se o estado mudou de fato
      if (leitura != estadoBotao) {
        estadoBotao = leitura;

        // Se o botão foi pressionado (nível LOW por causa do PULLUP)
        if (estadoBotao == LOW) {
          envia_dispositivo("BT1");
          beep();
          Serial.println("BT1"); // Inverte o LED
        }
      }
    }

    ultimoEstadoBotao = leitura;

    int leitura2 = digitalRead(botao2);
    // Verifica se o botão mudou de estado
    if (leitura2 != ultimoEstadoBotao2) {
      ultimaLeitura = millis(); // Reinicia o temporizador
    }

    // Se passou o tempo de debounce e o botão continua no mesmo estado
    if ((millis() - ultimaLeitura2) > debounceDelay) {
      // Verifica se o estado mudou de fato
      if (leitura2 != estadoBotao2) {
        estadoBotao2 = leitura2;

        // Se o botão foi pressionado (nível LOW por causa do PULLUP)
        if (estadoBotao2 == LOW) {
          envia_dispositivo("BT2");
          beep();
          Serial.println("BT2"); // Inverte o LED
        }
      }
    }

    ultimoEstadoBotao2 = leitura2;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// FUNÇÃO PARA SINALIZAR ERRO NA LINHA
void erro_na_linha(void * pvParameters){
  while(1){
    digitalWrite(Buzzer, HIGH);
    vTaskDelay(50);
    digitalWrite(Buzzer, LOW);
    vTaskDelay(50);
    digitalWrite(Buzzer, HIGH);
    vTaskDelay(50);
    digitalWrite(Buzzer, LOW);
    vTaskDelay(50);
    digitalWrite(Buzzer, HIGH);
    vTaskDelay(50);
    digitalWrite(Buzzer, LOW);
    vTaskDelay(500);
  }
}
//UTILIZADA APENAS PARA RECEBER MENSAGENS
/* ________________________________________________________________________________ */
// Função de callback que é chamada sempre que uma nova mensagem chega a um tópico inscrito
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");  // Imprime o tópico
  Serial.print(topic);
  Serial.print(". Message: ");
  
  String messageTemp = "";  // String temporária para armazenar a mensagem recebida

  // Imprime e armazena os dados da mensagem
  for (int i = 0; i < length; i++) { 
    messageTemp += (char)message[i]; // Adiciona o byte à string messageTemp
  }
  Serial.println("messageTemp");
  if(messageTemp == "erro_0"){
    if(estado){
    realiza_leitura();
    }
    Serial.println("Código inválido");
  }  
  if(messageTemp == "iniciar_erro_1"){
      xTaskCreatePinnedToCore(
        erro_na_linha,     /* Função da Task. */
        "erro_na_linha",   /* Nome da Task. */
        2000,                    /* Memória destinada a Task */
        NULL,                      /* Parâmetro para Task */
        2,                        /* Nível de prioridade da Task */
        &erro_na_linha_Task,              /* Handle da Task */
        0);                       /* Núcleo onde a task é executada 0 ou 1 */
  }
  if(messageTemp == "parar_erro_1"){
    digitalWrite(Buzzer, LOW);
    vTaskDelete(erro_na_linha_Task);
  }    
}

void realiza_leitura(){
  digitalWrite(LED_OnBoard, HIGH);
  vTaskResume(QRCodeReader_Task);
  //xSemaphoreGive(xSemaphore);
}

void para_leitura(){
  digitalWrite(LED_OnBoard, LOW);
  vTaskSuspend(QRCodeReader_Task);
}
void envia_dispositivo(char* msg){
    xTaskCreatePinnedToCore(
        enviar_mensagem_mqtt,     /* Função da Task. */
        "enviar_mensagem_mqtt",   /* Nome da Task. */
        20000,                    /* Memória destinada a Task */
        msg,                      /* Parâmetro para Task */
        4,                        /* Nível de prioridade da Task */
        &envio_Task,              /* Handle da Task */
        1);                       /* Núcleo onde a task é executada 0 ou 1 */
}