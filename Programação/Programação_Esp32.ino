#include <WiFi.h>
#include <PubSubClient.h>

/*inclusão das Biblioteca do FreeRTOS*/
#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/task.h>
#include <FreeRTOS/queue.h>


const char* ssid = "Valdecir 2GHz";  
const char* password = "02040810";

const char* mqtt_server = "54.233.88.93";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

bool estado = 0;
const int ledPin = 33;
const int button = 27;

xTimerHandle xTimer;
QueueHandle_t xFila;

/* Variáveis para armazenar o handle da Task */
TaskHandle_t xTaskSensorHandle;
TaskHandle_t xTaskMQTTHandle;

/* Protótipo das Tasks*/
void vTaskSensor(void *pvParameters ); 
void vTaskMQTT(void *pvParameters); 

/* timer rtos */
void callBackTimer(TimerHandle_t pxTimer );

/* Funções auxiliares */
void rtosInit();
void mqttInit();
int sensorRead();
void mqttSendJson(float valor1);
void mqttSendJsonIO(void);
void mqttSendJsonIOviatask(void);


void setup() {

  mqttInit(); 
  rtosInit();

  Serial.begin(115200);
  pinMode(33, OUTPUT);
  pinMode (27, INPUT);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  vTaskDelay(pdMS_TO_TICKS(1000));    /* Delay de 1 segundos */

}

void mqttInit() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void rtosInit(){

    xFila = xQueueCreate(1, sizeof(int));
if (xFila == NULL)
  {
     Serial.println("Erro: nao e possivel criar a fila");
     while(1); /* Sem a fila o funcionamento esta comprometido. Nada mais deve ser feito. */
  } 
    
    xTimer = xTimerCreate("TIMER",pdMS_TO_TICKS(2000),pdTRUE, 0, callBackTimer);

    xTaskCreatePinnedToCore(
      vTaskSensor,                       /* Funcao a qual esta implementado o que a tarefa deve fazer */
       "TaskADC",                        /* Nome (para fins de debug, se necessário) */
       configMINIMAL_STACK_SIZE + 1024,  /* Tamanho da stack (em words) reservada para essa tarefa */
       NULL,                             /* Parametros passados (nesse caso, não há) */
         2,                              /* Prioridade */
           &xTaskSensorHandle,          /* Handle da tarefa, opcional (nesse caso, não há) */
           APP_CPU_NUM);                /* Core */

    xTaskCreatePinnedToCore(vTaskMQTT,  "TaskMQTT",  configMINIMAL_STACK_SIZE + 2024,  NULL,  3,  &xTaskMQTTHandle,PRO_CPU_NUM);  

    xTimerStart(xTimer,0);

    /* A partir deste momento, o scheduler de tarefas entra em ação e as tarefas executam */
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("Sala1");
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(1000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
   mqttSendJsonIO();
}

void vTaskSensor(void *pvParameters )
{
  (void) pvParameters;
  
  if(digitalRead(button) == LOW) // Se o botão for pressionado
  {
    estado = !estado; // troca o estado do LED
    digitalWrite(ledPin, estado);
    while(digitalRead(button) == LOW);
    xQueueOverwrite(xFila, &estado);/* envia valor atual de count para fila*/
    vTaskDelay(pdMS_TO_TICKS(100)); /* Aguarda 100 ms antes de uma nova iteração*/      
  }   
}

/*Implementação da Task MQTT */
void vTaskMQTT(void *pvParameters){
  (void) pvParameters;
  int valor_recebido = 0;

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  while(1)
  {
    
      if(xQueueReceive(xFila, &valor_recebido, portMAX_DELAY) == pdTRUE) //verifica se há valor na fila para ser lido. Espera 1 segundo
      {
        if(!client.connected()){
          reconnect();
        }
        mqttSendJson(valor_recebido);   
         /* Para fins de teste de ocupação de stack, printa na serial o high water mark */
       //  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
       //  Serial.print(pcTaskGetTaskName(NULL));
       //  Serial.print(" : ");
      //  Serial.println(uxHighWaterMark);

        vTaskDelay(500);

      }
      else
      {
        Serial.println("TIMEOUT");
      }
  }
}

void mqttSendJson(float valor1){
    //Envia a mensagem ao broker
     /// . produzindo mensagem
   // DynamicJsonDocument doc(1024);
    doc["device"] = "ESP32";
    doc["analogico"] = valor1;
    char JSONmessageBuffer[200];
    serializeJson(doc, JSONmessageBuffer);
    client.publish("Sala1", JSONmessageBuffer);
    Serial.print("msg json enviado: ");
    Serial.println(JSONmessageBuffer);
}
