#include "arduino_secrets.h"
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <IRremote.h>
#include <Time.h>
#include <RTClib.h>

#include "DHT.h"
#include <WDT.h>
#include "FspTimer.h"
#include <MqttClient.h>
#include <NTPClient.h>

#define DHTTYPE DHT22
#define PIN_DHT22 7
#define PULSWEITE 5

#define GRENZWERT 250 
#define ZEIT_SEK 3000
unsigned long startTime = 0; //Zeitmessvariable für PC-State

const int alarmPin = 10;
const int magnPin = 2; //Read Kontakt Eingang

const int irLedPin = 13;
const int switchPin = 5; 
const int buttonPin = 3; //und pin 6 auch
const int keyPin = 11;

const int relayPin = 9;
const int pcPin = A0;
const int rgbPin = A1;
const int switchButtonLight = 8;

const char broker[] = BROKER_HOST_ADRESS;
int        port   = BROKER_HOST_PORT;
const char mqtt_user[] = BROKER_USER;
const char mqtt_pass[] = BROKER_PASSWORD;

const char TOPIC_PC_STATUS[] = "linus/haydn17/kellerzimmer/pc/status";
const char TOPIC_TEMP[]   = "linus/haydn17/kellerzimmer/temperature";
const char TOPIC_PC_HUMIDITY[] = "linus/haydn17/kellerzimmer/humidity";
const char TOPIC_PC_CMD[]    = "linus/haydn17/kellerzimmer/pc/command";
const char TOPIC_RGB_CMD[]   = "linus/haydn17/kellerzimmer/rgb/command";
const char TOPIC_RGB_STATUS[] = "linus/haydn17/kellerzimmer/rgb/status";

bool pc_status_mqtt = false;
bool last_pc_status = false;
float temp_mqtt = 0;
float last_temp = 0;
float humi_mqtt = 0;
float last_humi = 0;
bool rgb_status_mqtt = false;
bool last_rgb_status = false;

bool LEDblinkerState=false;
bool watchdogState=false;

bool alarmPinState = 0;

struct Button {
  const char* name;
  uint32_t hex;
};

const Button buttons[] = {
  {"Heller",     0xFF3AC5},
  {"Dunkler",    0xFFBA45},
  {"Skip",       0xFF827D},
  {"On/Off",     0xFF02FD},
  {"Rot",        0xFFA25D},
  {"Blau",       0xFF9A65},
  {"Gruen",       0xFF1AE5},
  {"Weiss",      0xFF22DD},
  {"Pink",       0xFF7887},
  {"Tuerkis",     0xFF18E7},
  {"Quick",      0xFFE817},
  {"Slow",       0xFFC837},
  {"Fade 1",     0xFF609F},
  {"Fade 2",     0xFFE01F},
  {"DIY1",       0xFF30CF},
  {"DIY2",       0xFFB04F},
  {"DIY3",       0xFF708F},
  {"DIY4",       0xFF10EF},
  {"DIY5",       0xFF906F},
  {"DIY6",       0xFF50AF},
  {"RedUp",      0xFF6897},
  {"RedDown",    0xFF48B7},
  {"GreenUp",    0xFFA857},
  {"GreenDown",  0xFF8877},
  {"BlueUp",     0xFF28D7},
  {"BlueDown",   0xFF08F7},
  {"Dunkelgelb", 0xFF7887}
};

const int buttonCount = sizeof(buttons) / sizeof(buttons[0]);

const int udpPort = 43332;           // Port, auf dem empfangen wird
char incomingPacket[255];

unsigned long lastPublishTime = 0;
const long PUBLISH_INTERVAL = 1500;

const char* esp32IP = "192.168.0.123"; // IP-Adresse des ESP32
const int udpTestPort = 43332;         // Port des ESP32 für den Verbindungstest
bool connectionOK = false;             // Status der Verbindung
unsigned long lastTestMillis = 0;      // Letzter Testzeitpunkt
const unsigned long testInterval = 1000 * 60 * 60 * 24; // 12 Stunden

const long NTP_TIME_OFFSET = 7200;
const char* NTP_SERVER = "pool.ntp.org";

FspTimer led_blinker_timer;
WiFiClient wifiClient;
WiFiUDP udp;
MqttClient mqttClient(wifiClient);
DHT dht(PIN_DHT22, DHTTYPE);
NTPClient timeClient(udp, NTP_SERVER, NTP_TIME_OFFSET, 60000);


void LEDBlinken();
void led_blinker_callback(timer_callback_args_t __attribute((unused)) *p_args);
void serialIncome();
void UPDIncome();


void setup() {
  Serial.begin(115200);
  Serial.println("\nProgramm Start");

  pinMode(alarmPin, OUTPUT);
  pinMode(irLedPin, OUTPUT);
  pinMode(switchPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(pcPin, INPUT);
  pinMode(rgbPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(magnPin, INPUT);
  pinMode(keyPin, INPUT);
  pinMode(switchButtonLight, OUTPUT);
  
  attachInterrupt(buttonPin,buttonChange,CHANGE);
  beginLEDTimer(20);

  dht.begin();
  connectWifi();
  connectMqtt();
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(3000);

  udp.begin(udpPort);
  timeClient.begin();
  IrSender.begin(irLedPin);
  Serial.print("UDP Port gestartet: ");
  Serial.println(udpPort);

  Serial.println("Setup Beendet, Watchdog startet in 30 Sekunden");
}

void loop() {
  
  if(millis()>30000 && watchdogState==false) //Watchdog erst nach 30 Sekunden starten
  {
    WDT.begin(5000);
    watchdogState=true;
    Serial.println("Watchdog gestartet!");
  }

  timeClient.update();
  temp_mqtt = dht.readTemperature();
  humi_mqtt = dht.readHumidity();
  checkAnalogPin();
  serialIncome();
  UPDIncome();

  if (istZeitZumTesten()) {
    testeVerbindung();
    verbindungsAnimation(connectionOK);
  }

  if (!mqttClient.connected()) {
    // Wenn die Verbindung verloren geht, versuche erneut zu verbinden
    Serial.println("MQTT connection lost. Reconnecting...");
    connectMqtt();
  }
  mqttClient.poll();

  // Daten nur in einem bestimmten Intervall lesen und senden
  if (millis() - lastPublishTime > PUBLISH_INTERVAL) {
    lastPublishTime = millis();
    publishSensorData();
  }
  WDT.refresh();

}

void connectWifi() {
  // *** NEU: DNS-Server manuell setzen zur Stabilisierung ***
  IPAddress dns(8, 8, 8, 8); // Google's public DNS
  WiFi.setDNS(dns);
  Serial.println("DNS-Server manuell auf 8.8.8.8 gesetzt.");

  Serial.print("Verbinde mit WLAN '");
  Serial.print(WIFI_SSID);
  Serial.print("'...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nVerbunden! IP-Adresse: " + WiFi.localIP().toString());
}

void connectMqtt() {
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setUsernamePassword(mqtt_user, mqtt_pass);

  // *** DIESE ZEILEN HINZUFÜGEN ***
  String clientId = "arduino-uno-r4-" + String(random(0xffff), HEX);
  mqttClient.setId(clientId);
  Serial.println("Verwende zufällige Client-ID: " + clientId);
  // ******************************

  Serial.print("Verbinde mit MQTT Broker '");

  // Falls dein Broker eine Authentifizierung benötigt:
  // mqttClient.setUsernamePassword(mqtt_user, mqtt_pass);

  Serial.print("Connecting to MQTT broker '");
  Serial.print(broker);
  Serial.print("'...");
  while (!mqttClient.connect(broker, port)) {
    Serial.print(".");
    // Warte 5 Sekunden vor dem nächsten Versuch
    delay(700);
  }
  Serial.println("\nMQTT connected!");

  // Topics mit QoS 2 abonnieren
  Serial.println("Subscribing to topics with QoS 2...");
  mqttClient.subscribe(TOPIC_PC_CMD, 2);
  mqttClient.subscribe(TOPIC_RGB_CMD, 2);
}

void publishSensorData() {

  if (pc_status_mqtt != last_pc_status) {
    last_pc_status = pc_status_mqtt;
    mqttClient.beginMessage(TOPIC_PC_STATUS, true, 1); // topic, retained, qos
    mqttClient.print(pc_status_mqtt);
    mqttClient.endMessage();
  }

  if (isnan(temp_mqtt) || isnan(humi_mqtt)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Nur senden, wenn sich der Wert signifikant ändert
  if (abs(temp_mqtt - last_temp) > 0.1) {
    last_temp = temp_mqtt;
    mqttClient.beginMessage(TOPIC_TEMP, false, 0); // topic, retained, qos
    mqttClient.print(temp_mqtt);
    mqttClient.endMessage();
  }
  
  if (abs(humi_mqtt - last_humi) > 0.5) {
    last_humi = humi_mqtt;
    mqttClient.beginMessage(TOPIC_PC_HUMIDITY, false, 0);
    mqttClient.print(humi_mqtt);
    mqttClient.endMessage();
  }
}

void onMqttMessage(int messageSize) {
  String topic = mqttClient.messageTopic();
  String payload = "";
  while (mqttClient.available()) {
    payload += (char)mqttClient.read();
  }

  Serial.println("---");
  Serial.print("Received message on topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  Serial.println(payload);
  Serial.println("---");


  if (topic == TOPIC_PC_CMD) {
    handlePcCommand(payload);
  }

  if (topic == TOPIC_RGB_CMD) {
    handleRgbCommand(payload);
  }
}

void handlePcCommand(String command) {
  if (command == "TOGGLE") {
    Serial.println("Toggling PC relay...");
    digitalWrite(relayPin,HIGH);
    delay(1000);
    digitalWrite(relayPin,LOW);
  }
  else if (command == "RESET")
  {
    Serial.println("Resetting PC relay...");
    digitalWrite(relayPin,HIGH);
    WDT.refresh();
    delay(4000);
    WDT.refresh();
    delay(4000);
    WDT.refresh();
    digitalWrite(relayPin,LOW);
  }
  else {
    Serial.print("Unknown PC command: ");
    Serial.println(command);
  }
}

void handleRgbCommand(String command) {
  // HINWEIS: Du musst die Hex-Codes für deine spezifische Fernbedienung
  // mit einem IR-Empfänger-Sketch herausfinden.
  // Die hier verwendeten Codes sind nur Beispiele!
  
  Serial.print("Handling RGB command: ");
  Serial.println(command);

  int code = getHexByName(command.c_str());
  if (code == 0)
  {
    Serial.print("Ungueltiger Code: ");
    Serial.println(command);
    return;
  }
  IrSender.sendNEC(code, 32);
}

void verbindungsAnimation(bool verbindung)
{
  bool startState=true;
  if(analogRead(rgbPin)<500)
  {
    startState=false;
    IrSender.sendNEC(getHexByIndex(3), 32);
    delay(10);
  }
  IrSender.sendNEC(getHexByIndex(26), 32);
  for(int i = 0; i < 7; i++)
  {
    IrSender.sendNEC(getHexByIndex(0), 32);
    delay(20);
  }
  WDT.refresh();
  for(int i = 350; i>0; i-=35)
  {
    delay(i);
     IrSender.sendNEC(getHexByIndex(3), 32);
    delay(i); 
    IrSender.sendNEC(getHexByIndex(3), 32);
  }
  WDT.refresh();
  delay(20); 
  IrSender.sendNEC(getHexByIndex(3), 32);
  delay(20); 
  IrSender.sendNEC(getHexByIndex(3), 32);
  
  if(verbindung) IrSender.sendNEC(getHexByIndex(6), 32);
  else IrSender.sendNEC(getHexByIndex(4), 32);

  for(int i = 8; i > 0; i--)
  {
    IrSender.sendNEC(getHexByIndex(1), 32);
    delay(i*50);
    WDT.refresh();
  }

  IrSender.sendNEC(getHexByIndex(3), 32);
  delay(1500);
  
  if(startState)
  {
    IrSender.sendNEC(getHexByIndex(3), 32);
    for(int i = 0; i < 7; i++)
    {
      IrSender.sendNEC(getHexByIndex(0), 32);
      delay(20);
    }
    IrSender.sendNEC(getHexByIndex(12), 32);
    
  }

}

void testeVerbindung() {
  // Verbindungstest senden
  lastTestMillis = millis();
  udp.beginPacket(esp32IP, udpTestPort);
  udp.write((const uint8_t*)"verbindungstest", strlen("verbindungstest"));
  udp.endPacket();
  Serial.println("Verbindungstest gesendet");
  WDT.refresh();

  // Auf Antwort warten
  unsigned long startWait = millis();
  while (millis() - startWait < 4000) { // 5 Sekunden warten
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(incomingPacket, 255);
      if (len > 0) {
        incomingPacket[len] = '\0'; // Null-Terminierung
        if (String(incomingPacket) == "verbindung") {
          connectionOK = true;
          Serial.println("ESP32-Verbindung erfolgreich!");
          return;
        }
      }
    }
  }
  WDT.refresh();

  connectionOK = false; // Keine Antwort oder falsche Antwort
  Serial.println("ESP32-Verbindung fehlgeschlagen!");
}

bool istZeitZumTesten() {
  // Check 1: Ist das große Intervall (z.B. 12 Stunden) abgelaufen?
  if (millis() - lastTestMillis >= testInterval) {
    return true;
  }

  // Check 2: Ist es eine der festgelegten Uhrzeiten (17:00 oder 21:00)?
  int stunde = timeClient.getHours();
  int minute = timeClient.getMinutes();

  bool istTriggerZeit = (stunde == 17 && minute == 0) || (stunde == 21 && minute == 0);

  // Wenn es die Trigger-Zeit ist UND der letzte Test länger als 65 Sekunden her ist
  // (um mehrfaches Auslösen in derselben Minute zu verhindern), dann ist es Zeit.
  if (istTriggerZeit && (millis() - lastTestMillis > 65000)) {
    return true;
  }

  return false;
}

void UPDIncome()
{
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Paketdaten lesen
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0; // Null-Terminierung für String
    }
    Serial.print("Empfangenes Paket: ");
    Serial.println(incomingPacket);

    // Optional: Auf einen bestimmten Trigger reagieren
    if (strstr(incomingPacket, "trigger") && pc_status_mqtt) //Only give Alarm if PC is powered on
    {
      bool startState=true;
      if(analogRead(rgbPin)<500)
      {
        IrSender.sendNEC(getHexByIndex(3), 32);
        //delay(10);
        startState=false;
      }
      
      IrSender.sendNEC(getHexByIndex(5), 32);
      delay(20);
      IrSender.sendNEC(getHexByIndex(3), 32);
      delay(50);
      IrSender.sendNEC(getHexByIndex(3), 32);
      delay(50);
      IrSender.sendNEC(getHexByIndex(3), 32);
      delay(50);
      IrSender.sendNEC(getHexByIndex(3), 32);
      
      if(!startState)
      {
        delay(50);
        IrSender.sendNEC(getHexByIndex(3), 32);
      }
      else
      {
        delay(20);
        IrSender.sendNEC(getHexByIndex(12), 32);
      }
    } 
  }
}


void buttonChange()
{
  if(digitalRead(switchPin)==HIGH&&digitalRead(keyPin)==HIGH)
  {
    if(millis()>30000)digitalWrite(relayPin,digitalRead(buttonPin));
  }
  else
  {
    static long time=0;
    if(digitalRead(buttonPin)==HIGH)
    {
      time=millis();
    }
    else
    {
      long dif = millis()-time;
      if(dif == 0)return;
      dif=dif/100.0;
      if(dif>0.01&&dif<5)
      {
        LEDblinkerState=false;
      }
      else if(dif>4&&dif<10)
      {
        if(analogRead(rgbPin)<500)
        {
          IrSender.sendNEC(getHexByIndex(3), 32);
          delay(20);
          IrSender.sendNEC(getHexByIndex(12), 32);
        }
        else
        {
          IrSender.sendNEC(getHexByIndex(3), 32);
        }

        
      }
    }
  }
}
  

void led_blinker_callback(timer_callback_args_t __attribute((unused)) *p_args) {
  static int counter = 0;
  static bool switchState = false;
  counter++;

  if(switchState && !digitalRead(switchPin) && analogRead(rgbPin)>500)
  {
    switchState=false;
    IrSender.sendNEC(getHexByIndex(3), 32);
  }

  if (counter % 10 == 0&&digitalRead(switchPin)) {
    IrSender.sendNEC(getHexByIndex(3), 32);
    IrSender.sendNEC(getHexByIndex(4), 32);
    switchState=true;
  }
  
}

bool beginLEDTimer(float rate) {
  uint8_t timer_type = GPT_TIMER;
  int8_t tindex = FspTimer::get_available_timer(timer_type);
  if (tindex < 0){
    tindex = FspTimer::get_available_timer(timer_type, true);
  }
  if (tindex < 0){
    return false;
  }

  FspTimer::force_use_of_pwm_reserved_timer();

  if(!led_blinker_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, led_blinker_callback)){
    return false;
  }

  if (!led_blinker_timer.setup_overflow_irq()){
    return false;
  }

  if (!led_blinker_timer.open()){
    return false;
  }

  if (!led_blinker_timer.start()){
    return false;
  }
  return true;
}

void checkAnalogPin() {
  int wert = analogRead(pcPin);
  pc_status_mqtt = wert > 500;
}


void serialIncome() {
  String readString="";
  while (Serial.available()>0) {
    delay(10);  
    char c = Serial.read();
    readString += c;
  }

  if(readString.length()>0)
  {
    /*
     0-> Frei
     1-> ComTest
     2-> GetPCState
    */
    readString.remove(readString.length()-1); //letztes Zeichen (Zeilenumbruch) entfernen
    if(readString=="0")
    {
      Serial.println("1 - IP Adresse");
      Serial.println("2 - PC Analog Wert");
      Serial.println("3 - Led Blinker State (change with 3_0 or 3_1)");
      Serial.println("4 - Read Temperature Sensor");
      Serial.println("5_x - Send IR code");
      Serial.println("6 - Verbindungstest");

    }
    if(readString=="1")
    {
      Serial.println("Verbindung vorhanden!");
      Serial.print("ip: ");
      Serial.println(WiFi.localIP());
    }
    else if (readString=="2")
    {
      Serial.print("PC AnalogIn: ");
      Serial.println(analogRead(A0));

      Serial.print("RGB AnalogIn: ");
      Serial.println(analogRead(A1));
    }
    else if (readString=="3")
    {
      Serial.print("LEDblinkerState: ");
      Serial.println(LEDblinkerState);
    }
    else if (readString=="3_0")
    {
      Serial.println("LEDblinkerState set to: 0");
      LEDblinkerState=0;
    }
    else if (readString=="3_1")
    {
      Serial.println("LEDblinkerState set to: 1");
      LEDblinkerState=1;
    }
    else if (readString=="4")
    {
      Serial.print("Temperature is: ");
      Serial.println(dht.readTemperature());
    }
    else if (readString[0]=='5' && readString[1]=='_')
    {
      String numberPart = readString.substring(2);
      int IRcode = numberPart.toInt();
      Serial.print("Executed Code: ");
      Serial.println(IRcode);
      IrSender.sendNEC(getHexByIndex(IRcode), 32);
    }
    else if(readString=="6")
    {
      String zeitString = timeClient.getFormattedTime(); 
      Serial.println(zeitString);
      testeVerbindung();
      verbindungsAnimation(connectionOK);
    }
    else if(readString=="6_1")
    {
      IrSender.sendNEC(getHexByIndex(3), 32);
      IrSender.sendNEC(getHexByIndex(19), 32);
      for(int i = 0; i < 45; i++)
      {
        IrSender.sendNEC(getHexByIndex(21), 32);
        delay(20);
      }

      delay(1000);

      for(int i = 0; i < 45; i++)
      {
        IrSender.sendNEC(getHexByIndex(20), 32);
        delay(20);
      }
    }
    
    
  }
  readString="";  
}

uint32_t getHexByIndex(int index) {
  if (index >= 0 && index < buttonCount) {
    return buttons[index].hex;
  }
  return 0;
}

// Zugriff über Name
uint32_t getHexByName(const char* name) {
  for (int i = 0; i < buttonCount; i++) {
    if (strcmp(buttons[i].name, name) == 0) {
      return buttons[i].hex;
    }
  }
  return 0;
}