#include "arduino_secrets.h"
#include "thingProperties.h"
#include "WiFiS3.h"
#include <WiFiUdp.h>
#include <IRremote.h>
#include <Time.h>
#include <RTClib.h>

#include "DHT.h"
#include <WDT.h>
#include "FspTimer.h"
#include <arduinomqttclient.h>

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

const char* mqtt_client = "LinusUnoR4_1";           
const char* broker = "[2a02:8070:d81:4400::8816]";
const char* passwd = "3e67L\"=ODd=6";   
const int port = 1883;
const char* topic = "arduino/unoR4/test";

bool LEDblinkerState=false;
bool watchdogState=false;

bool alarmPinState = 0;
int IRcodes[27] = {0xFF3AC5, 0xFFBA45, 0xFF827D, 0xFF02FD, 0xFFA25D, 0xFF9A65, 0xFF1AE5, 0xFF22DD, 0xFF7887, 0xFF18E7, 0xFFE817, 0xFFC837, 0xFF609F, 0xFFE01F, 0xFF30CF, 0xFFB04F, 0xFF708F, 0xFF10EF, 0xFF906F, 0xFF50AF, 0xFF6897, 0xFF48B7, 0xFFA857, 0xFF8877, 0xFF28D7, 0xFF08F7, 0xFF7887};

FspTimer led_blinker_timer;
WiFiClient wifiClient;
//MqttClient mqttClient(wifiClient);
DHT dht(PIN_DHT22, DHTTYPE);

WiFiUDP udp;
const int udpPort = 43332;           // Port, auf dem empfangen wird
char incomingPacket[255];

const char* esp32IP = "192.168.0.123"; // IP-Adresse des ESP32
const int udpTestPort = 43332;         // Port des ESP32 für den Verbindungstest
bool connectionOK = false;             // Status der Verbindung
unsigned long lastTestMillis = 0;      // Letzter Testzeitpunkt
const unsigned long testInterval = 1000 * 60 * 60 * 24; // 12 Stunden

void LEDBlinken();
void led_blinker_callback(timer_callback_args_t __attribute((unused)) *p_args);
void serialIncome();
void UPDIncome();


void setup() {
  Serial.begin(115200);
  Serial.println("Programm Start");

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
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(3000);
  
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(4);
  ArduinoCloud.printDebugInfo();

  udp.begin(udpPort);
  IrSender.begin(irLedPin);
  Serial.print("UDP Port gestartet: ");
  Serial.println(udpPort);

  TimeService.setSyncInterval(600); //Alle 10 Minuten neu Time syncen

  Serial.println("Setup Beendet, Watchdog startet in 30 Sekunden");
}

void loop() {
  
  if(millis()>30000 && watchdogState==false)
  {
    WDT.begin(5000);
    watchdogState=true;
    Serial.println("Watchdog gestartet!");
  }

  temperaturIOT = dht.readTemperature();
  checkAnalogPin();
  ArduinoCloud.update();
  serialIncome();
  WDT.refresh();
  UPDIncome();

  if (istZeitZumTesten()) {
    testeVerbindung();
    verbindungsAnimation(connectionOK);
  }

}

void verbindungsAnimation(bool verbindung)
{
  bool startState=true;
  if(analogRead(rgbPin)<500)
  {
    startState=false;
    IrSender.sendNEC(IRcodes[3], 32);
    delay(10);
  }
  IrSender.sendNEC(IRcodes[26], 32);
  for(int i = 0; i < 7; i++)
  {
    IrSender.sendNEC(IRcodes[0], 32);
    delay(20);
  }
  WDT.refresh();
  for(int i = 350; i>0; i-=35)
  {
    delay(i);
     IrSender.sendNEC(IRcodes[3], 32);
    delay(i); 
    IrSender.sendNEC(IRcodes[3], 32);
  }
  WDT.refresh();
  delay(20); 
  IrSender.sendNEC(IRcodes[3], 32);
  delay(20); 
  IrSender.sendNEC(IRcodes[3], 32);
  
  if(verbindung) IrSender.sendNEC(IRcodes[6], 32);
  else IrSender.sendNEC(IRcodes[4], 32);

  for(int i = 8; i > 0; i--)
  {
    IrSender.sendNEC(IRcodes[1], 32);
    delay(i*50);
    WDT.refresh();
  }

  IrSender.sendNEC(IRcodes[3], 32);
  delay(1500);
  
  if(startState)
  {
    IrSender.sendNEC(IRcodes[3], 32);
    for(int i = 0; i < 7; i++)
    {
      IrSender.sendNEC(IRcodes[0], 32);
      delay(20);
    }
    IrSender.sendNEC(IRcodes[12], 32);
  }

}

void testeVerbindung() {
  // Verbindungstest senden
  lastTestMillis = millis();
  udp.beginPacket(esp32IP, udpTestPort);
  udp.write((const uint8_t*)"verbindungstest", strlen("verbindungstest"));
  udp.endPacket();
  Serial.println("Verbindungstest gesendet");

  // Auf Antwort warten
  unsigned long startWait = millis();
  while (millis() - startWait < 5000) { // 5 Sekunden warten
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

  connectionOK = false; // Keine Antwort oder falsche Antwort
  Serial.println("ESP32-Verbindung fehlgeschlagen!");
}

bool istZeitZumTesten() {
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastTestMillis >= testInterval) {
    lastTestMillis = millis();
    return true;
  }

  // Lokale Zeit aus ThingProperties verwenden (falls Arduino Cloud aktiv):
  time_t UnixTime;
  UnixTime = ArduinoCloud.getLocalTime();
  DateTime now = UnixTime;

  if ((now.hour() == 17 && now.minute() == 0 || now.hour() == 21 && now.minute() == 0)&&currentMillis - lastTestMillis > 1000*65) {
    lastTestMillis = millis();
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
    if (strstr(incomingPacket, "trigger") && pC_StatusIOT) //Only give Alarm if PC is powered on
    {
      bool startState=true;
      if(analogRead(rgbPin)<500)
      {
        IrSender.sendNEC(IRcodes[3], 32);
        //delay(10);
        startState=false;
      }
      
      IrSender.sendNEC(IRcodes[5], 32);
      delay(20);
      IrSender.sendNEC(IRcodes[3], 32);
      delay(50);
      IrSender.sendNEC(IRcodes[3], 32);
      delay(50);
      IrSender.sendNEC(IRcodes[3], 32);
      delay(50);
      IrSender.sendNEC(IRcodes[3], 32);
      
      if(!startState)
      {
        delay(50);
        IrSender.sendNEC(IRcodes[3], 32);
      }
      else
      {
        delay(20);
        IrSender.sendNEC(IRcodes[12], 32);
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
          IrSender.sendNEC(IRcodes[3], 32);
          delay(20);
          IrSender.sendNEC(IRcodes[12], 32);
        }
        else
        {
          IrSender.sendNEC(IRcodes[3], 32);
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
    IrSender.sendNEC(IRcodes[3], 32);
  }

  if (counter % 10 == 0&&digitalRead(switchPin)) {
    IrSender.sendNEC(IRcodes[3], 32);
    IrSender.sendNEC(IRcodes[4], 32);
    switchState=true;
  }
  
}

void onColorIOTChange()
{

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
  pC_StatusIOT = wert > 500;
}



void onPCSwitchIOTChange()  {
  if(pC_SwitchIOT&&millis()>20000)
  {
    digitalWrite(relayPin,HIGH);
    delay(1000);
    digitalWrite(relayPin,LOW);
  }
  else
  {
    digitalWrite(relayPin,LOW);
  }
  pC_SwitchIOT=false;
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
      IrSender.sendNEC(IRcodes[IRcode], 32);
    }
    else if(readString=="6")
    {
      time_t UnixTime;
      UnixTime = ArduinoCloud.getLocalTime() - 101;
      Serial.println(ctime(&UnixTime));
      testeVerbindung();
      verbindungsAnimation(connectionOK);
    }
    else if(readString=="6_1")
    {
      IrSender.sendNEC(IRcodes[3], 32);
      IrSender.sendNEC(IRcodes[19], 32);
      for(int i = 0; i < 45; i++)
      {
        IrSender.sendNEC(IRcodes[21], 32);
        delay(20);
      }

      delay(1000);

      for(int i = 0; i < 45; i++)
      {
        IrSender.sendNEC(IRcodes[20], 32);
        delay(20);
      }
    }
    
    
  }
  readString="";  
}