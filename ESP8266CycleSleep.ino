#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Include API-Headers
extern "C" {
#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "cont.h"
}

// SMS Service:
const char* SmsServer = "gateway.sms77.io";
const char* SmsUser ="SmsUser";
const char* SmsApiKey = "SmsApiKey";
const char* SmsNr = "SmsNr";

// WLAN Netzdaten
const char* ssid = "SSID";
const char* password = "PASSWORD";

// MQTT Broker
const char* mqtt_server = "IP";
const int mqtt_port = PORT;
long lastMsg = 0;
char msg[50];

// Thinkspeak - Server
String apiKey = "APIKEY";
const char* ThinkSpeakServer = "api.thingspeak.com";

#define ALARM_PIN 14                                // pin to test for alarm condition
#define BUZZER_PIN 5                                // pin to trigger buzzer
#define SLEEP_TIME 10*60*1000000                    // sleep intervalls in us
#define BATT_WARNING_VOLTAGE 3.2                    // Voltage for Low-Bat warning
#define WIFI_CONNECT_TIMEOUT_S 20                   // max time for wifi connect to router, if exceeded restart

// RTC-MEM Adresses
#define RTC_BASE 65
#define RTC_WAKE_COUNT 66
#define RTC_BAT_STATE  70
#define RTC_ALARM_STATE 71

// mail Types
#define MAIL_WELCOME 1     // mail at startup
#define MAIL_ALARM 3
#define MAIL_LOW_BAT 4

// global variables
byte buf[10];
byte BatState;
byte AlarmState;
uint32_t sleepCount;

String Str_StatWaterLeak;
String Str_StatBatLow;

double U_Bat;     // Batterie Spannung
bool pinState;    // Wasser Status PIN
bool ColdStart;

WiFiClient client;
PubSubClient MqttClient(client);

void MqttReconnect() {  // Loop until we're reconnected
  int MqttCnt = 0;
  while (!MqttClient.connected() & (MqttCnt < 5)) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (MqttClient.connect("ESP8266Client", "owntracks", "Minuten01")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(MqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
      MqttCnt++;
    }
  }
}

boolean SendSms(byte MailType)
{
  if (true)   // Hautschalter um SMS versand zu deaktivieren
  {
    if (client.connect(SmsServer, 80))
    {
      String url = "";
      if (MailType == MAIL_ALARM)
      { // send the HTTP PUT request:
        String text = "WaterLeakDetection";
        url = "/api/sms?u=" + SmsUser + "&p=" + SmsApiKey + "&to=" + SmsNr + "&text=WaterLeakDetection:%20WATER%20ALARM&type=direct&from=sms77.de&return_msg_id=1";
      }
      if (MailType == MAIL_LOW_BAT)
      { // send the HTTP PUT request:
        url = "/api/sms?u=" + SmsUser + " &p=" + SmsApiKey + " &to=" + SmsNr + "&text=WaterLeakDetection%20BATTERY%20LOW&type=direct&from=sms77.de&return_msg_id=1";
      }
      if (MailType == MAIL_WELCOME)
      { // send the HTTP PUT request:
        url = "/api/sms?u=" + SmsUser + "&p=" + SmsApiKey + " &to=" + SmsNr + "&text=WaterLeakDetection%20WELCOME&type=direct&from=sms77.de&return_msg_id=1";
      }
      Serial.print("requesting URL: ");
      Serial.println(url);

      String line = "";
      client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                   "Host: " + SmsServer + "\r\n" +
                   "User-Agent: WaterLeakDetection\r\n" +
                   "Connection: close\r\n\r\n");

      Serial.println("request sent");
      while (client.connected()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") {
          Serial.println("headers received");
          break;
        }
      }
      Serial.print("reply was:");
      Serial.println(line);
      Serial.println("closing connection");
      Serial.println("SMS Server request send");
    }
    else
    { // if you couldn't make a connection:
      Serial.println("ERR: SMS Server not connected");
      return false;
    }
    client.stop();
    return true;
  }
  return true;
}

boolean SendToThingSpeak()
{
  if (client.connect(ThinkSpeakServer, 80)) { //   "184.106.153.149" or api.thingspeak.com
    String postStr = apiKey;
    postStr += "&field1=";
    postStr += U_Bat;
    postStr += "&field2=";
    postStr += pinState;
    postStr += "&field3=";
    postStr += sleepCount;
    postStr += "\r\n\r\n";

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);
    client.stop();
    Serial.println("Daten zu ThingSpeak gesendet");
    Serial.print("DatenString: ");
    Serial.println(postStr);
  }
  else
  { // Connection Failed
    Serial.print("ERR: ThinkSpeak failed.");
    return false;
  }
  client.stop();
  return true;
}

double ReadBatVoltage()
{
  double Volt_Bat;
  pinMode(A0, INPUT);   // Eingang fÃ¼r Analogspannung
  delay(1000);

  long int wert = 0;
  for (int i = 1; i <= 10; i++)
  { wert += analogRead(A0);
    delay(200);
  }
  wert = wert / 10;

  // Widerstandsverhältnis: 10/(10+39) = 10/49
  Volt_Bat = ((double) wert) / 208.9796;
  Serial.print("Bat. Spg.: "); Serial.println(Volt_Bat);
  return Volt_Bat;
}

boolean WiFiConnect()
{
  uint32_t time1;
  time1 = system_get_time();

  // Connect to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (((system_get_time() - time1) / 1000000) > WIFI_CONNECT_TIMEOUT_S)  // wifi connection lasts too ling, retry
    {
      Serial.println("timeout: no connection");
      return false;
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  return true;
}

void setup()
{
  uint32_t time3;
  time3 = millis();
  bool StatusSms = false;

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("WaterLeakDetection V6 .... Start SETUP");

  // Prüfung ob Wasser Alarm
  Serial.println("Pruefung auf Wasser");
  pinMode(ALARM_PIN, INPUT);
  pinState = digitalRead(ALARM_PIN);  // read the alarm pin to find out whether an alarm is teh reset cause
  Serial.print("Wasser PIN: ");
  Serial.println(pinState);

  // Prüfung der Batteriespannung
  Serial.println("Pruefung der Batteriespannung");
  U_Bat = ReadBatVoltage();
  Serial.print("Spg.: " );
  Serial.println(U_Bat);

  Serial.println("Read RTC data");
  system_rtc_mem_read(RTC_BASE, buf, 2); // read 2 bytes from RTC-MEMORY
  system_rtc_mem_read(RTC_BAT_STATE, &BatState, 1); // read 1 bytes from RTC-MEMORY
  system_rtc_mem_read(RTC_ALARM_STATE, &AlarmState, 1); // read 1 bytes from RTC-MEMORY

  Serial.print("Bat State: ");
  Serial.println(BatState);
  Serial.print("Alarm State: ");
  Serial.println(AlarmState);
w
  if ((buf[0] != 0x55) || (buf[1] != 0xaa))   // Cold Start
  {
    Serial.println("Cold Start");
    buf[0] = 0x55; buf[1] = 0xaa;
    system_rtc_mem_write(RTC_BASE, buf, 2);
    sleepCount = 0;
    system_rtc_mem_write(RTC_WAKE_COUNT, &sleepCount, 4); // initialize counter
    BatState = 0;
    AlarmState = 0;
    ColdStart = true;
  }
  else // reset was due to sleep-wake or external event
  { // RegulÃ¤rer zyklischer Aufruf
    system_rtc_mem_read(RTC_WAKE_COUNT, &sleepCount, 4); // read counter
    sleepCount++;
    Serial.print(sleepCount);
    Serial.println(". zyklischer Aufruf");

    // no special event, go to sleep again
    system_rtc_mem_write(RTC_WAKE_COUNT, &sleepCount, 4); // write counter
    ColdStart = 0;
  }

  if (U_Bat <= BATT_WARNING_VOLTAGE)
  { // this is an battery alarm!
    Serial.println("Batteriespannung zur gering!!");
    BatState = BatState | 1;
    Str_StatBatLow = "Batteriespannung LOW";
  }
  else {
    Str_StatBatLow = "Batteriespannung OK";
  }

  if (pinState == true)
  {
    Serial.println("ALERT: Water Alarm!!!");
    AlarmState = AlarmState | 1;
  }

  bool WifiCon = WiFiConnect();
  if (WifiCon == true)
  {
    // Connect to MQTT Broker
    Serial.println("Connect to MQTT Broker ...");
    MqttClient.setServer(mqtt_server, 8883);
    if (!MqttClient.connected())
    {
      MqttReconnect();
    }

    MqttClient.loop();
    delay(1000);

    String StrValue;
    StrValue = String(U_Bat);
    Serial.print("Publish message: ");
    Serial.println(StrValue.c_str());
    MqttClient.publish("BatVolt", StrValue.c_str(),true);

    if (pinState == 1) {
      Str_StatWaterLeak = "Wasser";
    }
    else {
      Str_StatWaterLeak = "kein Wasser";
    }

    Serial.print("Publish message: ");
    Serial.println(Str_StatWaterLeak);
    MqttClient.publish("LeakStat", Str_StatWaterLeak.c_str(),true);

    Serial.print("Publish message: ");
    Serial.println(Str_StatBatLow);
    MqttClient.publish("BatStat", Str_StatBatLow.c_str(),true);

    String StrLifeCnt;
    StrLifeCnt = String(sleepCount);
    Serial.print("Publish message: ");
    Serial.println(StrLifeCnt.c_str());
    MqttClient.publish("LifeCnt", StrLifeCnt.c_str(),true);

    if (ColdStart == 1) // Cold Start
    {
      StatusSms = SendSms(MAIL_WELCOME);
    }
    else
    {
      // Email Senden:
      if ((AlarmState & 255) == 1)        // Wasser Alarm
      {
        StatusSms = SendSms(MAIL_ALARM);
        if (StatusSms == 1)
        {
          AlarmState = AlarmState | 2;
        }
      }

      if ((BatState  & 255) == 1)    // Batterie leer
      {
        StatusSms = SendSms(MAIL_LOW_BAT);
        if (StatusSms == 1)
        {
          BatState = BatState | 2;
        }
      }
    }
    Serial.println("Daten zu ThingSpeak senden");
    SendToThingSpeak();
  }
  system_rtc_mem_write(RTC_BAT_STATE, &BatState, 1); // initialize counter
  system_rtc_mem_write(RTC_ALARM_STATE, &AlarmState, 1); // initialize counter

  //Serial.println("Going to sleep ...");
  // Dauer des Aufrufs in millisekunden
  time3 = millis() - time3;
  ESP.deepSleep(SLEEP_TIME - time3 * 1000, WAKE_RFCAL);
  delay(200);
  yield();
}

void loop()
{
  delay(100);
}

