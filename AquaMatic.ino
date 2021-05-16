#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <time.h>
#include <coredecls.h>
#include <EEPROM.h>
#include <FS.h>   // SPIFFS library
#include <ArduinoJson.h>

// WiFiManger dependences
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal

extern "C" {
  #include "pwm.h"
  #include "psychrolib.h"
}

#define LED_PIN 13
#define FAN_PWM_PIN 0
#define FAN_GATE_PIN 15
#define WIFI_RESET_TRIGGER 14

// #define DEBUG
#ifdef DEBUG
 #define DEBUG_PRINT(x)    Serial.print (x)
 #define DEBUG_PRINTDEC(x) Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)
#endif

#define PWM_CHANNELS        2
#define PERIOD          10000 // * 200ns ^= 1 kHz
#define LED_PWM_REGISTER    0
#define FAN_PWM_REGISTER    1
#define LED_MAX_LOAD      100
#define LED_MAX_PERIOD PERIOD
#define FAN_MAX_LOAD      100
#define FAN_MAX_PERIOD   3000

Adafruit_BME280 bme;
double airTemp, airHumi, airPres, airWetb, wtrTemp;
unsigned long wtrTempSampleCount = 0;

#define ACQUISITION_INTERVAL  300000
#define LED_CONTROL_INTERVAL   10000
#define LED_RAMPING_INTERVAL    1000
unsigned long previousLedTimestamp = 0;
unsigned long previousRampTimestamp = 0;
unsigned long previousAcquisitionTimestamp = -ACQUISITION_INTERVAL;
bool initialRampDone = false;

#define DATA_ARRAY_LENGTH 288
typedef struct {
  short airTemp[DATA_ARRAY_LENGTH];
  short airHumi[DATA_ARRAY_LENGTH];
  short airPres[DATA_ARRAY_LENGTH];
  short wtrTemp[DATA_ARRAY_LENGTH];
  short fanLoad[DATA_ARRAY_LENGTH];
  short index;
} logsStruct;
logsStruct logs;

#define LED_SCHEDULE_LENGTH 22
typedef struct {
  int time;
  short load;
} scheduleStruct;
scheduleStruct ledSchedule[LED_SCHEDULE_LENGTH];
char ledScheduleIndex = 0;

struct statusStruct {
  short load;
  short targetLoad;
  float period;
  float targetPeriod;
  float normalizedPeriod;
  float rampStep;
  short rampTime;
  bool autoMode;
  bool ramping;
};
statusStruct led = { 0, 0, 0, 0, 0, 0, 300, true, true };
statusStruct fan = { 0, 0, 0, 0, 0, 0,  60, true, true };

WiFiEventHandler gotIpEventHandler, disconnectedEventHandler, apUpHandler, apDownHandler;

ESP8266WebServer httpServer(80);
String content;
unsigned char statusCode;

time_t tnow;

#define WIFI_MODE_ADDRESS 3
#define LED_SCHEDULE_ADDRESS 4
//#define AIR_TEMPERATURE_OFFSET LED_SCHEDULE_ADDRESS + sizeof(ledSchedule)
//#define WATER_TEMPERATURE_OFFSET AIR_TEMPERATURE_OFFSET + sizeof(float)
//#define next WATER_TEMPERATURE_OFFSET + sizeof(float)

#define OK_MESSAGE "{\"ErrMsg\":\"OK\"}"

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(FAN_GATE_PIN, OUTPUT);
  pinMode(FAN_GATE_PIN, OUTPUT);

  // StefanBruens/ESP8266_new_pwm config
  uint32 pwm_duty_init[PWM_CHANNELS] = {0, 0}; //initial duty cycle values (all off)
  uint32 io_info[PWM_CHANNELS][3] = {
    {PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13, LED_PIN},
    {PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0, FAN_PWM_PIN}
  };
  pwm_init(PERIOD, pwm_duty_init, PWM_CHANNELS, io_info);
  pwm_start();

  // psychrolib config;
  SetUnitSystem(SI);

  Serial.begin(115200);
  DEBUG_PRINTLN("\n\n\n\n\nBoot.");

  if ( digitalRead(WIFI_RESET_TRIGGER) == LOW || WiFi.SSID() == "" ) {
    WiFiManager wifiManager;
    wifiManager.startConfigPortal("AquaMatic", "AquaMatic8266");
  } else {
    WiFi.begin();
    DEBUG_PRINTLN("Connecting to: " + WiFi.SSID());
  }

  bme.begin(0x76);
  DEBUG_PRINT("BME280 ID: ");
  DEBUG_PRINTLN(bme.sensorID());

  EEPROM.begin(1024);
  EEPROM.get(LED_SCHEDULE_ADDRESS, ledSchedule);

  ledScheduleIndex = getLedScheduleIndex();

  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP & event) {
    DEBUG_PRINT("Station connected, IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
    ArduinoOTA.begin();
    httpServer.begin();
  });
  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected & event) {
    DEBUG_PRINTLN("Station disconnected");
  });

  SPIFFS.begin();
  httpServer.on("/api/led-schedule", handleLED_schedule);
  httpServer.on("/api/led", handleLED);
  httpServer.on("/api/fan", handleFan);
  httpServer.on("/api/log", handleLog);
  httpServer.onNotFound([]() {
    if (!handleFileRead(httpServer.uri()))
      httpServer.send(404, "text/plain", "404: Not Found.");
  });

  ArduinoOTA.setHostname("AquaMatic");
  ArduinoOTA.onStart([]() {
    String type = ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem";
    DEBUG_PRINTLN("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_PRINTLN("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    DEBUG_PRINT("Progress: ");
    DEBUG_PRINT(progress / (total / 100));
    DEBUG_PRINT("%%\r");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    // Serial.printf("Error[%u]: ", error);
    DEBUG_PRINT("Error ");
    DEBUG_PRINT(error);
    DEBUG_PRINT(" ");
    if (error == OTA_AUTH_ERROR) {
      DEBUG_PRINTLN("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      DEBUG_PRINTLN("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      DEBUG_PRINTLN("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      DEBUG_PRINTLN("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      DEBUG_PRINTLN("End Failed");
    }
  });

  configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  settimeofday_cb(initialRamp); // time update callback (update every hour)
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
    httpServer.handleClient();
  }

  if (time(nullptr) < 1.6e9) {
    delay(500);
    return;
  }

  if (millis() - previousRampTimestamp > LED_RAMPING_INTERVAL) {
    previousRampTimestamp += LED_RAMPING_INTERVAL;
    if(led.ramping) rampLED();

    wtrTemp += AnalogTemp();
    wtrTempSampleCount++;
  }

  if (millis() - previousLedTimestamp > LED_CONTROL_INTERVAL) {
    previousLedTimestamp += LED_CONTROL_INTERVAL;

    if (led.ramping == false & led.autoMode == true) ledScheduleControl();
  }

  if (millis() - previousAcquisitionTimestamp > ACQUISITION_INTERVAL) {
    previousAcquisitionTimestamp += ACQUISITION_INTERVAL;

    acquisiteData();

    tempControl(airWetb, wtrTemp);
    wtrTemp = 0;
  }

  delay(50);
}

String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".pdf")) return "application/x-pdf";
  else if (filename.endsWith(".zip")) return "application/x-zip";
  else if (filename.endsWith(".png")) return "image/png";
  return "text/plain";
}
bool handleFileRead(String path) {
  DEBUG_PRINTLN("handleFileRead: " + path);
  if (path.endsWith("/"))
    path += "index.html";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
    if (SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = httpServer.streamFile(file, contentType);
    file.close();
    return true;
  }
  DEBUG_PRINTLN("\t File Not Found.");
  return false;
}

void acquisiteData() {
    airTemp = bme.readTemperature();
    airHumi = bme.readHumidity();
    airPres = bme.readPressure();
    airWetb = GetTWetBulbFromRelHum(airTemp, airHumi / 100, airPres);
    wtrTemp /= wtrTempSampleCount;
    wtrTempSampleCount = 0;

    tnow = time(nullptr) + 8 * 3600;

    logs.index = ++logs.index == DATA_ARRAY_LENGTH ? 0 : logs.index;
    logs.airTemp[logs.index] = (short) round(airTemp * 10);
    logs.airHumi[logs.index] = (short) round(airHumi * 10);
    logs.wtrTemp[logs.index] = (short) round(wtrTemp * 10);
    logs.airPres[logs.index] = (short) round(airPres - 80000);
    logs.fanLoad[logs.index] = (short) round(fan.load * 10);
}
double AnalogTemp() {
  double V_ADC = analogRead(A0);

  double Vcc = 3.3, Rs = 41300, R_0 = 10e3, T_0 = 298.15, B = 3435;
  double V_NTC = V_ADC / 1024;
  double R_NTC = (Rs * V_NTC) / (Vcc - V_NTC);
  double r_inf = R_0 * exp(-B / T_0);
  double Temp_B = B / log(R_NTC/r_inf) - 273.15;

  return Temp_B;
}
void tempControl(float wetBulbTemp, float waterTemp) {
  // DEBUG_PRINTLN("Temperautre Control.");
}
void fanPID() {
  // DEBUG_PRINTLN("fanPID.");
}
short ledPeriodNormalizer(float period) { 
  // return period * period / PERIOD;
  return period > 1000 ? period * period / PERIOD : period / 10;
}
float ledLoad2PeriodMapping(short load) {
  return PERIOD / LED_MAX_LOAD * load;
}
short ledPeriod2LoadMapping(float period) {
  return period / PERIOD * LED_MAX_LOAD;
}
void ledScheduleControl() {
  time_t tcurrent = time(nullptr) + 8 * 3600;
  int secondOfDay = tcurrent % 86400;
  if (secondOfDay < ledSchedule[ledScheduleIndex - 1].time || ledSchedule[ledScheduleIndex].time < secondOfDay)
    ledScheduleIndex = getLedScheduleIndex();

  float y0 = ledSchedule[ledScheduleIndex - 1].load;
  float y1 = ledSchedule[ledScheduleIndex].load;
  // short circuit if load is satisfied.
  if (y1 == led.period * LED_MAX_LOAD / LED_MAX_PERIOD) return;

  float t0 = ledSchedule[ledScheduleIndex - 1].time;
  float t1 = ledSchedule[ledScheduleIndex].time;
  float load = (secondOfDay - t0) / (t1 - t0) * (y1 - y0) + y0;

  float period = ledLoad2PeriodMapping(load);
  short normalizedPeriod = ledPeriodNormalizer(period);
  pwm_set_duty(normalizedPeriod, LED_PWM_REGISTER);
  pwm_start();           // commit

  led.load = load;
  led.targetLoad = y1;
  led.period = period;
  led.targetPeriod = ledLoad2PeriodMapping(y1);
  led.normalizedPeriod = normalizedPeriod;
  led.rampStep = (led.targetPeriod - led.period) / ((t1 - secondOfDay) / (LED_RAMPING_INTERVAL / 1000));

  if(led.rampStep != 0) led.ramping = true;
}
char getLedScheduleIndex() {
  time_t tcurrent = time(nullptr) + 8 * 3600;
  int secondOfDay = tcurrent % 86400;
  for (char i = 1; i < LED_SCHEDULE_LENGTH; i++)
    if (ledSchedule[i - 1].time <= secondOfDay & secondOfDay < ledSchedule[i].time)
      return i;
}
float interpolateLoad(int targetTime, char index) {
  float t0 = ledSchedule[index - 1].time;
  float t1 = ledSchedule[index].time;
  float y0 = ledSchedule[index - 1].load;
  float y1 = ledSchedule[index].load;
  return (targetTime - t0) / (t1 - t0) * (y1 - y0) + y0;
}
void handleLED_schedule() {
  if (httpServer.method() == HTTP_POST) {
    StaticJsonDocument<1040> doc;
    deserializeJson(doc, httpServer.arg("plain"));
    // fill ledSchedule with schedule settings starting from ledSchedule[i=1]
    for (char i = 0; i < doc.size(); i++)
      ledSchedule[i + 1] = { doc[i]["t"].as<int>(), doc[i]["l"].as<short>() };
    // fill the rest of ledSchedule with -1
    for (char i = doc.size(); i < LED_SCHEDULE_LENGTH; i++)
      ledSchedule[i + 1] = { -1, -1 };

    // ledSchedule[0] is a special key event,
    // its event time should always be 0,
    // its vulue is the interpolation of schedule[1] and schedule[last],
    // it is used to make the schedule work at a 24 hours periodic style.
    ledSchedule[0].time = 0;
    float x0 = ledSchedule[1].time;
    float y0 = ledSchedule[1].load;
    float x1 = ledSchedule[doc.size()].time;
    float y1 = ledSchedule[doc.size()].load;
    ledSchedule[0].load = ((0 - y0) / (x1 - (86400 + x0)) * (y1 - y0) + y0);

    // if schedule[last].time!=1440 (last minute of day) then
    // we should add a key event at time=1440 for the sake of a full 24 hours schedule chart
    if (ledSchedule[doc.size()].time != 86400) { 
      ledSchedule[doc.size() + 1].time = 86400;
      ledSchedule[doc.size() + 1].load = ledSchedule[0].load;
    }
    EEPROM.put(LED_SCHEDULE_ADDRESS, ledSchedule);
    EEPROM.commit();

    content = OK_MESSAGE;
  }
  else {
    content = "{\"ErrMsg\":\"OK\",\"Data\":[";
    // if schedule[1].time==0 then send schedule from schedule[1] else from schedule[0]
    for ( short i = ledSchedule[1].time==0 ? 1 : 0; i < LED_SCHEDULE_LENGTH; i++) {
      content += "{\"t\":" + (String) ledSchedule[i].time + ",\"l\":" + (String) ledSchedule[i].load + "}";
      if (ledSchedule[i].time > ledSchedule[i+1].time) {
        content += "]}";
        break;
      }
      content += ",";
    }
  }

  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", content);
}
void rampLED() {
  led.period += led.rampStep;

  if((led.rampStep > 0 & led.period > led.targetPeriod) |
      (led.rampStep < 0 & led.period < led.targetPeriod)) {
    led.period = led.targetPeriod;
    led.ramping = false;
  }
  
  short normalizedPeriod = ledPeriodNormalizer(led.period);
  pwm_set_duty(normalizedPeriod, LED_PWM_REGISTER);
  pwm_start();           // commit
  
  led.load = ledPeriod2LoadMapping(led.period);
  led.normalizedPeriod = normalizedPeriod;
}
void initialRamp() {
  if (initialRampDone == false) rampLEDToScheduleLoad();
  initialRampDone = true;
}
void rampLEDToScheduleLoad() {
  time_t tcurrent = time(nullptr) + 8 * 3600;
  int secondOfDay = tcurrent % 86400;
  ledScheduleIndex = getLedScheduleIndex();

  float t0 = ledSchedule[ledScheduleIndex - 1].time;
  float t1 = ledSchedule[ledScheduleIndex].time;
  float y0 = ledSchedule[ledScheduleIndex - 1].load;
  float y1 = ledSchedule[ledScheduleIndex].load;
  float target = (secondOfDay + led.rampTime - t0) / (t1 - t0) * (y1 - y0) + y0;

  led.targetLoad = target;
  led.targetPeriod = ledLoad2PeriodMapping(target);
  led.rampStep = (led.targetPeriod - led.period) / (led.rampTime / (LED_RAMPING_INTERVAL / 1000));
  led.autoMode = true;
  led.ramping = true;
  
  rampLED();
}
void deviceStatusString(statusStruct device, String *status) {
    *status = "{\"ErrMsg\":\"OK\",\"Data\":{\"load\":" + (String)device.load;
    *status += ",\"targetLoad\":" + (String)device.targetLoad;
    *status += ",\"period\":" + (String)device.period;
    *status += ",\"targetPeriod\":" + (String)device.targetPeriod;
    *status += ",\"normalizedPeriod\":" + (String)device.normalizedPeriod;
    *status += ",\"rampStep\":" + (String)device.rampStep;
    *status += ",\"rampTime\":" + (String)device.rampTime;
    *status += ",\"autoMode\":" + (String)device.autoMode;
    *status += ",\"ramping\":" + (String)device.ramping + "}}";
}
void handleLED() {
  if (httpServer.hasArg("val")) {
    short target = httpServer.arg("val").toInt();

    led.targetLoad = target < 0 ? 0 : (target > LED_MAX_LOAD ? LED_MAX_LOAD : target);
    led.targetPeriod = ledLoad2PeriodMapping(led.targetLoad);
    led.rampStep = (led.targetPeriod - led.period) / (led.rampTime / (LED_RAMPING_INTERVAL / 1000));
    led.autoMode = false;
    led.ramping = true;

    rampLED();

    content = OK_MESSAGE;
  }
  else if (httpServer.hasArg("mode")) {
    led.autoMode = httpServer.arg("mode") == "auto" ? true : false;
    if (led.autoMode) rampLEDToScheduleLoad();

    content = OK_MESSAGE;
  }
  else {
    // content = "{\"ErrMsg\":\"OK\",\"Data\":{\"load\":" + (String)led.load;
    // content += ",\"targetLoad\":" + (String)led.targetLoad;
    // content += ",\"period\":" + (String)led.period;
    // content += ",\"targetPeriod\":" + (String)led.targetPeriod;
    // content += ",\"normalizedPeriod\":" + (String)led.normalizedPeriod;
    // content += ",\"rampStep\":" + (String)led.rampStep;
    // content += ",\"rampTime\":" + (String)led.rampTime;
    // content += ",\"autoMode\":" + (String)led.autoMode;
    // content += ",\"ramping\":" + (String)led.ramping + "}}";
    deviceStatusString(led, &content);
  }

  statusCode = 200;
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(statusCode, "application/json", content);
}
void handleFan() {
  statusCode = 200;
  if (httpServer.hasArg("val")) {
    fanOutput(httpServer.arg("val").toInt());
    content = OK_MESSAGE;
  } else
    // content = "{\"ErrMsg\":\"OK\",\"Data\":{\"load\":" + (String) fan.load + "}}";
    deviceStatusString(fan, &content);

  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(statusCode, "application/json", content);
}
void fanOutput(short load) {
  load = load < 0 ? 0 : (load > FAN_MAX_LOAD ? FAN_MAX_LOAD : load);
  short regulatedLoad = load * FAN_MAX_PERIOD / FAN_MAX_LOAD;
  pwm_set_duty(regulatedLoad, FAN_PWM_REGISTER);
  pwm_start();           // commit
  digitalWrite(FAN_GATE_PIN, load > 0 ? HIGH : LOW);

  fan.load = load;
  fan.targetLoad = load;
  fan.period = regulatedLoad;
  fan.targetPeriod = regulatedLoad;
  fan.normalizedPeriod = regulatedLoad;
}
void handleLog() {
  DEBUG_PRINTLN("handle log");
  content = "{\"ErrMsg\":\"OK\",\"Data\":{\"airHumi\":[";
  for (short i = 0; i < DATA_ARRAY_LENGTH; i++) {
    content += (String) logs.airHumi[i];
    content += i < DATA_ARRAY_LENGTH - 1 ? "," : "],\"airTemp\":[";
  }
  for (short i = 0; i < DATA_ARRAY_LENGTH; i++) {
    content += (String) logs.airTemp[i];
    content += i < DATA_ARRAY_LENGTH - 1 ? "," : "],\"airPres\":[";
  }
  for (short i = 0; i < DATA_ARRAY_LENGTH; i++) {
    content += (String) logs.airPres[i];
    content += i < DATA_ARRAY_LENGTH - 1 ? "," : "],\"wtrTemp\":[";
  }
  for (short i = 0; i < DATA_ARRAY_LENGTH; i++) {
    content += (String) logs.wtrTemp[i];
    content += i < DATA_ARRAY_LENGTH - 1 ? "," : "],\"fanLoad\":[";
  }
  for (short i = 0; i < DATA_ARRAY_LENGTH; i++) {
    content += (String) logs.fanLoad[i];
    content += i < DATA_ARRAY_LENGTH - 1 ? "," : "],\"index\":";
  }
  content += (String) logs.index + "}}";

  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", content);

  content = "";
}
