#include <LittleFS.h>                   //this needs to be first, or it all crashes and burns...
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <Roomba.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Timezone.h>
#include "config.h"
extern "C" {
#include "user_interface.h"
}
// WiFi Setup 
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager

// Remote debugging over telnet. Just run:
// `telnet roomba.local` OR `nc roomba.local 23`
#if LOGGING
#include <RemoteDebug.h>
#define DLOG(msg, ...)                \
  if (Debug.isActive(Debug.DEBUG))    \
  {                                   \
    Debug.printf(msg, ##__VA_ARGS__); \
  }
#define VLOG(msg, ...)                \
  if (Debug.isActive(Debug.VERBOSE))  \
  {                                   \
    Debug.printf(msg, ##__VA_ARGS__); \
  }
RemoteDebug Debug;
#else
#define DLOG(msg, ...)
#endif

//config setup
char MQTT_SERVER[16];
char MQTT_PORT[6] = "1883";
char MQTT_USER[40];
char MQTT_PASSWORD[40];


// Roomba setup
Roomba roomba(&Serial, Roomba::Baud115200);

// Roomba state
typedef struct
{
  // Sensor values
  int16_t distance;
  uint8_t  chargingState;
  uint16_t voltage;
  int16_t  current;
  int8_t   temperature;
  // Supposedly unsigned according to the OI docs, but I've seen it
  // underflow to ~65000mAh, so I think signed will work better.
  uint16_t charge;
  uint16_t capacity;
  uint8_t  charge_source;
  uint8_t  stasis;
  // Derived state
  bool cleaning = false;
  bool docked = false;
  uint16_t timestamp;
  bool sent;
} RoombaState;

RoombaState roombaState = {};

// Roomba sensor packet
// uint8_t roombaPacket[100];
// structured list with sensorID, nBytes, sign (1=signed/0=unsigned)
// sample entry: 
// 21,2,1,
// 24,3,0
uint8_t sensor_list[] = {
  Roomba::SensorChargingState,1,0,              // PID 21, 1 byte, unsigned
  Roomba::SensorVoltage,2,0,                    // PID 22, 2 bytes, mV, unsigned
  Roomba::SensorCurrent,2,1,                    // PID 23, 2 bytes, mA, signed
  Roomba::SensorBatteryTemperature,1,1,         // PID 24, 1 byte, Celsius, signed
  Roomba::SensorBatteryCharge,2,0,              // PID 25, 2 bytes, mAh, unsigned
  Roomba::SensorBatteryCapacity,2,0,            // PID 26, 2 bytes, mAh, unsigned
  Roomba::SensorChargingSourcesAvailable,1,0,    // PID 34, 1 byte, code, unsigned
  Roomba::SensorStatisWheel,58,1,0              // PID 58, 1 byte, code, unsigned
  };


// Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120}; // Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};   // Central European Standard Time
Timezone tz(CEST, CET);

// Network setup
WiFiClient wifiClient;
bool OTAStarted;

// MQTT setup
PubSubClient mqttClient(wifiClient);
const char *commandTopic = MQTT_COMMAND_TOPIC;
const char *stateTopic = MQTT_STATE_TOPIC;
const char *configTopic = MQTT_CONFIG_TOPIC;

void wakeup()
{
  DLOG("Wakeup Roomba\n");
  pinMode(BRC_PIN, OUTPUT);
  digitalWrite(BRC_PIN, LOW);
  delay(200);
  pinMode(BRC_PIN, INPUT);
  delay(200);
  roomba.start();
}

void stopCleaning()
{
  if (roombaState.cleaning) {
      DLOG("Stopping\n");
      roomba.cover();
  } else  {
    DLOG("Not cleaning, can't stop\n");
  }
}

bool performCommand(const char *cmdchar)
{
  wakeup();
  delay(1000);

  // Char* string comparisons dont always work
  String cmd(cmdchar);

  // MQTT protocol commands
  if (cmd == "turn_on" || cmd == "start")
  {
    if (!roombaState.cleaning) {
      DLOG("Start cleaning\n");
      roomba.cover();
      roombaState.cleaning = true;
    } else {
      DLOG("Already cleaning!\n");
    }
  }
  else if (cmd == "turn_off")
  {
    DLOG("Turning off\n");
    roomba.power();
    roombaState.cleaning = false;
  }
  else if (cmd == "stop" || cmd == "pause")
  {
    stopCleaning();
  }
  else if (cmd == "clean_spot")
  {
    stopCleaning();
    delay(1000);
    DLOG("Cleaning Spot\n");
    roomba.spot();
    roombaState.cleaning = true;
  }
  else if (cmd == "locate")
  {
    DLOG("Playing song #0\n");
    roomba.safeMode();
    delay(50);
    roomba.playSong(0);
    delay(4000);
    roomba.playSong(1);
    delay(4000);
    roomba.playSong(2);
    delay(3500);
    roomba.playSong(3);
  }
  else if (cmd == "return_to_base")
  {
    stopCleaning();
    delay(1000);
    DLOG("Returning to Base\n");
    roombaState.cleaning = true;
    roomba.dock();
  }
  else
  {
    return false;
  }
  return true;
}

char *getMAC(const char *divider = "")
{
  byte MAC[6];
  WiFi.macAddress(MAC);
  static char MACc[30];
  sprintf(MACc, "%02X%s%02X%s%02X%s%02X%s%02X%s%02X", MAC[0], divider, MAC[1], divider, MAC[2], divider, MAC[3], divider, MAC[4], divider, MAC[5]);
  return strlwr(MACc);
}

char *getEntityID()
{
  char entityID[100];
  sprintf(entityID, "%s%s", MQTT_IDPREFIX, getMAC());
  // avoid confusions with lower/upper case differences in IDs
  return strlwr(entityID);
}

char *getMQTTTopic(const char *topic)
{
  // build mqtt target topic
  static char mqttTopic[200];
  sprintf(mqttTopic, "%s%s%s%s", MQTT_TOPIC_BASE, getEntityID(), MQTT_DIVIDER, topic);
  return mqttTopic;
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  DLOG("Received mqtt callback for topic %s\n", topic);
  if (strcmp(getMQTTTopic(commandTopic), topic) == 0)
  {
    // turn payload into a null terminated string
    char *cmd = (char *)malloc(length + 1);
    memcpy(cmd, payload, length);
    cmd[length] = 0;

    if (!performCommand(cmd))
    {
      DLOG("Unknown command %s\n", cmd);
    }
    free(cmd);
  }
}

float readADC(int samples)
{
  // Basic code to read from the ADC
  int adc = 0;
  for (int i = 0; i < samples; i++)
  {
    delay(1);
    adc += analogRead(A0);
  }
  adc = adc / samples;
  float mV = adc * ADC_VOLTAGE_DIVIDER;
  DLOG("ADC for %d is %.1fmV with %d samples\n", adc, mV, samples);
  return mV;
}

void setDateTime()
{
  DLOG("Setting Roomba time and date\n");
  configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2)
  {
    delay(500);
    now = time(nullptr);
  }
  time_t local = tz.toLocal(now);
  roomba.setDayTime(dayOfWeek(local) - 1, hour(local), minute(local));
}

void debugCallback()
{
  String cmd = Debug.getLastCommand();

  // Debugging commands via telnet
  if (performCommand(cmd.c_str()))
  {
  }
  else if (cmd == "quit")
  {
    DLOG("Stopping Roomba\n");
    Serial.write(173);
  }
  else if (cmd == "rreset")
  {
    DLOG("Resetting Roomba\n");
    roomba.reset();
  }
  else if (cmd == "mqtthello")
  {
    mqttClient.publish("vacuum/hello", "hello there");
  }
  else if (cmd == "version")
  {
    const char compile_date[] = __DATE__ " " __TIME__;
    DLOG("Compiled on: %s\n", compile_date);
  }
  else if (cmd == "baud115200")
  {
    DLOG("Setting baud to 115200\n");
    Serial.begin(115200);
    delay(100);
  }
  else if (cmd == "baud19200")
  {
    DLOG("Setting baud to 19200\n");
    Serial.begin(19200);
    delay(100);
  }
  else if (cmd == "baud57600")
  {
    DLOG("Setting baud to 57600\n");
    Serial.begin(57600);
    delay(100);
  }
  else if (cmd == "baud38400")
  {
    DLOG("Setting baud to 38400\n");
    Serial.begin(38400);
    delay(100);
  }
  else if (cmd == "sleep5")
  {
    DLOG("Going to sleep for 5 seconds\n");
    delay(100);
    ESP.deepSleep(5e6);
  }
  else if (cmd == "wake")
  {
    DLOG("Toggle BRC pin\n");
    wakeup();
  }
  else if (cmd == "readadc")
  {
    float adc = readADC(10);
    DLOG("ADC voltage is %.1fmV\n", adc);
  }
  else if (cmd == "streamresume")
  {
    DLOG("Resume streaming\n");
    roomba.streamCommand(Roomba::StreamCommandResume);
  }
  else if (cmd == "streampause")
  {
    DLOG("Pause streaming\n");
    roomba.streamCommand(Roomba::StreamCommandPause);
  }
  /*
  else if (cmd == "stream")
  {
    DLOG("Requesting stream\n");
    roomba.stream(sensors, sizeof(sensors));
  }
  else if (cmd == "streamreset")
  {
    DLOG("Resetting stream\n");
    roomba.stream({}, 0);
  }
  */
  else if (cmd == "time")
  {
    setDateTime();
  }
  else if (cmd == "sense")
  {
    DLOG("Request Sensor\n")
    uint8_t dest[1];
    bool data = roomba.getSensors(Roomba::SensorCliffLeft,dest, 1);

    if(data) {
      DLOG("%d ", dest[0]);
    }

  }
  else
  {
    DLOG("Unknown command %s\n", cmd.c_str());
  }
}

void sleepIfNecessary()
{
#if ENABLE_ADC_SLEEP
  // Check the battery, if it's too low, sleep the ESP (so we don't murder the battery)
  float mV = readADC(10);
  // According to this post, you want to stop using NiMH batteries at about 0.9V per cell
  // https://electronics.stackexchange.com/a/35879 For a 12 cell battery like is in the Roomba,
  // That's 10.8 volts.
  if (mV < 10800)
  {
    // Fire off a quick message with our most recent state, if MQTT is connected
    DLOG("Battery voltage is low (%.1fV). Sleeping for 10 minutes\n", mV / 1000);
    if (mqttClient.connected())
    {
      StaticJsonDocument<200> root;
      root["battery_level"] = 0;
      root["cleaning"] = false;
      root["docked"] = false;
      root["charging"] = false;
      root["voltage"] = mV / 1000;
      root["charge"] = 0;
      String jsonStr;
      serializeJson(root, jsonStr);
      mqttClient.publish(getMQTTTopic(stateTopic), jsonStr.c_str(), true);
    }
    delay(200);

    // Sleep for 10 minutes
    ESP.deepSleep(600e6);
  }
#endif
}
void readSensorPacket() {
  roombaState.timestamp = millis();
  uint8_t dest[10];
  uint i = 0;
  for (i = 0; i < sizeof(sensor_list); i += 3){
    DLOG("Request Sensor: %d\r\n", sensor_list[i]);
    bool received = roomba.getSensors(sensor_list[i], dest, sensor_list[i+1]);
    if (received){
      switch (sensor_list[i]){
        case Roomba::SensorChargingState:
          roombaState.chargingState = dest[0];
          DLOG("data: %u \r\n", roombaState.chargingState);
          break;
        case Roomba::SensorVoltage:
          roombaState.voltage = (256*dest[0] + dest[1]);
          DLOG("data: %d \r\n", roombaState.voltage);
          break;
        case Roomba::SensorCurrent:
          roombaState.current = (256*dest[0] + dest[1]);
          DLOG("data: %d \r\n", roombaState.current);
          break;
        case Roomba::SensorBatteryTemperature:
          roombaState.temperature = dest[0];
          DLOG("data: %d \r\n", roombaState.temperature);
          break;
        case Roomba::SensorBatteryCharge:
          roombaState.charge = (256*dest[0] + dest[1]);
          DLOG("data: %d \r\n", roombaState.charge);
          break;
        case Roomba::SensorBatteryCapacity:
          roombaState.capacity = (256*dest[0] + dest[1]);
          DLOG("data: %d \r\n", roombaState.capacity);
          break;
        case Roomba::SensorChargingSourcesAvailable:
          roombaState.charge_source = dest[0];
          DLOG("data: %u \r\n", roombaState.charge_source);
          break;
        case Roomba::SensorStatisWheel:
          roombaState.stasis = dest[0];
          DLOG("data: %u \r\n", roombaState.stasis);
          break;
        default:
          DLOG("Unsupported format at: %d \r\n", sensor_list[i]);
          break;
      }
    } else {
      DLOG("Unknown command or timeout occurred: %d\r\n", sensor_list[i]);
    }
    delay(4000);
  }
}

void onOTAStart()
{
  DLOG("Starting OTA session\n");
  DLOG("Pause streaming\n");
  roomba.streamCommand(Roomba::StreamCommandPause);
  OTAStarted = true;
}

//begin  added for wifiManager
//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setupLittleFS(){
  //clean FS, for testing
 //LittleFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (LittleFS.begin()) {
    Serial.println("mounted file system");
    if (LittleFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        StaticJsonDocument<256> json;
        DeserializationError jsonError = deserializeJson(json, buf.get());
        serializeJsonPretty(json, Serial);
        if (!jsonError) {
          Serial.println("\nparsed json");

          strcpy(MQTT_SERVER, json["mqtt_server"]);
          strcpy(MQTT_PORT, json["mqtt_port"]);
          strcpy(MQTT_USER, json["mqtt_user"]);
          strcpy(MQTT_PASSWORD, json["mqtt_password"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
}
// end added for wifi manager





void setup()
{
  Serial.begin(115200);
  Serial.println();

  // added for wifimanager config
  setupLittleFS();

  // WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;

  //set config save notify callback
  wm.setSaveConfigCallback(saveConfigCallback);

  // setup custom parameters
  // 
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", MQTT_SERVER, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", MQTT_PORT, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", MQTT_USER, 40);
  WiFiManagerParameter custom_mqtt_password("password", "mqtt password", MQTT_PASSWORD, 40);

  //add all your parameters here
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_password);

  if (!wm.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    // if we still have not connected restart and try all over again
    ESP.restart();
    delay(5000);
  }
  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(MQTT_SERVER, custom_mqtt_server.getValue());
  strcpy(MQTT_PORT, custom_mqtt_port.getValue());
  strcpy(MQTT_USER, custom_mqtt_user.getValue());
  strcpy(MQTT_PASSWORD, custom_mqtt_password.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    StaticJsonDocument<256> json;

    json["mqtt_server"]     = MQTT_SERVER;
    json["mqtt_port"]       = MQTT_PORT;
    json["mqtt_user"]       = MQTT_USER;
    json["mqtt_password"]   = MQTT_PASSWORD;

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    serializeJsonPretty(json, Serial);
    serializeJson(json, configFile);
    configFile.close();
    //end save
    shouldSaveConfig = false;
  }
/*
  Serial.println("==================================");
  Serial.println("IP Address:");
  Serial.println(WiFi.localIP());
  Serial.println("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.println("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.println("==================================");
  Serial.println("WiFi SSID: ");
  Serial.println(WiFi.SSID());
  Serial.println("==================================");
  Serial.println("MQTT Server: ");
  Serial.println(MQTT_SERVER);
  Serial.println("MQTT User: ");
  Serial.println(MQTT_USER);
  Serial.println("MQTT Port: ");
  Serial.println(MQTT_PORT);
  Serial.println("==================================");
*/
// end added for wifi manager config

  // High-impedence on the BRC_PIN
  pinMode(BRC_PIN, INPUT);


  // Sleep immediately if ENABLE_ADC_SLEEP and the battery is low
  sleepIfNecessary();

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  wifiManager.autoConnect();


  // Set Hostname.
  String hostname(HOSTNAME);
  WiFi.hostname((const char *)hostname.c_str());
  WiFi.begin();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();
  ArduinoOTA.onStart(onOTAStart);

  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT));
  mqttClient.setCallback(mqttCallback);

#if LOGGING
  Debug.begin((const char *)hostname.c_str());
  Debug.setResetCmdEnabled(true);
  Debug.setCallBackProjectCmds(debugCallback);
  Debug.setSerialEnabled(false);
#endif

  // Learn locate song
  roomba.safeMode();
  byte locateSong0[18] = {55, 32, 55, 32, 55, 32, 51, 24, 58, 8, 55, 32, 51, 24, 58, 8, 55, 64};
  byte locateSong1[18] = {62, 32, 62, 32, 62, 32, 63, 24, 58, 8, 54, 32, 51, 24, 58, 8, 55, 64};
  byte locateSong2[24] = {67, 32, 55, 24, 55, 8, 67, 32, 66, 24, 65, 8, 64, 8, 63, 8, 64, 16, 30, 16, 56, 16, 61, 32};
  byte locateSong3[28] = {60, 24, 59, 8, 58, 8, 57, 8, 58, 16, 10, 16, 52, 16, 54, 32, 51, 24, 58, 8, 55, 32, 51, 24, 58, 8, 55, 64};
  roomba.song(0, locateSong0, 18);
  roomba.song(1, locateSong1, 18);
  roomba.song(2, locateSong2, 24);
  roomba.song(3, locateSong3, 28);

  roomba.start();
  delay(100);
  // Reset stream sensor values
 // roomba.stream({}, 0);
 // delay(100);
  // Request sensor stream
 // roomba.stream(sensors, sizeof(sensors));

#if SET_DATETIME
  wakeup();
  // set time
  setDateTime();
#endif
}

void reconnect()
{
  DLOG("Attempting MQTT connection...\n");
  // Attempt to connect
  if (mqttClient.connect(HOSTNAME, MQTT_USER, MQTT_PASSWORD))
  {
    DLOG("MQTT connected\n");
    mqttClient.subscribe(getMQTTTopic(commandTopic));
  }
  else
  {
    DLOG("MQTT failed rc=%d try again in 5 seconds\n", mqttClient.state());
  }
}

void sendConfig()
{
  if (!mqttClient.connected())
  {
    DLOG("MQTT Disconnected, not sending config\n");
    return;
  } 
  StaticJsonDocument<500> root;
  root["name"] = String("Roomba ") + getMAC();
  root["unique_id"] = getEntityID();
  root["schema"] = "state";
  char baseTopic[200];
  sprintf(baseTopic, "%s%s", MQTT_TOPIC_BASE, getEntityID());
  root["~"] = baseTopic;
  root["stat_t"] = String("~/") + stateTopic;
  root["cmd_t"] = String("~/") + commandTopic;
  root["send_cmd_t"] = String("~/") + commandTopic;
  root["json_attr_t"] = String("~/") + stateTopic;
  root["sup_feat"][0] = "start";
  root["sup_feat"][1] = "stop";
  root["sup_feat"][2] = "pause";
  root["sup_feat"][3] = "return_home";
  root["sup_feat"][4] = "locate";
  root["sup_feat"][5] = "clean_spot";
  root["dev"]["name"] = String("Roomba ") + getMAC();
  root["dev"]["ids"][0] = getEntityID();
  root["dev"]["mf"] = "iRobot";
  root["dev"]["mdl"] = ROOMBA_MODEL;
  String jsonStr;
  serializeJson(root, jsonStr);
  DLOG("Reporting config: %s\n", jsonStr.c_str());
  mqttClient.publish(getMQTTTopic(configTopic), jsonStr.c_str());
}

void sendStatus()
{
  if (!mqttClient.connected())
  {
    DLOG("MQTT Disconnected, not sending status\n");
    return;
  }
  StaticJsonDocument<200> root;
  //root["battery_level"] = (roombaState.charge * 100) / roombaState.capacity;
  root["test"] = roombaState.capacity;
  root["cleaning"] = roombaState.cleaning;
  root["docked"] = roombaState.docked;
  root["charging"] = roombaState.chargingState == Roomba::ChargeStateReconditioningCharging || roombaState.chargingState == Roomba::ChargeStateFullCharging || roombaState.chargingState == Roomba::ChargeStateTrickleCharging;
  root["voltage"] = roombaState.voltage;
  root["current"] = roombaState.current;
  root["charge"] = roombaState.charge;
  String curState = "idle";
  if (roombaState.docked)
  {
    curState = "docked";
  }
  else
  {
    if (roombaState.cleaning)
    {
      curState = "cleaning";
    }
  }
  root["state"] = curState;
  String jsonStr;
  serializeJson(root, jsonStr);
  DLOG("Reporting status: %s\n", jsonStr.c_str());
  mqttClient.publish(getMQTTTopic(stateTopic), jsonStr.c_str());
}

int lastStateMsgTime = 0;
int lastConnectTime = 0;
int configLoop = 0;
int lastTimeSync = 0;

void loop()
{
  // Important callbacks that _must_ happen every cycle
  ArduinoOTA.handle();
  yield();
  Debug.handle();

  // Skip all other logic if we're running an OTA update
  if (OTAStarted)
  {
    return;
  }

  long now = millis();
  if ( SET_DATETIME && (now / 1000 - lastTimeSync / 1000 ) > 300)
  {
    lastTimeSync = now;
    wakeup();
    // set time
    setDateTime();
  }

  // If MQTT client can't connect to broker, then reconnect every 30 seconds
  if ((now / 1000 - lastConnectTime / 1000 ) > 30)
  {
    lastConnectTime = now;
    if (!mqttClient.connected())
    {
      DLOG("Reconnecting MQTT\n");
      reconnect();
      sendConfig();
    }
    else
    {
      // resend config every now and then to reconfigure entity e.g. in case homeassistant has been restarted
      if (configLoop == 19)
      {
        sendConfig();
        configLoop = 0;
      }
      else
      {
        configLoop++;
      }
    }
  }
  // Report the status over mqtt at fixed intervals
  if ((now / 1000 - lastStateMsgTime / 1000) > 30)
  {
    lastStateMsgTime = now;
      DLOG("Sending status\n");
      readSensorPacket();      
      sendStatus();
      sleepIfNecessary();
  }

  mqttClient.loop();
}
