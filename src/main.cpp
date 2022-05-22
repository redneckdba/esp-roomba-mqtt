#include <LittleFS.h>                   //this needs to be first, or it all crashes and burns...
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <Roomba.h>
#include <PubSubClient.h>
#include "SimpleTimer.h"
#include <ArduinoJson.h>
#include "config.h"
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager

//TimeZone and NTP Settings
#include <WiFiUdp.h>
#include <NTPClient.h>


#if LOGGING
#include <RemoteDebug.h>
RemoteDebug Debug;
#endif

//config setup
char MQTT_SERVER[16];
char MQTT_PORT[6] = "1883";
char MQTT_USER[40];
char MQTT_PASSWORD[40];

struct Sensors
{
  bool hasData;
  unsigned int bytesRead;
  bool bumpRight;
  bool bumpLeft;
  bool wheelDropRight;
  bool wheelDropLeft;
  unsigned int dirtLevel;
  int chargingState;
  int current;
  int voltage;
  int temperature;
  unsigned int batteryCharge;
  unsigned int batteryCapacity;
  unsigned int batteryPercent;
  bool isExternalCharger;
  bool isHome;
};

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
SimpleTimer timer;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


const char *commandTopic = MQTT_COMMAND_TOPIC;
const char *stateTopic = MQTT_STATE_TOPIC;
const char *configTopic = MQTT_CONFIG_TOPIC;
const char *availabilityTopic = MQTT_AVAILABILITY_TOPIC;

int getBitFromByte(int byte, int bit)
{
  return (byte >> bit) & 1;
}


// the output range should be -128 to 128
int signed1BytesInt(int byte)
{
  int topBit = getBitFromByte(byte, 7);
  int lowerBits = byte & 127;
  if (topBit == 1) {
  return lowerBits - (1 << 7);
  } else {
  return lowerBits;
}   
}

// the output range should be -32768 to 32767
int signed2BytesInt(int highByte, int lowByte)
 {
   // 1. Take everything except the top bit from the high byte
   int topBit = getBitFromByte(highByte, 7);
   int lowerBits = highByte & 127;
   int unsignedInt = lowerBits << 8 | (lowByte & 0xFF);

  if (topBit == 1) {
    return unsignedInt - (1 << 15);
  } else {
    return unsignedInt;
  }    
 }

 int unsigned2BytesInt(int highByte, int lowByte)
 {
   return highByte << 8 | lowByte;
 }

void wakeUp()
{
  debugV("Waking up...");
  pinMode(ROOMBA_WAKEUP, OUTPUT);
  delay(100);
  digitalWrite(ROOMBA_WAKEUP, HIGH);
  delay(100);
  digitalWrite(ROOMBA_WAKEUP, LOW);
  delay(500);
  pinMode(ROOMBA_WAKEUP, HIGH);
  debugV("Wakeup complete!");
}

void clean() {
  debugV("Sending 'clean' command");
  Serial.write(128);
  delay(50);
  Serial.write(135);
}

void cleanMax() {
  debugV("Sending 'cleanMax' command");
  Serial.write(128);
  delay(50);
  Serial.write(136);
}

void cleanSpot() {
  debugV("Sending 'cleanSpot' command");
  Serial.write(128);
  delay(50);
  Serial.write(134);
}

void seekDock() {
  debugV("Sending 'seekDock' command");
  Serial.write(128);
  delay(50);
  Serial.write(143);
}

void stop() {
  debugV("Sending 'stop' command");
  Serial.write(128);
  delay(50);
  Serial.write(173);
}

void powerOff() {
  debugV("Sending 'powerOff' command");
  Serial.write(128);
  delay(50);
  Serial.write(133);
}

void flush() {
  debugV("Flushing serial buffer");
  int bytes = Serial.available();
  char buf[bytes];
  if(bytes > 0) {
    Serial.readBytes(buf, bytes);
  }
}

bool updateTime() 
{
  debugV("Updating time with NTP");
  timeClient.begin();
  timeClient.setTimeOffset(NTP_TIME_OFFSET);
  bool result = timeClient.forceUpdate();
  if(result) {
    int day = timeClient.getDay();
    int hour = timeClient.getHours();
    int minutes = timeClient.getMinutes();

    Serial.write(128);
    delay(50);

    Serial.write(168);
    Serial.write(day);
    Serial.write(hour);
    Serial.write(minutes);
     debugV("Updating time success (%d:%d:%d)", day, hour, minutes);
  }

  return result;
  
}

Sensors updateSensors() {
  Sensors sensors;

  // List of sensors to update
  // 7 - bumps & wheel drops, 1 byte | 0
  // 15 - dirt detect, 1 byte | 8
  // 21 - charging state, 1 byte | 16
  // 22 - Voltage, 2 bytes unsigned | 17-18
  // 23 - Current, 2 bytes unsigned | 19-20
  // 25 - Battery charge, 2 bytes | 22-23
  // 26 - Battery capacity, 2 bytes | 24-25
  
  flush();

  debugV("Updating sensors start...");
  Serial.write(128);
  Serial.write(142);
  Serial.write(6);

  delay(100);
 
  int i = Serial.available();
  char sensorbytes[100];
  if(i > 0) {
    Serial.readBytes(sensorbytes, i);

    sensors.hasData = true;
    sensors.bytesRead = i;
    sensors.bumpRight = sensorbytes[0] & 1;
    sensors.bumpLeft = sensorbytes[0] & 2;
    sensors.wheelDropRight = sensorbytes[0] & 4;
    sensors.wheelDropLeft = sensorbytes[0] & 8;

    sensors.dirtLevel = sensorbytes[8];


    sensors.chargingState = sensorbytes[16];
    sensors.isHome = sensors.chargingState > 0 && sensors.chargingState < 4;

    sensors.voltage = unsigned2BytesInt(sensorbytes[17], sensorbytes[18]);
    sensors.current = signed2BytesInt(sensorbytes[19], sensorbytes[20]);
    sensors.temperature = signed1BytesInt(sensorbytes[21]);


    sensors.batteryCharge = unsigned2BytesInt(sensorbytes[22],sensorbytes[23]);
    sensors.batteryCapacity = unsigned2BytesInt(sensorbytes[24], sensorbytes[25]);
    sensors.batteryPercent = 100 * sensors.batteryCharge / MAX_BATT_CAPACITY;
    debugV("Updating sensors success!");
  } else {
    sensors.hasData = false;
    debugV("Updating sensors failed");
  }

  return sensors;
}

String getState(Sensors sensors) {

  if(sensors.isHome) {
    return "docked"; //TODO Add "return_to_base" status based on last command (probably)
  }

  if(sensors.current < MIN_CLEANING_CURRENT) {
    return "cleaning";
  }

  return "idle";
}

String getFanSpeed(String state) {

  return state == "cleaning" ? "max" : "off";
}

void publishSensorsInformation() {
  if(!mqttClient.connected() || WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  Sensors sensors = updateSensors();
  if(sensors.hasData) {

    StaticJsonDocument<500> doc;
    
    String state = getState(sensors);
    doc["state"] = state;
    doc["fan_speed"] = getFanSpeed(state);
    doc["battery_level"] = sensors.batteryPercent;

    doc["bump_left"] = sensors.bumpLeft;
    doc["bump_right"] = sensors.bumpRight;

    doc["wheel_drop_left"] = sensors.wheelDropLeft;
    doc["wheel_drop_right"] = sensors.wheelDropRight;

    doc["dirt_level"] = sensors.dirtLevel;

    doc["voltage"] = sensors.voltage;

    doc["battery_charge"] = sensors.batteryCharge;
    doc["battery_capacity"] = sensors.batteryCapacity;

    doc["charging_state"] = sensors.chargingState;
    doc["current"] = sensors.current;

    doc["temperature"] = sensors.temperature;

    doc["bytes"] = sensors.bytesRead;

    String str;
    serializeJson(doc, str);
    mqttClient.publish(MQTT_STATE_TOPIC, str.c_str());
    debugV("Sensors info published to MQTT");
  }
}

void publishHomeAssistantAutoDiscovery(String uniqueId, String name, String valueTemplate, String deviceClass, String unitOfMeasurement, String topicType, bool isVacuum)
{
  DynamicJsonDocument doc(500);
  String fullName;
  String fullId;
  if(name == "") {
    fullName =  String(MQTT_CLIENT_NAME);
  } else {
    fullName = String(MQTT_CLIENT_NAME) + " " + name;
  }

  if(uniqueId == "") {
    fullId =  String(HASS_UNIQUE_ID);
  } else {
    fullId = String(HASS_UNIQUE_ID) + "_" + uniqueId;
  }

  doc["name"] = fullName;
  doc["unique_id"] = fullId;
  doc["state_topic"] = MQTT_STATE_TOPIC;
  doc["availability_topic"] = MQTT_AVAILABILITY_TOPIC;

  if(valueTemplate != "") {
    doc["value_template"] = "{{ value_json."+ valueTemplate + " }}";
  }

  if(deviceClass != "") {
    doc["device_class"] = deviceClass;
  }

  if(unitOfMeasurement != "") {
    doc["unit_of_measurement"] = unitOfMeasurement;
  }

  JsonObject device = doc.createNestedObject("device");

  if(isVacuum) {
    device["manufacturer"] = HASS_MANUFACTURER;
    device["model"] = HASS_MODEL;
    device["name"] = HASS_NAME;
    device["sw_version"] = HASS_VERSION;

    JsonArray connections = device.createNestedArray("connections");
    JsonArray connection = connections.createNestedArray();
    connection.add("mac");
    connection.add(WiFi.macAddress());

  } else {
    device["via_device"] = HASS_UNIQUE_ID;
  }
  
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(HASS_UNIQUE_ID);

  String str;
  serializeJson(doc, str);
  String topic = "homeassistant/" + topicType + "/" + fullId + "/config";
  mqttClient.publish(topic.c_str(), str.c_str(), true);
  debugV("Auto discovery info published to MQTT!");
}

void onMQTTMessage(char* topic, byte* payload, unsigned int length) 
{
  String newTopic = topic;
  payload[length] = '\0';
  String command = String((char *)payload);
  debugV("MQTT received command: %s", command.c_str());
  if (newTopic == MQTT_COMMAND_TOPIC) {
    wakeUp();

    // Official Home Assistant commands
    if (command == "start" || command == "pause" || command == "start_pause" || command == "stop") {
      clean();
    }
    else if (command == "clean_spot") {
      cleanSpot();
    }
    else if (command == "return_to_base") {
      seekDock();
    }
    else if (command == "turn_off") {
      powerOff();
    }
  }
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
  return strlwr(entityID);
}

void setupLittleFS(){
  //clean FS, for testing
  //LittleFS.format();

  Serial.println("mounting FS...");

  if (LittleFS.begin()) {
    Serial.println("mounted file system");
    if (LittleFS.exists("/config.json")) {
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        StaticJsonDocument<256> json;
        DeserializationError jsonError = deserializeJson(json, buf.get());
        serializeJsonPretty(json, Serial);
        if (!jsonError) {
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
}

bool shouldSaveConfig = false;
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void SetupWifiManager()
{
  WiFiManager wm;

  wm.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", MQTT_SERVER, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", MQTT_PORT, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", MQTT_USER, 40);
  WiFiManagerParameter custom_mqtt_password("password", "mqtt password", MQTT_PASSWORD, 40);

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_password);

  if (!wm.autoConnect()) {
    delay(3000);
    ESP.restart();
    delay(5000);
  }

  strcpy(MQTT_SERVER, custom_mqtt_server.getValue());
  strcpy(MQTT_PORT, custom_mqtt_port.getValue());
  strcpy(MQTT_USER, custom_mqtt_user.getValue());
  strcpy(MQTT_PASSWORD, custom_mqtt_password.getValue());

  if (shouldSaveConfig) {
    StaticJsonDocument<256> json;

    json["mqtt_server"]     = MQTT_SERVER;
    json["mqtt_port"]       = MQTT_PORT;
    json["mqtt_user"]       = MQTT_USER;
    json["mqtt_password"]   = MQTT_PASSWORD;

    File configFile = LittleFS.open("/config.json", "w");

    serializeJsonPretty(json, Serial);
    serializeJson(json, configFile);
    configFile.close();
    shouldSaveConfig = false;
  }
  WiFiManager wifiManager;
  wifiManager.autoConnect();

  String hostname(HOSTNAME);
  WiFi.hostname((const char *)hostname.c_str());
  WiFi.begin();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
}

void connectMQTT()
{
  while (!mqttClient.connected())
  {
    // Attempt to connect
  if (mqttClient.connect(HOSTNAME, MQTT_USER, MQTT_PASSWORD, MQTT_AVAILABILITY_TOPIC, 0, true, "offline")) 
  {
        debugV("MQTT connection sucessful, subscribing to command topic: %s", MQTT_COMMAND_TOPIC);
        mqttClient.subscribe(MQTT_COMMAND_TOPIC);
        mqttClient.publish(MQTT_AVAILABILITY_TOPIC, "online", true);
        publishHomeAssistantAutoDiscovery("", "", "", "", "", "vacuum", true);
        publishHomeAssistantAutoDiscovery("state", "State", "state", "", "", "sensor", false);
        publishHomeAssistantAutoDiscovery("battery", "Battery", "battery_level", "battery", "%", "sensor", false);
        publishHomeAssistantAutoDiscovery("voltage", "Voltage", "voltage / 1000", "battery", "V", "sensor", false);
        publishHomeAssistantAutoDiscovery("temperature", "Temperature", "temperature", "temperature", "°C", "sensor", false);
        publishHomeAssistantAutoDiscovery("dirt_level", "Dirt Level", "dirt_level / 255 * 100 | round(0)", "temperature", "%", "sensor", false);
      } 
      else 
      {
        delay(MQTT_RECONNECT_DELAY);
      }
  }
}

void startMQTT()
{
  mqttClient.setClient(wifiClient);
  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT));
  mqttClient.setCallback(onMQTTMessage);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  connectMQTT();
}

void startOTA()
{
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();
}

void startDebug()
{
  Debug.begin(HOSTNAME);
  Debug.setResetCmdEnabled(true);
  
}

void setup()
{
  Serial.begin(115200);
  Serial.swap();

  setupLittleFS();
  SetupWifiManager();
  startDebug();
  startOTA();
  wakeUp();
  startMQTT();
  #if SET_DATETIME
    updateTime();
  #endif

  timer.setInterval(ROOMBA_WAKEUP_INTERVAL, wakeUp);
  timer.setInterval(MQTT_PUBLISH_INTERVAL, publishSensorsInformation);

}


void loop()
{
  if(!mqttClient.connected()) {
    connectMQTT();
  }

  mqttClient.loop();
  timer.run();
  ArduinoOTA.handle();
  Debug.handle();
}
