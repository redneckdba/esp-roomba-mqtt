#define HOSTNAME "roomba" // e.g. roomba.local
#define OTA_PORT 8266
#define ROOMBA_WAKEUP 14
#define MAX_BATT_CAPACITY 3000 // battery capacity in mAh

#define SET_DATETIME 1
#define NTP_TIME_OFFSET 3600 * 1
#define NTP_SERVER "pool.ntp.org"
#define ROOMBA_WAKEUP_INTERVAL 180000
#define MIN_CLEANING_CURRENT -500

#define ADC_VOLTAGE_DIVIDER 44.551316985
#define ENABLE_ADC_SLEEP 0

// Home Assistant Settings
#define HASS_UNIQUE_ID "roomba"
#define HASS_NAME "Roomba"
#define HASS_MANUFACTURER "iRobot"
#define HASS_MODEL "Roomba 866"
#define HASS_VERSION "1.0.0-dev.0"

// Only change if you know what you're doing!
#define MQTT_BUFFER_SIZE 1024
#define MQTT_RECONNECT_DELAY 5000
#define MQTT_PUBLISH_INTERVAL 10000
#define MQTT_CLIENT_NAME "Roomba"
#define MQTT_DISCOVERY "homeassistant"
#define MQTT_DEVICE_CLASS "vacuum"
#define MQTT_DIVIDER "/"
#define MQTT_TOPIC_BASE MQTT_DISCOVERY MQTT_DIVIDER MQTT_DEVICE_CLASS MQTT_DIVIDER
#define MQTT_IDPREFIX "roomba_"
#define MQTT_COMMAND_TOPIC "command"
#define MQTT_STATE_TOPIC "state"
#define MQTT_CONFIG_TOPIC "config"
#define MQTT_AVAILABILITY_TOPIC "status"
