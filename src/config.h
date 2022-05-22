#define HOSTNAME "roomba" // e.g. roomba.local
#define BRC_PIN 14
//#define ROOMBA_650_SLEEP_FIX 0

#define SET_DATETIME 1
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"

#define ADC_VOLTAGE_DIVIDER 44.551316985
#define ENABLE_ADC_SLEEP 1

// define your Roomba model, e.g. "780"
#define ROOMBA_MODEL "Roomba 866"

// Only change if you know what you're doing!
#define MQTT_DISCOVERY "homeassistant"
#define MQTT_DEVICE_CLASS "vacuum"
#define MQTT_DIVIDER "/"
#define MQTT_TOPIC_BASE MQTT_DISCOVERY MQTT_DIVIDER MQTT_DEVICE_CLASS MQTT_DIVIDER
#define MQTT_IDPREFIX "roomba_"
#define MQTT_COMMAND_TOPIC "command"
#define MQTT_STATE_TOPIC "state"
#define MQTT_CONFIG_TOPIC "config"
