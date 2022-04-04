/**
  PWM LED controller firmware for the Open eXtensible Rack System

  Documentation:  
    https://oxrs.io/docs/hardware/output-devices/pwm-controllers.html
  
  GitHub repository:
    https://github.com/austinscreations/OXRS-AC-LEDController-ESP-FW

  Copyright 2022 Austins Creations
*/

/*------------------------ Board Type ---------------------------------*/
//#define MCU32
//#define MCU8266 
//#define MCULILY

/*----------------------- Connection Type -----------------------------*/
//#define ETHMODE
//#define WIFIMODE

/*------------------------- I2C pins ----------------------------------*/
//#define I2C_SDA   0
//#define I2C_SCL   1

//rack32   = 21  22
//LilyGO   = 33  32
//room8266 =  4   5
//D1 mini  =  4   0

/*--------------------------- Macros ----------------------------------*/
#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

/*--------------------------- Libraries -------------------------------*/
#include <Arduino.h>
#include <Wire.h>                   // For I2C
#include <PubSubClient.h>           // For MQTT
#include <OXRS_MQTT.h>              // For MQTT
#include <OXRS_API.h>               // For REST API
#include <OXRS_SENSORS.h>           // For QWICC I2C sensors
#include <Adafruit_NeoPixel.h>      // control LED channels
#include <WiFiManager.h>            // captive wifi AP config

#if defined(MCU8266)
#include <ESP8266WiFi.h>            // For networking
#if defined(ETHMODE)
#include <Ethernet.h>               // For networking
#include <SPI.h>                    // For ethernet
#endif
#endif

/*--------------------------- Constants ----------------------------------*/
// Serial
#define SERIAL_BAUD_RATE            115200

// REST API
#define REST_API_PORT               80

// Supported LED modes
#define       LED_MODE_NONE             0
#define       LED_MODE_COLOUR           1
#define       LED_MODE_FLASH            3
#define       LED_MODE_BLINK            4

// Supported Tower modes
#define       TOWER_MODE_SINGLE         0
#define       TOWER_MODE_MULTI          1

// For the 3ch WS2811 LED driver IC
#define       LED_PIN                   0
#define       LED_COUNT                 2

#define       DEFAULT_BLINK_RATE        1000

// Ethernet
#if defined(ETHMODE)
#define DHCP_TIMEOUT_MS             15000
#define DHCP_RESPONSE_TIMEOUT_MS    4000
#define WIZNET_RST_PIN              2
#define ETHERNET_CS_PIN             15
#endif

/*-------------------------- Internal datatypes --------------------------*/

/*--------------------------- Global Variables ---------------------------*/
// Flashing state for any strips in flash mode
uint8_t g_flash_state = false;

// Blink flash control - internal flashing control
uint8_t g_blink_state = false;
uint32_t blinkMs = DEFAULT_BLINK_RATE;
unsigned long lastBlink;

// LED varibles
uint8_t towermode = TOWER_MODE_SINGLE;   // set default tower mode to single - can be changed via mqtt config
uint8_t ledmode[6] = {0,0,0,0,0,0};        // mode for each led channel
uint8_t ledbrightness[6] = {0,0,0,0,0,0};  // brightness for led channel

// tower varible
uint8_t output[6] = {0,0,0,0,0,0};       // output value for tower 
uint8_t sleepModes[6] = {0,1,1,1,1,0};   // if a light is on / off during sleep mode
uint8_t sleepState = false;              // is the device supposed to be asleep

// stack size counter (for determine used heap size on ESP8266)
char * g_stack_start;

/*--------------------------- Instantiate Global Objects -----------------*/
#if defined(ETHMODE)
EthernetClient client;
EthernetServer server(REST_API_PORT);
#endif

#if defined(WIFIMODE)
WiFiClient client;
WiFiServer server(REST_API_PORT);
#endif

// MQTT
PubSubClient mqttClient(client);
OXRS_MQTT mqtt(mqttClient);

// REST API
OXRS_API api(mqtt);

// I2C sensors
OXRS_SENSORS sensors(mqtt);

// LED Driver IC
Adafruit_NeoPixel driver(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

/*--------------------------- JSON builders -----------------*/
uint32_t getStackSize()
{
  char stack;
  return (uint32_t)g_stack_start - (uint32_t)&stack;  
}

void getFirmwareJson(JsonVariant json)
{
  JsonObject firmware = json.createNestedObject("firmware");

  firmware["name"] = STRINGIFY(FW_NAME);
  firmware["shortName"] = STRINGIFY(FW_SHORT_NAME);
  firmware["maker"] = STRINGIFY(FW_MAKER);
  firmware["version"] = STRINGIFY(FW_VERSION);
}

void getSystemJson(JsonVariant json)
{
  JsonObject system = json.createNestedObject("system");

  system["flashChipSizeBytes"] = ESP.getFlashChipSize();
  system["heapFreeBytes"] = ESP.getFreeHeap();

  #if defined(MCU8266)
  system["heapUsedBytes"] = getStackSize();
  #endif

  system["sketchSpaceUsedBytes"] = ESP.getSketchSize();
  system["sketchSpaceTotalBytes"] = ESP.getFreeSketchSpace();

  #if defined(MCU8266)
  FSInfo fsInfo;
  SPIFFS.info(fsInfo);  
  system["fileSystemUsedBytes"] = fsInfo.usedBytes;
  system["fileSystemTotalBytes"] = fsInfo.totalBytes;
  #endif
}

void getNetworkJson(JsonVariant json)
{
  JsonObject network = json.createNestedObject("network");
  
  byte mac[6];
  
  #if defined(ETHMODE)
  Ethernet.MACAddress(mac);
  network["ip"] = Ethernet.localIP();
  #elif defined(WIFIMODE)
  WiFi.macAddress(mac);
  network["ip"] = WiFi.localIP();
  #endif
  
  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  network["mac"] = mac_display;
}

void getConfigSchemaJson(JsonVariant json)
{
  JsonObject configSchema = json.createNestedObject("configSchema");
  
  // Config schema metadata
  configSchema["$schema"] = JSON_SCHEMA_VERSION;
  configSchema["title"] = STRINGIFY(FW_SHORT_NAME);
  configSchema["type"] = "object";

  JsonObject properties = configSchema.createNestedObject("properties");

  JsonObject towerMode = properties.createNestedObject("towerMode");
  towerMode["type"] = "string";
  JsonArray towerEnum = towerMode.createNestedArray("enum");
  towerEnum.add("single");
  towerEnum.add("multi");

  JsonObject blinkMillis = properties.createNestedObject("blinkMs");
  blinkMillis["type"] = "integer";
  blinkMillis["minimum"] = 0;
  blinkMillis["maximum"] = 10000;
  blinkMillis["description"] = "Rate to blink a light in milliseconds";

  JsonObject towerSleep = properties.createNestedObject("towerSleep");
  towerSleep["type"] = "array";

  JsonObject towerItems = towerSleep.createNestedObject("items");
  towerItems["type"] = "object";

  JsonObject towerSleepProperties = towerItems.createNestedObject("properties");
  
  JsonObject sleepModes = towerSleepProperties.createNestedObject("sleepModes");
  sleepModes["type"] = "array";
  sleepModes["minItems"] = 6;
  sleepModes["maxItems"] = 6;
  JsonObject sleepItems = sleepModes.createNestedObject("items");
  sleepItems["type"] = "number";
  sleepItems["minimum"] = 0;
  sleepItems["maximum"] = 1;

  // Add any sensor config
  sensors.setConfigSchema(properties);
}

void getCommandSchemaJson(JsonVariant json)
{
  JsonObject commandSchema = json.createNestedObject("commandSchema");
  
  // Command schema metadata
  commandSchema["$schema"] = JSON_SCHEMA_VERSION;
  commandSchema["title"] = STRINGIFY(FW_SHORT_NAME);
  commandSchema["type"] = "object";

  JsonObject properties = commandSchema.createNestedObject("properties");

  JsonObject tower = properties.createNestedObject("tower");
  tower["type"] = "array";

  JsonObject towerItems = tower.createNestedObject("items");
  towerItems["type"] = "object";

  JsonObject towerProperties = towerItems.createNestedObject("properties");

  JsonObject channeling = towerProperties.createNestedObject("channel");
  channeling["type"] = "integer";
  channeling["minimum"] = 1;
  channeling["maximum"] = 5;

  JsonObject modes = towerProperties.createNestedObject("mode");
  modes["type"] = "string";
  JsonArray modesEnum = modes.createNestedArray("enum");
  modesEnum.add("colour");
  modesEnum.add("flash");
  modesEnum.add("blink");

  JsonObject brightnesss = towerProperties.createNestedObject("brightness");
  brightnesss["type"] = "integer";
  brightnesss["maximum"] = 255;

  JsonArray required = towerItems.createNestedArray("required");
  required.add("channel");
  required.add("mode");
  required.add("brightness");

  JsonObject sleeping = properties.createNestedObject("sleep");
  sleeping["type"] = "boolean";

  JsonObject flash = properties.createNestedObject("flash");
  flash["type"] = "boolean";

  JsonObject restart = properties.createNestedObject("restart");
  restart["type"] = "boolean";

  // Add any sensor commands
  sensors.setCommandSchema(properties);
}

void apiAdopt(JsonVariant json)
{
  // Build device adoption info
  getFirmwareJson(json);
  getSystemJson(json);
  getNetworkJson(json);
  getConfigSchemaJson(json);
  getCommandSchemaJson(json);
}

/*--------------------------- Initialisation -------------------------------*/
void initialiseRestApi(void)
{
  // NOTE: this must be called *after* initialising MQTT since that sets
  //       the default client id, which has lower precendence than MQTT
  //       settings stored in file and loaded by the API

  // Set up the REST API
  api.begin();

  // Register our callbacks
  api.onAdopt(apiAdopt);

  server.begin();
}

/*--------------------------- LED -----------------*/
void driver_off() {
    driver.setPixelColor(0,0,0,0);         //  Set pixel's color (in RAM)
    driver.setPixelColor(1,0,0,0);         //  Set pixel's color (in RAM)
    driver.show();                         //  Update drivers to match
}

void driver_loop()
{
  for (uint8_t chan = 0; chan < 6; chan++)
  {
/*
 * need to know what each channel has to do
 * save each channels value to combine at the end and send to drivers
 * update values based on flashing mode
 * buzzer to be added later
 * 
 */
    if (ledmode[chan] == LED_MODE_COLOUR) // static color selected
    {
      output[chan] = ledbrightness[chan];
    }
    else if (ledmode[chan] == LED_MODE_FLASH) // flash specified channel
    {
      if (g_flash_state == HIGH)
      {
        output[chan] = ledbrightness[chan];
      }
      else
      {
        output[chan] = 0;
      }
    }
    else if (ledmode[chan] == LED_MODE_BLINK) // blink specified channel
    {
      if (g_blink_state == HIGH)
      {
        output[chan] = ledbrightness[chan];
      }
      else
      {
        output[chan] = 0;
      }
    }
    else
    {
      output[chan] = 0;
    }
    // device should be asleep instead
    if (sleepState == true)
    {
      if (sleepModes[chan] == 1) // this channel is confirmed to be off in sleepstate mode
      {
        output[chan] = 0;
      }
    }
  }
    driver.setPixelColor(0,output[0],output[1],output[2]);         //  Set pixel's color (in RAM)
    driver.setPixelColor(1,output[3],output[4],output[5]);         //  Set pixel's color (in RAM)
    driver.show();                                                 //  Update drivers to match
//  Serial.println("driver called");
}

/*--------------------------- MQTT/API -----------------*/
void mqttConnected() 
{
  // Publish device adoption info
  DynamicJsonDocument json(JSON_ADOPT_MAX_SIZE);
  mqtt.publishAdopt(api.getAdopt(json.as<JsonVariant>()));

  // Log the fact we are now connected
  Serial.println("[TLC] mqtt connected");
}

void mqttDisconnected(int state) 
{
  // Log the disconnect reason
  // See https://github.com/knolleary/pubsubclient/blob/2d228f2f862a95846c65a8518c79f48dfc8f188c/src/PubSubClient.h#L44
  switch (state)
  {
    case MQTT_CONNECTION_TIMEOUT:
      Serial.println(F("[TLC] mqtt connection timeout"));
      break;
    case MQTT_CONNECTION_LOST:
      Serial.println(F("[TLC] mqtt connection lost"));
      break;
    case MQTT_CONNECT_FAILED:
      Serial.println(F("[TLC] mqtt connect failed"));
      break;
    case MQTT_DISCONNECTED:
      Serial.println(F("[TLC] mqtt disconnected"));
      break;
    case MQTT_CONNECT_BAD_PROTOCOL:
      Serial.println(F("[TLC] mqtt bad protocol"));
      break;
    case MQTT_CONNECT_BAD_CLIENT_ID:
      Serial.println(F("[TLC] mqtt bad client id"));
      break;
    case MQTT_CONNECT_UNAVAILABLE:
      Serial.println(F("[TLC] mqtt unavailable"));
      break;
    case MQTT_CONNECT_BAD_CREDENTIALS:
      Serial.println(F("[TLC] mqtt bad credentials"));
      break;      
    case MQTT_CONNECT_UNAUTHORIZED:
      Serial.println(F("[TLC] mqtt unauthorised"));
      break;      
  }
}

uint8_t getChannel(JsonVariant json)
{
  if (!json.containsKey("channel"))
  {
    Serial.println(F("[TLC] missing channel"));
    return 0;
  }
  
  uint8_t channel = json["channel"].as<uint8_t>();

  // Check the controller is valid for this device
  if (channel <= 0 || channel > 6)
  {
    Serial.println(F("[TLC] invalid channel"));
    return 0;
  }

  return channel;
}

void jsonTowerCommand(JsonVariant json) //sets led channel mode and brightness during normal operation
{
  uint8_t channels = getChannel(json);
  if (channels == 0) return;

  uint8_t lightchan = (channels - 1);

if (json.containsKey("mode"))
  {
    if (strcmp(json["mode"], "colour") == 0)
    { 
      ledmode[lightchan] = LED_MODE_COLOUR;
    }
    else if (strcmp(json["mode"], "flash") == 0)
    {
      ledmode[lightchan] = LED_MODE_FLASH;
    }
    else if (strcmp(json["mode"], "blink") == 0)
    {
      ledmode[lightchan] = LED_MODE_BLINK;
    }
    else 
    {
      Serial.println(F("[TLC] invalid mode"));
      return;
    }
  }

  if (json.containsKey("brightness"))
  {
    if (lightchan == 5)
    {
      ledbrightness[lightchan] = json["brightness"].as<uint8_t>();
    }
    else
    {
      if (towermode == TOWER_MODE_MULTI)
      {
        ledbrightness[lightchan] = json["brightness"].as<uint8_t>();
      }
      else
      {
        for (uint8_t x = 0; x < 5; x++)
        {
          ledbrightness[x] = 0;
        }
        ledbrightness[lightchan] = json["brightness"].as<uint8_t>();
      }
    }
  }
}

void jsonCommand(JsonVariant json)
{
  
  if (json.containsKey("tower"))
  {
    for (JsonVariant tower : json["tower"].as<JsonArray>())
    {
      jsonTowerCommand(tower);
    }
  }

  if (json.containsKey("flash"))
  {
    g_flash_state = json["flash"].as<bool>() ? HIGH : LOW;
  }

  if (json.containsKey("restart") && json["restart"].as<bool>())
  {
    ESP.restart();
  }

  // Let the sensors handle any commands
  sensors.cmnd(json);
}

void jsonConfig(JsonVariant json)
{
  if (json.containsKey("towerMode")) // for what mode the tower lights are in
  {
    if (strcmp(json["towerMode"], "single") == 0)
    {
      towermode = TOWER_MODE_SINGLE;
      for (uint8_t x = 0; x < 6; x++){ledbrightness[x] = 0;} // reset tower outputs
    }
    else if (strcmp(json["towerMode"], "multi") == 0)
    {
      towermode = TOWER_MODE_MULTI;
      for (uint8_t x = 0; x < 6; x++){ledbrightness[x] = 0;} // reset tower outputs
    }
    else 
    {
      Serial.println(F("[TLC] invalid Tower Mode Config"));
    }
  }

  if (json.containsKey("blinkMs")) // for updating blink speed
  {
    blinkMs = json["blinkMs"].as<uint32_t>();
  }

  if (json.containsKey("towerSleep"))
  {
      if (json.containsKey("sleepModes"))
      {
        JsonArray array = json["sleepModes"].as<JsonArray>();
        uint8_t sleepnum = 0;
    
        for (JsonVariant v : array)
        {
          sleepModes[sleepnum++] = v.as<uint8_t>();
        }
      }
  }

   // Let the sensors handle any config
  sensors.conf(json);
}

void mqttCallback(char * topic, uint8_t * payload, unsigned int length) 
{
  // Pass this message down to our MQTT handler
  mqtt.receive(topic, payload, length);
}

void initialiseMqtt(byte * mac)
{
  // Set the default client id to the last 3 bytes of the MAC address
  char clientId[32];
  sprintf_P(clientId, PSTR("%02x%02x%02x"), mac[3], mac[4], mac[5]);  
  mqtt.setClientId(clientId);
  
  // Register our callbacks
  mqtt.onConnected(mqttConnected);
  mqtt.onDisconnected(mqttDisconnected);
  mqtt.onConfig(jsonConfig);
  mqtt.onCommand(jsonCommand);  

  // Start listening for MQTT messages
  mqttClient.setCallback(mqttCallback);  
}

/*--------------------------- Network -------------------------------*/
#if defined(WIFIMODE)
void initialiseWifi()
{
  // Ensure we are in the correct WiFi mode
  WiFi.mode(WIFI_STA);

  // Get WiFi base MAC address
  byte mac[6];
  WiFi.macAddress(mac);

  // Display the MAC address on serial
  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(F("[TLC] mac address: "));
  Serial.println(mac_display);

  // Update OLED display
  sensors.oled(mac);

  // Connect using saved creds, or start captive portal if none found
  // Blocks until connected or the portal is closed
  WiFiManager wm;
  if (!wm.autoConnect("OXRS_WiFi", "superhouse"))
  {
    // If we are unable to connect then restart
    ESP.restart();
  }

  // Display IP address on serial
  Serial.print(F("[TLC] ip address: "));
  Serial.println(WiFi.localIP());

  // Update OLED display
  sensors.oled(Ethernet.localIP());

  // Set up MQTT (don't attempt to connect yet)
  initialiseMqtt(mac);

  // Set up the REST API once we have an IP address
  initialiseRestApi();
}
#endif

#if defined(ETHMODE)
void initialiseEthernet()
{
  // Get ESP base MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  
  // Ethernet MAC address is base MAC + 3
  // See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address
  mac[5] += 3;

  // Display the MAC address on serial
  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(F("[TLC] mac address: "));
  Serial.println(mac_display);

  // Update OLED display
  sensors.oled(mac);

  // Initialise ethernet library
  Ethernet.init(ETHERNET_CS_PIN);

  // Reset Wiznet W5500
  pinMode(WIZNET_RST_PIN, OUTPUT);
  digitalWrite(WIZNET_RST_PIN, HIGH);
  delay(250);
  digitalWrite(WIZNET_RST_PIN, LOW);
  delay(50);
  digitalWrite(WIZNET_RST_PIN, HIGH);
  delay(350);

  // Get an IP address via DHCP
  Serial.print(F("[TLC] ip address: "));
  if (!Ethernet.begin(mac, DHCP_TIMEOUT_MS, DHCP_RESPONSE_TIMEOUT_MS))
  {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println(F("ethernet shield not found"));
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println(F("ethernet cable not connected"));
    } else {
      Serial.println(F("failed to setup ethernet using DHCP"));
    }
    return;
  }
  
  // Display IP address on serial
  Serial.println(Ethernet.localIP());

  // Update OLED display
  sensors.oled(Ethernet.localIP());

  // Set up MQTT (don't attempt to connect yet)
  initialiseMqtt(mac);

  // Set up the REST API once we have an IP address
  initialiseRestApi();

}
#endif

void initialiseSerial()
{
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  
  Serial.println(F("[TLC ] starting up..."));

  DynamicJsonDocument json(128);
  getFirmwareJson(json.as<JsonVariant>());

  Serial.print(F("[TLC ] "));
  serializeJson(json, Serial);
  Serial.println();
}

/*--------------------------- Program -------------------------------*/
void setup()
{
  // Store the address of the stack at startup so we can determine
  // the stack size at runtime (see getStackSize())
  char stack;
  g_stack_start = &stack;

  // Set up serial
  initialiseSerial();  

  // Start the I2C bus
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Start the sensor library (scan for attached sensors)
  sensors.begin();

  // Set up LED Driver
  driver.begin();        // INITIALIZE NeoPixel strip object (REQUIRED)
  driver_off();          // Turn all drivers off
  driver_off();          // Turn all drivers off

  // Set up network/MQTT/REST API
  #if defined(WIFIMODE)
  initialiseWifi();
  #elif defined(ETHMODE)
  initialiseEthernet();
  #endif
}

void loop()
{
  // Check our MQTT broker connection is still ok
  mqtt.loop();

  // Maintain DHCP lease
  #if defined(ETHMODE)
  Ethernet.maintain();
  #endif
  
  // Handle any API requests
  #if defined(WIFIMODE)
  WiFiClient client = server.available();
  api.checkWifi(&client);
  #elif defined(ETHMODE)
  EthernetClient client = server.available();
  api.checkEthernet(&client);
  #endif

  driver_loop();  // update driver channels
  sensors.oled();  // update OLED
  sensors.tele();  // update mqtt sensors

  if ((millis() - lastBlink) > blinkMs)
  { 
    if (g_blink_state == LOW)
    {
      g_blink_state = HIGH;
    }
    else
    {
      g_blink_state = LOW;
    }
    lastBlink = millis();
    driver_loop();  // update driver channels
  }
}