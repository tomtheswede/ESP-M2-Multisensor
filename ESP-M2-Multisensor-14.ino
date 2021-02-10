/*********
  Original sketch by Rui Santos (Complete project details at https://randomnerdtutorials.com), adapted by Thomas Friberg for JSON6 and added sensor discovery for HomeAssistant leveraging https://github.com/AnaviTechnology/anavi-thermometer-sw/blob/master/anavi-thermometer-sw/anavi-thermometer-sw.ino, also https://github.com/ItKindaWorks/ESP8266/blob/master/Home%20Automation/Part%201/ESP8266_SimpleMQTT/ESP8266_SimpleMQTT.ino
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ArduinoJson.h>


// Replace the next variables with your SSID/Password combination
const char* ssid = "NOKIA-4023435DF995";
const char* password = "5985619313";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.1.117";
const char subTopic[] = "homeassistant/switch/studyESP/set";
const char pubTopic[] = "homeassistant/sensor/studyESP";
const char switchPubTopic[] = "homeassistant/sensor/studyESP";
const char tempPubTopic[] = "homeassistant/sensor/studyESP/temperature";
const char humPubTopic[] = "homeassistant/sensor/studyESP/humidity";
const char topicPrefix[] = "homeassistant/sensor/studyESP/";
const char switchTopicPrefix[] = "homeassistant/switch/studyESP/";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
String discoveryString;
char data[1000];

//uncomment the following lines if you're using SPI
/*#include <SPI.h>
#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5*/

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
float temperature = 0;
float humidity = 0;

//TSL2561 Adafruit library initialisation
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
float light = 0;


// LED Pin
const int ledPin = 2;

void setup() {
  Serial.begin(115200);
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();  
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  //initialise TSL2561
  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light/outdoor to avoid sensor saturation */
  tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light/indoor to boost sensitivity */
  // tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  pinMode(ledPin, OUTPUT);
}

void setup_wifi() {
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
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
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
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == subTopic) {
    Serial.print("Changing output to ");
    if(messageTemp == "ON"){
      Serial.println("ON");
      digitalWrite(ledPin, HIGH);
      //Send state update back to base
      char tempStateTopic[200] = ""; //Reset the variable before concatinating
      strcat(tempStateTopic,switchTopicPrefix);
      strcat(tempStateTopic,"state");
      client.publish(tempStateTopic, "ON", true);
    }
    else if(messageTemp == "OFF"){
      Serial.println("OFF");
      digitalWrite(ledPin, LOW);
      //Send state update back to base
      char tempStateTopic[200] = ""; //Reset the variable before concatinating
      strcat(tempStateTopic,switchTopicPrefix);
      strcat(tempStateTopic,"state");
      client.publish(tempStateTopic, "OFF", true);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client","user","passwd")) {
      Serial.println("connected");
      // Discovery
      publishSensorDiscovery("sensor", "temp", "temperature", "temperature", pubTopic, "°C", "{{ value_json.temperature }}");
      publishSensorDiscovery("sensor", "hum", "humidity", "humidity", pubTopic, "%", "{{ value_json.humidity }}");
      publishSensorDiscovery("sensor", "lum", "illuminance", "light", pubTopic, "lx", "{{ value_json.light }}");
      publishSwitchDiscovery("switch", "led", 0, "switch", switchPubTopic);
      
      // Subscribe
      client.subscribe(subTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishSensorDiscovery(const char *component, const char *config_key, const char *device_class, const char *name_suffix, const char *state_topic, const char *units, const char *value_template) {

  char* ha_name = "ESP"; //added
  DynamicJsonDocument json(1000);
  if (device_class) json["device_class"] = device_class;
  json["name"] = String(ha_name) + " " + name_suffix;
  json["uniq_id"] = String("ESP32-") + name_suffix;;
  json["stat_t"] = state_topic;  //"homeassistant/sensor/studyESP";
  json["unit_of_meas"] = units;  //Abreviated to reduce package length - https://www.home-assistant.io/docs/mqtt/discovery/
  json["val_tpl"] = value_template;   //Abreviated to reduce package length - https://www.home-assistant.io/docs/mqtt/discovery/
  //json["availability_topic"] = mqtt_status(mqtt_name)->availability_topic;
//  json["device"]["identifiers"] = "machId";
//  json["device"]["manufacturer"] = "Thomas Tech";
//  json["device"]["model"] = "ThomasSensorMk1";
//  json["device"]["name"] = String(ha_name) + " " + name_suffix;
//  json["device"]["sw_version"] = ESP.getSketchMD5();
//  JsonArray connections = json["device"].createNestedArray("connections").createNestedArray();
//  connections.add("mac");
//  connections.add(WiFi.macAddress());

  //prep for send
  serializeJson(json, data);
  //size_t n = serializeJson(json, data);
  char tempTopic[200] = ""; //Reset the variable before concatinating
  strcat(tempTopic,topicPrefix); //forming the discovery config topic based on config key
  strcat(tempTopic,config_key);
  strcat(tempTopic,"/config");
  client.publish(tempTopic, data, true); //publishing the discovery note
  
  Serial.print("Discovery pub complete: ");
  Serial.println(data);

}

void publishSwitchDiscovery(const char *component, const char *config_key, const char *device_class, const char *name_suffix, const char *state_topic) {

  char* ha_name = "ESP"; //added
  DynamicJsonDocument json(1000);
  if (device_class) json["device_class"] = device_class;
  json["name"] = String(ha_name) + " " + name_suffix;
  json["uniq_id"] = String("ESP32-") + name_suffix;;
  char tempStateTopic[200] = ""; //Reset the variable before concatinating
  strcat(tempStateTopic,switchTopicPrefix);
  //strcat(tempStateTopic,"/");
  strcat(tempStateTopic,"state");
  json["stat_t"] = tempStateTopic;  //"homeassistant/sensor/studyESP";
  char tempCmdTopic[200] = ""; //Reset the variable before concatinating
  strcat(tempCmdTopic,switchTopicPrefix);
  //strcat(tempCmdTopic,"/");
  strcat(tempCmdTopic,"set");
  json["command_topic"] = tempCmdTopic; 
  //json["payload_on"] = "ON";
  //json["payload_off"] = "OFF";
//  json["unit_of_meas"] = units;  //Abreviated to reduce package length - https://www.home-assistant.io/docs/mqtt/discovery/
//  json["val_tpl"] = value_template;   //Abreviated to reduce package length - https://www.home-assistant.io/docs/mqtt/discovery/
  //json["availability_topic"] = mqtt_status(mqtt_name)->availability_topic;
//  json["device"]["identifiers"] = "machId";
//  json["device"]["manufacturer"] = "Thomas Tech";
//  json["device"]["model"] = "ThomasSensorMk1";
//  json["device"]["name"] = String(ha_name) + " " + name_suffix;
//  json["device"]["sw_version"] = ESP.getSketchMD5();
//  JsonArray connections = json["device"].createNestedArray("connections").createNestedArray();
//  connections.add("mac");
//  connections.add(WiFi.macAddress());

  //prep for send
  serializeJson(json, data);
  //size_t n = serializeJson(json, data);
  char tempTopic[200] = ""; //Reset the variable before concatinating
  strcat(tempTopic,switchTopicPrefix); //forming the discovery config topic based on config key
  strcat(tempTopic,config_key);
  strcat(tempTopic,"/config");
  client.publish(tempTopic, data, true); //publishing the discovery note
  
  Serial.print("Discovery pub complete: ");
  Serial.println(data);

}


//-------------------

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 30000) {
    lastMsg = now;
    
    // Temperature in Celsius
    temperature = bme.readTemperature();   
    // Uncomment the next line to set temperature in Fahrenheit 
    // (and comment the previous temperature line)
    //temperature = 1.8 * bme.readTemperature() + 32; // Temperature in Fahrenheit
    // Convert the value to a char array
    char tempString[8];
    dtostrf(temperature, 3, 1, tempString);
    //Serial.print("Temperature: ");
    //Serial.println(tempString);
    // Convert the value to a char array
    humidity = bme.readHumidity();
    char humString[8];
    dtostrf(humidity, 3, 1, humString);
    //Serial.print("Humidity: ");
    //Serial.println(humString);

    //Call TSL2561 sensor data
    sensors_event_t event;
    tsl.getEvent(&event);
    /* Display the results (light is measured in lux) */
    if (event.light)
    {
      Serial.print(event.light); Serial.println(" lux");
      light = event.light;
    }
    else
    {
      /* If event.light = 0 lux the sensor is probably saturated
         and no reliable data could be generated! */
      Serial.println("Sensor overload");
    }
    char lightString[8];
    dtostrf(light, 5, 0, lightString);

    StaticJsonDocument<200> doc;
    //JsonObject& doc = doc.createObject();
    String tempString3 = "";
    tempString3=(String)tempString;
    tempString3.trim();
    doc["temperature"] = (String)tempString3;
    doc["humidity"] = (String)humString; //BME280 sometimes doesn't collect a reading on first load, skip the first reading if not good.

    String tempString2 = "";
    tempString2=(String)lightString;
    tempString2.trim();
    doc["light"] = tempString2;

    //prep for send
    //char data[200];
    serializeJson(doc,Serial);
    Serial.println();
    serializeJson(doc, data); //, doc.measureJson() + 1
    client.publish(pubTopic, data, true);
    
  }
}
