#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <LOLIN_I2C_MOTOR.h>
#include <map>
#include <string>

std::map<std::string, int> buttons = {
  { "ANTICLOCKWISE", 0 },  // 0 could represent "off" or "inactive"
  { "DOWN", 1 },           // 1 could represent "on" or "active"
  { "OUT", 0 },
  { "CLOCKWISE", 1 },
  { "UP", 0 },
  { "IN", 1 }
};

// Debug mode - set to false to disable Serial output
#define DEBUG_MODE true

// Configuration structure
struct Config {
  // OTA Configuration
  char hostname[32] = "crane-controller";
  char ota_password[32] = "";

  // Motor Configuration
  uint8_t motor_speed = 100;

  // Network Configuration
  char wifi_ssid[32] = "the robot network";
  char wifi_password[32] = "isaacasimov";

  // MQTT Configuration
  char mqtt_server[32] = "robotmqtt";
  int mqtt_port = 1883;
  char mqtt_user[32] = "public";
  char mqtt_password[32] = "public";
  char mqtt_client_id[32] = "crane-controller";
  char mqtt_topic[32] = "crane/buttons";

  // System Configuration
  int wifi_timeout = 20;
  int mqtt_keepalive = 60;
} config;

// LOLIN I2C Motor Shield Configuration
LOLIN_I2C_MOTOR motor1(0x20);  // Motor board at address 0x20
LOLIN_I2C_MOTOR motor2(0x21);  // Motor board at address 0x21

// Network objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// System state
bool isEmergencyStop = false;

// --- Debugging Macros ---
#if DEBUG_MODE
#define debugPrintln(message) Serial.println(message)
#define debugPrint(message) Serial.print(message)
#else
#define debugPrintln(message)  // do nothing
#define debugPrint(message)    // do nothing
#endif

// Function to connect to WiFi with retry
bool connectToWiFi() {
  debugPrintln("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifi_ssid, config.wifi_password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < config.wifi_timeout) {
    delay(500);
    debugPrint(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    debugPrintln("");
    debugPrint("Connected to WiFi network with IP Address: ");
    debugPrintln(WiFi.localIP());
    return true;
  } else {
    debugPrintln("");
    debugPrintln("Failed to connect to WiFi");
    return false;
  }
}

// Configure motor shield
void setupMotors() {

  // Initialize I2C Motor Shield
  while (motor1.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR)  //wait motor shield ready.
  {
    motor1.getInfo();

    delay(5);
  }

  while (motor2.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR)  //wait motor shield ready.
  {
    motor2.getInfo();

    delay(5);
  }
}

// Stop all motors
void stopAllMotors() {
  // Stop all motors
  motor1.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);
  motor2.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);

  debugPrintln("All motors stopped");
}

// Control a specific motor
void controlMotor() {
  if (isEmergencyStop) {
    stopAllMotors();
    return;
  }

  if (buttons["ANTICLOCKWISE"] == 1 && buttons["CLOCKWISE"] == 0) {
    motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CCW);
    motor1.changeDuty(MOTOR_CH_A, config.motor_speed);
  }
  if (buttons["ANTICLOCKWISE"] == 0 && buttons["CLOCKWISE"] == 1) {
    motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CW);
    motor.changeDuty(MOTOR_CH_A, config.motor_speed);
  }

  if (buttons["UP"] == 1 && buttons["DOWN"] == 0) {
    motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CCW);
    motor.changeDuty(MOTOR_CH_B, config.motor_speed);
  }
  if (buttons["UP"] == 0 && buttons["DOWN"] == 1) {
    motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
    motor1.changeDuty(MOTOR_CH_B, config.motor_speed);
  }

  if (buttons["IN"] == 1 && buttons["OUT"] == 0) {
    motor2.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CCW);
    motor2.changeDuty(MOTOR_CH_A, config.motor_speed);
  }
  if (buttons["UP"] == 0 && buttons["DOWN"] == 1) {
    motor2.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CW);
    motor2.changeDuty(MOTOR_CH_A, config.motor_speed);
  }
}

// Function to connect to MQTT broker
void connectToMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  mqttClient.setServer(config.mqtt_server, config.mqtt_port);
  mqttClient.setCallback(mqttCallback);

  debugPrintln("Connecting to MQTT broker...");
  if (mqttClient.connect(config.mqtt_client_id, config.mqtt_user, config.mqtt_password)) {
    debugPrintln("Connected to MQTT broker");

    // Subscribe to topic
    mqttClient.subscribe(config.mqtt_topic);
    debugPrint("Subscribed to: ");
    debugPrintln(config.mqtt_topic);
  } else {
    debugPrint("Failed to connect to MQTT broker, rc=");
    debugPrintln(mqttClient.state());
  }
}

// MQTT message callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  debugPrint("Message arrived on topic: ");
  debugPrint(topic);
  debugPrint(". Message: ");

  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  debugPrintln(message);

  // Handle emergency stop
  if (message.indexOf("EMERGENCY_STOP") >= 0) {
    isEmergencyStop = true;
    debugPrintln("Emergency stop activated");
    return;
  }

  // Parse JSON for movement commands
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    debugPrint("deserializeJson() failed: ");
    debugPrintln(error.c_str());
    return;
  }

  // ANTICLOCKWISE
  if (doc.containsKey("ANTICLOCKWISE")) {
    buttons["ANTICLOCKWISE"] = doc["ANTICLOCKWISE"];
  }
  if (doc.containsKey("CLOCKWISE")) {
    buttons["CLOCKWISE"] = doc["CLOCKWISE"];
  }
  if (doc.containsKey("UP")) {
    buttons["UP"] = doc["UP"];
  }
  if (doc.containsKey("DOWN")) {
    buttons["DOWN"] = doc["DOWN"];
  }
  if (doc.containsKey("OUT")) {
    buttons["OUT"] = doc["OUT"];
  }
  if (doc.containsKey("IN")) {
    buttons["IN"] = doc["IN"];
  }

  controlMotor();
}

void setup() {
#if DEBUG_MODE
  Serial.begin(115200);
  delay(100);
#endif

  debugPrintln("ESP8266 Crane Motor Controller");

  // Initialize I2C communication
  Wire.begin();

  // Setup motor shield
  setupMotors();

  // Connect to WiFi and MQTT
  if (!connectToWiFi()) {
    return;
  }

setupOTA() ;

  connectToMQTT();

  debugPrintln("Setup complete");
}

void loop() {
  // Handle OTA Updates
  ArduinoOTA.handle();

  // Reconnect to WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    if (connectToWiFi()) {
      connectToMQTT();
    }
  }

  // Reconnect to MQTT if disconnected
  if (!mqttClient.connected()) {
    connectToMQTT();
  }

  // Process MQTT messages
  mqttClient.loop();

  // Small delay
  delay(10);
}


// Setup OTA Update
void setupOTA() {
  // Set hostname for OTA
  ArduinoOTA.setHostname(config.hostname);

  // Set OTA password
  ArduinoOTA.setPassword(config.ota_password);

  // OTA Event Handlers
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else  // U_SPIFFS
      type = "filesystem";

    // Stop motors during update
    stopAllMotors();

    debugPrintln("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    debugPrintln("\nOTA Update Complete");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    debugPrint("Progress: ");
    debugPrint(progress / (total / 100));
    debugPrintln("%");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    debugPrint("Error[");
    debugPrint(error);
    debugPrintln("]: ");

    if (error == OTA_AUTH_ERROR) debugPrintln("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) debugPrintln("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) debugPrintln("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) debugPrintln("Receive Failed");
    else if (error == OTA_END_ERROR) debugPrintln("End Failed");
  });

  // Start OTA
  ArduinoOTA.begin();
  debugPrintln("OTA Update Service Started");
}