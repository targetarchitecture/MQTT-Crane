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

// Map to store button states and their corresponding values
// 0 = inactive/off, 1 = active/on
std::map<std::string, int> buttons = {
  { "ANTICLOCKWISE", 0 },  // Controls rotation counter-clockwise
  { "DOWN", 0 },           // Controls downward movement
  { "OUT", 0 },            // Controls extension outward
  { "CLOCKWISE", 0 },      // Controls rotation clockwise
  { "UP", 0 },             // Controls upward movement
  { "IN", 0 }              // Controls retraction inward
};

// Debug mode flag - enables/disables Serial output for debugging
#define DEBUG_MODE true

// Configuration structure to store all system settings
struct Config {
  // OTA (Over-The-Air) Update Configuration
  char hostname[32] = "crane-controller";  // Device hostname for network identification
  char ota_password[32] = "";              // Password for OTA updates

  // Motor Control Configuration
  uint8_t motor_speed = 100;  // Default motor speed (0-100%)

  // WiFi Network Configuration
  char wifi_ssid[32] = "the robot network";  // WiFi network name
  char wifi_password[32] = "isaacasimov";    // WiFi network password

  // MQTT Communication Configuration
  char mqtt_server[32] = "robotmqtt";            // MQTT broker address
  int mqtt_port = 1883;                          // MQTT broker port
  char mqtt_user[32] = "public";                 // MQTT username
  char mqtt_password[32] = "public";             // MQTT password
  char mqtt_client_id[32] = "crane-controller";  // Unique MQTT client identifier
  char mqtt_topic[32] = "crane/buttons";         // MQTT topic for receiving commands

  // System Timing Configuration
  int wifi_timeout = 20;    // WiFi connection timeout (seconds)
  int mqtt_keepalive = 60;  // MQTT keepalive interval (seconds)
} config;

// Initialize motor control objects with their I2C addresses
LOLIN_I2C_MOTOR motor1(0x20);  // First motor board at I2C address 0x20
LOLIN_I2C_MOTOR motor2(0x21);  // Second motor board at I2C address 0x21

// Network communication objects
WiFiClient espClient;                // WiFi client for network communication
PubSubClient mqttClient(espClient);  // MQTT client for message handling

// System state tracking
bool isEmergencyStop = false;  // Emergency stop flag for safety
unsigned long lastMessageTime = 0;  // Timestamp of last received MQTT message
const unsigned long MESSAGE_TIMEOUT = 1000;  // Timeout period in milliseconds (1 second)

// Debug output macros - only active when DEBUG_MODE is true
#if DEBUG_MODE
#define debugPrintln(message) Serial.println(message)
#define debugPrint(message) Serial.print(message)
#else
#define debugPrintln(message)  // Debug messages disabled
#define debugPrint(message)    // Debug messages disabled
#endif

// Attempts to connect to WiFi network with timeout
bool connectToWiFi() {
  debugPrintln("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);  // Set WiFi mode to Station (client)
  WiFi.begin(config.wifi_ssid, config.wifi_password);

  // Wait for connection with timeout
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < config.wifi_timeout) {
    delay(500);
    debugPrint(".");
    timeout++;
  }

  // Report connection status
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

// Initialize and verify motor shield connections
void setupMotors() {
  // Wait for first motor board to be ready
  while (motor1.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) {
    motor1.getInfo();
    delay(5);
  }

  // Wait for second motor board to be ready
  while (motor2.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) {
    motor2.getInfo();
    delay(5);
  }
}

// Emergency stop function - stops all motors immediately
void stopAllMotors() {
  // Stop all motors on both boards
  motor1.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);
  motor2.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);

  debugPrintln("All motors stopped");
}

// Controls motor movement based on button states
void controlMotor() {
  // Emergency stop check - stops all motors if activated
  if (isEmergencyStop) {
    stopAllMotors();
    return;
  }

  // Rotation control (Motor A on first board)
  //FIX THE TRANSMITTER
  if (buttons["ANTICLOCKWISE"] == 1 && buttons["CLOCKWISE"] == 1) {
    motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CCW);
    motor1.changeDuty(MOTOR_CH_A, config.motor_speed);
  }
  if (buttons["ANTICLOCKWISE"] == 0 && buttons["CLOCKWISE"] == 1) {
    motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CW);
    motor1.changeDuty(MOTOR_CH_A, config.motor_speed);
  }
  if (buttons["ANTICLOCKWISE"] == 0 && buttons["CLOCKWISE"] == 0) {
    motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_STOP);
  }

  // Vertical movement control (Motor B on first board)
  if (buttons["UP"] == 1 && buttons["DOWN"] == 0) {
    motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CCW);
    motor1.changeDuty(MOTOR_CH_B, config.motor_speed);
  }
  if (buttons["UP"] == 0 && buttons["DOWN"] == 1) {
    motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
    motor1.changeDuty(MOTOR_CH_B, config.motor_speed);
  }
  if (buttons["UP"] == 0 && buttons["DOWN"] == 0) {
    motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_STOP);
  }


  // Extension control (Motor A on second board)
  if (buttons["IN"] == 1 && buttons["OUT"] == 0) {
    motor2.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CCW);
    motor2.changeDuty(MOTOR_CH_A, config.motor_speed);
  }
  if (buttons["UP"] == 0 && buttons["OUT"] == 1) {
    motor2.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CW);
    motor2.changeDuty(MOTOR_CH_A, config.motor_speed);
  }
  if (buttons["UP"] == 0 && buttons["OUT"] == 0) {
    motor2.changeStatus(MOTOR_CH_A, MOTOR_STATUS_STOP);
  }
}

// Establishes connection to MQTT broker
void connectToMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  mqttClient.setServer(config.mqtt_server, config.mqtt_port);
  mqttClient.setCallback(mqttCallback);

  debugPrintln("Connecting to MQTT broker...");
  if (mqttClient.connect(config.mqtt_client_id, config.mqtt_user, config.mqtt_password)) {
    debugPrintln("Connected to MQTT broker");

    // Subscribe to command topic
    mqttClient.subscribe(config.mqtt_topic);
    debugPrint("Subscribed to: ");
    debugPrintln(config.mqtt_topic);
  } else {
    debugPrint("Failed to connect to MQTT broker, rc=");
    debugPrintln(mqttClient.state());
  }
}

// Handles incoming MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Update last message time
  lastMessageTime = millis();
  
  debugPrint("Message arrived on topic: ");
  debugPrint(topic);
  debugPrint(". Message: ");

  // Convert payload to string
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  debugPrintln(message);

  // Check for emergency stop command
  if (message.indexOf("EMERGENCY_STOP") >= 0) {
    isEmergencyStop = true;
    debugPrintln("Emergency stop activated");
    return;
  }

  // Parse JSON message for movement commands
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    debugPrint("deserializeJson() failed: ");
    debugPrintln(error.c_str());
    return;
  }

  // Update button states from JSON message
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

  // Execute motor control based on updated button states
  controlMotor();
}

// System initialization
void setup() {
#if DEBUG_MODE
  Serial.begin(115200);  // Initialize serial communication for debugging
  delay(100);            // Wait for serial to stabilize
#endif

  debugPrintln("ESP8266 Crane Motor Controller");

  // Initialize I2C communication for motor control
  Wire.begin();

  // Initialize motor shields
  setupMotors();

  // Initialize last message time
  lastMessageTime = millis();

  // Connect to network and MQTT
  if (!connectToWiFi()) {
    return;
  }

  // Setup OTA update capability
  setupOTA();

  // Connect to MQTT broker
  connectToMQTT();

  debugPrintln("Setup complete");
}

// Main program loop
void loop() {
  // Handle any pending OTA updates
  ArduinoOTA.handle();

  // Check for message timeout
  if (millis() - lastMessageTime > MESSAGE_TIMEOUT) {
    // Reset all button states
    buttons["ANTICLOCKWISE"] = 0;
    buttons["CLOCKWISE"] = 0;
    buttons["UP"] = 0;
    buttons["DOWN"] = 0;
    buttons["OUT"] = 0;
    buttons["IN"] = 0;
    
    // Stop all motors
    stopAllMotors();
    
    // Reset emergency stop flag
    isEmergencyStop = false;
  }

  // Maintain WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    if (connectToWiFi()) {
      connectToMQTT();
    }
  }

  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    connectToMQTT();
  }

  // Process any pending MQTT messages
  mqttClient.loop();

  // Small delay to prevent overwhelming the system
  delay(10);
}

// Configure Over-The-Air update functionality
void setupOTA() {
  // Set device hostname for network identification
  ArduinoOTA.setHostname(config.hostname);

  // Set OTA update password
  ArduinoOTA.setPassword(config.ota_password);

  // Configure OTA event handlers
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else  // U_SPIFFS
      type = "filesystem";

    // Ensure motors are stopped during update
    stopAllMotors();

    debugPrintln("Start updating " + type);
  });

  // Handle OTA completion
  ArduinoOTA.onEnd([]() {
    debugPrintln("\nOTA Update Complete");
  });

  // Display update progress
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    debugPrint("Progress: ");
    debugPrint(progress / (total / 100));
    debugPrintln("%");
  });

  // Handle OTA errors
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

  // Start OTA service
  ArduinoOTA.begin();
  debugPrintln("OTA Update Service Started");
}