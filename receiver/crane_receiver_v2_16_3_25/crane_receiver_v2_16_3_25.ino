#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <LOLIN_I2C_MOTOR.h>

// Debug mode - set to false to disable Serial output
#define DEBUG_MODE true

// OTA Configuration
#define HOSTNAME "esp8266-crane-controller"
#define OTA_PASSWORD ""  // Change this to a secure password

// LOLIN I2C Motor Shield Configuration
LOLIN_I2C_MOTOR motor1(0x20);  // First motor board at address 0x20
LOLIN_I2C_MOTOR motor2(0x21);  // Second motor board at address 0x21

// Motor speed settings
const uint8_t motorSpeed = 100;  // 0-100 range for LOLIN I2C Motor Shield

// Define movement types corresponding to the transmitter
const char* MOVEMENTS[] = {
  "ANTICLOCKWISE",  // Rotation counter-clockwise
  "DOWN",           // Move down
  "OUT",            // Extend outward
  "CLOCKWISE",      // Rotation clockwise
  "UP",             // Move up
  "IN"              // Retract inward
};
const int NUM_MOVEMENTS = 6;

// Motor state tracking
int motorStates[6] = {0, 0, 0, 0, 0, 0};  // 0: Stopped, 1: Forward, -1: Reverse

// Wi-Fi credentials
const char* WIFI_SSID = "the robot network";
const char* WIFI_PASSWORD = "isaacasimov";

// MQTT Broker settings
const char* MQTT_SERVER = "robotmqtt";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "public";
const char* MQTT_PASSWORD = "public";
const char* MQTT_CLIENT_ID = "Crane-Controller";
const char* MQTT_TOPIC = "crane/buttons";

// Network objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// --- Debugging Macros ---
#if DEBUG_MODE
#define debugPrintln(message) Serial.println(message)
#define debugPrint(message) Serial.print(message)
#else
#define debugPrintln(message)  // do nothing
#define debugPrint(message)    // do nothing
#endif

// Setup OTA Update
void setupOTA() {
  // Set hostname for OTA
  ArduinoOTA.setHostname(HOSTNAME);
  
  // Set OTA password
  ArduinoOTA.setPassword(OTA_PASSWORD);
  
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

// Function to connect to WiFi
void connectToWiFi() {
  debugPrintln("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);  // Explicitly set station mode for ESP8266
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait for connection (with timeout)
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    debugPrint(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    debugPrintln("");
    debugPrint("Connected to WiFi network with IP Address: ");
    debugPrintln(WiFi.localIP());
    
    // Setup OTA after successful WiFi connection
    setupOTA();
  } else {
    debugPrintln("");
    debugPrintln("Failed to connect to WiFi");
  }
}

// Configure motor shield
void setupMotors() {
  // Initialize First I2C Motor Shield
  if (motor1.MOTOR_STATUS() == false) {
    debugPrintln("First I2C Motor Shield not found at address 0x20. Check wiring.");
    while (1);
  }
  debugPrintln("First I2C Motor Shield found at address 0x20");

  // Initialize Second I2C Motor Shield
  if (motor2.MOTOR_STATUS() == false) {
    debugPrintln("Second I2C Motor Shield not found at address 0x21. Check wiring.");
    while (1);
  }
  debugPrintln("Second I2C Motor Shield found at address 0x21");

  // Initialize all motors to stop
  stopAllMotors();
}

// Stop all motors
void stopAllMotors() {
  // Stop motors on first board
  motor1.MOTOR_STOP(1);
  motor1.MOTOR_STOP(2);
  motor1.MOTOR_STOP(3);
  
  // Stop motors on second board
  motor2.MOTOR_STOP(1);
  motor2.MOTOR_STOP(2);
  motor2.MOTOR_STOP(3);
  
  // Reset motor states
  for (int i = 0; i < 6; i++) {
    motorStates[i] = 0;
  }
  
  debugPrintln("All motors stopped");
}

// Control a specific motor
void controlMotor(int motorIndex, int direction) {
  int boardNum;
  int motorNum;
  
  // Select the appropriate board and motor number
  if (motorIndex < 3) {
    boardNum = 1;  // First board
    motorNum = motorIndex + 1;
  } else {
    boardNum = 2;  // Second board
    motorNum = motorIndex - 2;
  }
  
  // Update motor state
  motorStates[motorIndex] = direction;
  
  if (direction == 1) {  // Forward
    if (boardNum == 1) {
      motor1.MOTOR_CCW(motorNum, motorSpeed);
    } else {
      motor2.MOTOR_CCW(motorNum, motorSpeed);
    }
    debugPrint("Motor ");
    debugPrint(motorIndex + 1);
    debugPrint(" on board ");
    debugPrint(boardNum);
    debugPrintln(" moving forward (CCW)");
  } 
  else if (direction == -1) {  // Reverse
    if (boardNum == 1) {
      motor1.MOTOR_CW(motorNum, motorSpeed);
    } else {
      motor2.MOTOR_CW(motorNum, motorSpeed);
    }
    debugPrint("Motor ");
    debugPrint(motorIndex + 1);
    debugPrint(" on board ");
    debugPrint(boardNum);
    debugPrintln(" moving reverse (CW)");
  } 
  else {  // Stop
    if (boardNum == 1) {
      motor1.MOTOR_STOP(motorNum);
    } else {
      motor2.MOTOR_STOP(motorNum);
    }
    debugPrint("Motor ");
    debugPrint(motorIndex + 1);
    debugPrint(" on board ");
    debugPrint(boardNum);
    debugPrintln(" stopped");
  }
}

// Function to connect to MQTT broker
void connectToMQTT() {
  // Only attempt to connect if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // Try to connect to MQTT broker
  debugPrintln("Connecting to MQTT broker...");
  if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
    debugPrintln("Connected to MQTT broker");
    
    // Subscribe to the button topic
    mqttClient.subscribe(MQTT_TOPIC);
    debugPrint("Subscribed to: ");
    debugPrintln(MQTT_TOPIC);
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
  
  // Convert payload to string
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  debugPrintln(message);
  
  // Parse JSON
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  // Check for parsing errors
  if (error) {
    debugPrint("deserializeJson() failed: ");
    debugPrintln(error.c_str());
    return;
  }
  
  // Process movement commands
  processMovements(doc);
}

// Process movement commands from JSON
void processMovements(JsonDocument& doc) {
  // Handle each movement type
  
  // Motor 1 - ANTICLOCKWISE/CLOCKWISE (rotation)
  if (doc.containsKey("ANTICLOCKWISE") && doc["ANTICLOCKWISE"] == 1) {
    controlMotor(0, 1);  // Run motor 1 forward
  } else if (doc.containsKey("CLOCKWISE") && doc["CLOCKWISE"] == 1) {
    controlMotor(0, -1);  // Run motor 1 reverse
  } else if (motorStates[0] != 0) {
    controlMotor(0, 0);  // Stop motor 1
  }
  
  // Motor 2 - DOWN/UP (lifting)
  if (doc.containsKey("DOWN") && doc["DOWN"] == 1) {
    controlMotor(1, 1);  // Run motor 2 forward
  } else if (doc.containsKey("UP") && doc["UP"] == 1) {
    controlMotor(1, -1);  // Run motor 2 reverse
  } else if (motorStates[1] != 0) {
    controlMotor(1, 0);  // Stop motor 2
  }
  
  // Motor 3 - OUT/IN (extension)
  if (doc.containsKey("OUT") && doc["OUT"] == 1) {
    controlMotor(2, 1);  // Run motor 3 forward
  } else if (doc.containsKey("IN") && doc["IN"] == 1) {
    controlMotor(2, -1);  // Run motor 3 reverse
  } else if (motorStates[2] != 0) {
    controlMotor(2, 0);  // Stop motor 3
  }
}

void setup() {
  // Initialize serial communication for debugging
  #if DEBUG_MODE
  Serial.begin(115200);
  delay(100);  // Short delay to ensure serial is ready
  #endif

  debugPrintln("ESP8266 Crane Motor Controller");
  
  // Initialize I2C communication
  Wire.begin();
  
  // Setup motor shield
  setupMotors();

  // Connect to WiFi and MQTT
  connectToWiFi();
  connectToMQTT();

  debugPrintln("Setup complete");
}

void loop() {
  // Handle OTA Updates
  ArduinoOTA.handle();
  
  
  // Reconnect to WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
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