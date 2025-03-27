#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Debug mode - set to false to disable Serial output
#define DEBUG_MODE true

// L9110S Motor Driver pins (A-1A, A-1B for each motor)
// Motor 1 - ANTICLOCKWISE/CLOCKWISE movement (rotation)
#define MOTOR1_A 4
#define MOTOR1_B 5

// Motor 2 - DOWN/UP movement (lifting)
#define MOTOR2_A 6
#define MOTOR2_B 7

// Motor 3 - OUT/IN movement (extension)
#define MOTOR3_A 8
#define MOTOR3_B 9

// PWM properties
const int freq = 5000;
const int resolution = 8;  // 8-bit resolution (0-255)
const int motorChannels[] = {0, 1, 2, 3, 4, 5};

// Motor speed settings (0-255)
const int motorSpeed = 200;  // Default motor speed

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
int motorStates[3] = {0, 0, 0};  // 0: Stopped, 1: Forward, -1: Reverse

// Wi-Fi credentials
const char* WIFI_SSID = "the robot network";
const char* WIFI_PASSWORD = "isaacasimov";

// MQTT Broker settings
const char* MQTT_SERVER = "robotmqtt";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "public";
const char* MQTT_PASSWORD = "public";
const char* MQTT_CLIENT_ID = "Crane_Controller";
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

// Function to connect to WiFi
void connectToWiFi() {
  debugPrintln("Connecting to WiFi...");
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
  } else {
    debugPrintln("");
    debugPrintln("Failed to connect to WiFi");
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

// Configure motor pins for PWM control
void setupMotors() {
  // Setup Motor 1 PWM channels (rotation)
  ledcSetup(motorChannels[0], freq, resolution);
  ledcSetup(motorChannels[1], freq, resolution);
  ledcAttachPin(MOTOR1_A, motorChannels[0]);
  ledcAttachPin(MOTOR1_B, motorChannels[1]);

  // Setup Motor 2 PWM channels (lifting)
  ledcSetup(motorChannels[2], freq, resolution);
  ledcSetup(motorChannels[3], freq, resolution);
  ledcAttachPin(MOTOR2_A, motorChannels[2]);
  ledcAttachPin(MOTOR2_B, motorChannels[3]);

  // Setup Motor 3 PWM channels (extension)
  ledcSetup(motorChannels[4], freq, resolution);
  ledcSetup(motorChannels[5], freq, resolution);
  ledcAttachPin(MOTOR3_A, motorChannels[4]);
  ledcAttachPin(MOTOR3_B, motorChannels[5]);

  // Initialize all motors to stop
  stopAllMotors();
}

// Stop all motors
void stopAllMotors() {
  for (int i = 0; i < 6; i++) {
    ledcWrite(motorChannels[i], 0);
  }
  
  // Reset motor states
  for (int i = 0; i < 3; i++) {
    motorStates[i] = 0;
  }
  
  debugPrintln("All motors stopped");
}

// Control a specific motor
void controlMotor(int motorIndex, int direction) {
  // Motor pairs (A and B pins)
  int channelA = motorIndex * 2;
  int channelB = motorIndex * 2 + 1;
  
  // Update motor state
  motorStates[motorIndex] = direction;
  
  if (direction == 1) {  // Forward
    ledcWrite(channelA, motorSpeed);
    ledcWrite(channelB, 0);
    debugPrint("Motor ");
    debugPrint(motorIndex + 1);
    debugPrintln(" moving forward");
  } 
  else if (direction == -1) {  // Reverse
    ledcWrite(channelA, 0);
    ledcWrite(channelB, motorSpeed);
    debugPrint("Motor ");
    debugPrint(motorIndex + 1);
    debugPrintln(" moving reverse");
  } 
  else {  // Stop
    ledcWrite(channelA, 0);
    ledcWrite(channelB, 0);
    debugPrint("Motor ");
    debugPrint(motorIndex + 1);
    debugPrintln(" stopped");
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

  debugPrintln("ESP32 Crane Motor Controller");
  
  // Setup motor control pins
  setupMotors();

  // Connect to WiFi and MQTT
  connectToWiFi();
  connectToMQTT();

  debugPrintln("Setup complete");
}

void loop() {
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