#include <Adafruit_NeoPixel.h>
#include "esp_sleep.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Debug mode - set to false to disable Serial output
#define DEBUG_MODE true

// Define the WS2812 LED pin
#define LED_PIN 21
#define NUM_LEDS 1

// Define the button pins
const int buttonPins[] = { 1, 2, 3, 4, 5, 6 };
const int numButtons = 6;

const char* buttonMovements[] = {
  "ANTICLOCKWISE",  // Button 1
  "DOWN",           // Button 2
  "OUT",            // Button 3
  "CLOCKWISE",      // Button 4
  "UP",             // Button 5
  "IN"              // Button 6
};

// Define colors for each button (in RGB format)
uint32_t buttonColors[] = {
  0xFFAA00,  // Orange (Button 1 - ANTICLOCKWISE)
  0xFF0000,  // Dark Green (Button 2 - DOWN)
  0x0000FF,  // Bright Blue (Button 3 - OUT)
  0x00AA00,  // Bright Red (Button 4 - CLOCKWISE)
  0x8800FF,  // Purple (Button 5 - UP)
  0x00AAAA   // Teal (Button 6 - IN)
};

// Variables to keep track of button states
int buttonStates[6] = { LOW, LOW, LOW, LOW, LOW, LOW };
int lastButtonStates[6] = { LOW, LOW, LOW, LOW, LOW, LOW };

unsigned long lastDebounceTime[6] = { 0, 0, 0, 0, 0, 0 };
const unsigned long debounceDelay = 50;  // Debounce time in milliseconds

// MQTT periodic update variables
unsigned long lastMqttUpdateTime = 0;
const unsigned long mqttUpdateInterval = 500;  // Send MQTT update every 500ms (half second)

// Sleep parameters
const unsigned long sleepDelay = 3000;  // Time in ms before going to sleep after button release
unsigned long buttonReleaseTime = 0;    // Time when the last button was released
bool buttonPressed = false;             // Flag to track if any button is pressed
bool readyToSleep = true;               // Flag to indicate ready to enter sleep

// Wi-Fi credentials
const char* WIFI_SSID = "the robot network";
const char* WIFI_PASSWORD = "isaacasimov";

// MQTT Broker settings
const char* MQTT_SERVER = "robotmqtt";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "public";      // Optional, leave as is if no authentication
const char* MQTT_PASSWORD = "public";  // Optional, leave as is if no authentication
const char* MQTT_CLIENT_ID = "Crane_Remote";
const char* MQTT_TOPIC = "crane/buttons";

// Network objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Initialize the NeoPixel library
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

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
    delay(100);
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

  // Try to connect to MQTT broker
  debugPrintln("Connecting to MQTT broker...");
  if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
    debugPrintln("Connected to MQTT broker");
  } else {
    debugPrint("Failed to connect to MQTT broker, rc=");
    debugPrintln(mqttClient.state());
  }
}

// Function to send button states via MQTT
void sendButtonStates() {
  // Create a JSON document
  StaticJsonDocument<200> doc;

  // Add button states to the JSON document
  for (int i = 0; i < numButtons; i++) {
    //doc["button" + String(i + 1)] = buttonStates[i] == HIGH ? 1 : 0;
    doc[buttonMovements[i]] = buttonStates[i] == HIGH ? 1 : 0;
  }

  // Serialize the JSON document
  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);

  // Send the message
  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_TOPIC, jsonBuffer);
    debugPrint("MQTT message sent: ");
    debugPrintln(jsonBuffer);
  } else {
    debugPrintln("MQTT not connected, reconnecting...");
    connectToMQTT();
    // Try to send again after reconnecting
    if (mqttClient.connected()) {
      mqttClient.publish(MQTT_TOPIC, jsonBuffer);
      debugPrint("MQTT message sent after reconnect: ");
      debugPrintln(jsonBuffer);
    }
  }
}

// Get wake cause and button that triggered wake up
void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT1:
      {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        debugPrintln("Wakeup caused by external signal using EXT1");

        // Determine which button woke up the device
        for (int i = 0; i < numButtons; i++) {
          if (wakeup_pin_mask & (1ULL << buttonPins[i])) {
            debugPrint("Wakeup button: GPIO ");
            debugPrintln(buttonPins[i]);

            // Set the LED to the color of the button that woke up the device
            strip.setPixelColor(0, buttonColors[i]);
            strip.show();
            break;
          }
        }
        break;
      }
    case ESP_SLEEP_WAKEUP_TIMER:
      debugPrintln("Wakeup caused by timer");
      break;
    default:
#if DEBUG_MODE
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
#endif
      break;
  }
}

void setup() {

  // Initialize serial communication for debugging
#if DEBUG_MODE
  Serial.begin(115200);
  delay(100);  // Short delay to ensure serial is ready
#endif

  debugPrintln("ESP32-S3 WS2812 LED Control with Deep Sleep and MQTT");

  // Initialize the LED strip
  strip.begin();
  strip.setBrightness(50);  // Set brightness (0-255)
  strip.show();             // Initialize all pixels to 'off'

  // Set initial LED color to black/off
  strip.setPixelColor(0, strip.Color(0, 0, 0));  // Explicitly set to black (R=0, G=0, B=0)
  strip.show();

  // Initialize button pins as inputs (externally pulled down)
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT);
    debugPrint("Button on GPIO ");
    debugPrint(buttonPins[i]);
    debugPrintln(" initialized");
  }

  // Check if the ESP32 woke up from deep sleep
  printWakeupReason();

  // Connect to WiFi and MQTT
  connectToWiFi();
  connectToMQTT();

  // Configure the wake-up source (all buttons)
  uint64_t mask = 0;
  for (int i = 0; i < numButtons; i++) {
    mask |= (1ULL << buttonPins[i]);
  }

  // Configure EXT1 wake sources (any high level on specified GPIOs will wake up the ESP32)
  esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);

  debugPrintln("Setup complete");
}

void loop() {

  //set variables
  bool anyButtonPressed = false;
  bool buttonStateChanged = false;
  int activeButton = -1;

  // Check each button
  for (int i = 0; i < numButtons; i++) {
    // Read the button state
    int reading = digitalRead(buttonPins[i]);

    // Check if the button state has changed
    if (reading != lastButtonStates[i]) {
      // Reset the debounce timer
      lastDebounceTime[i] = millis();
    }

    // If enough time has passed, consider the state change valid
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
    // If the button state has changed
    if (reading != buttonStates[i]) {
      buttonStates[i] = reading;
      buttonStateChanged = true;

      // If the button is pressed (HIGH with external pull-down)
      if (buttonStates[i] == HIGH) {
        debugPrint("Button ");
        debugPrint(i + 1);
        debugPrint(" on GPIO ");
        debugPrint(buttonPins[i]);
        debugPrintln(" pressed");

        activeButton = i;
        anyButtonPressed = true;
        buttonPressed = true;
        readyToSleep = false;

        // Light up the LED with the color for this button
        strip.setPixelColor(0, buttonColors[i]);
        strip.show();
      }
    }
    }

    // Keep track if any button is currently pressed
    if (buttonStates[i] == HIGH) {
      anyButtonPressed = true;
    }

    // Save the current reading for the next loop
    lastButtonStates[i] = reading;
  }

  // If button state changed, send MQTT message
  if (buttonStateChanged) {
    sendButtonStates();
  }

  // If no button is currently pressed but one was pressed before, track release time
  if (!anyButtonPressed && buttonPressed) {
    buttonPressed = false;
    buttonReleaseTime = millis();
    readyToSleep = true;

    // Turn off the LED when button is released
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();

    // Send final button states before preparing for sleep
    sendButtonStates();

    debugPrintln("All buttons released, preparing for sleep");
  }

  // Keep MQTT connection alive
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop();

  // Send periodic MQTT updates if any button is pressed
  if (anyButtonPressed && (millis() - lastMqttUpdateTime > mqttUpdateInterval)) {
    sendButtonStates();
    lastMqttUpdateTime = millis();
  }

  // If it's time to sleep and we're ready
  if (readyToSleep && (millis() - buttonReleaseTime > sleepDelay)) {
    debugPrintln("Going to deep sleep now");
#if DEBUG_MODE
    Serial.flush();
#endif

    // Make sure LED is off before sleep
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();

    // Enter deep sleep
    esp_deep_sleep_start();
  }

  // Short delay for the loop
  delay(50);
  debugPrint(".");
}
