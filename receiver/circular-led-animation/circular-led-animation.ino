#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>  // Add MQTT library

// WiFi credentials
const char* ssid = "the robot network";
const char* password = "isaacasimov";

// OTA hostname (how it will appear in Arduino IDE)
const char* otaHostname = "LED-Controller";

// MQTT Configuration
const char* mqtt_server = "robotmqtt";  // Change to your MQTT broker IP
const int mqtt_port = 1883;                 // Default MQTT port
const char* mqtt_client_id = "LED-Controller";
const char* mqtt_username = "public";             // Leave empty if no credentials needed
const char* mqtt_password = "public";             // Leave empty if no credentials needed
const char* mqtt_topic_sub = "crane/buttons";

// Define which pin the LED is connected to
// On the Lolin RGB Shield, this is typically D4 (GPIO2)
#define LED_PIN D4
// Define how many LEDs you have (1 in center, 6 in circle)
#define LED_COUNT 7
// Center LED is index 0, surrounding LEDs are 1-6
#define CENTER_LED 0
// Initialize the NeoPixel library
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Current active LED in the circle (1-6)
int currentLED = 1;
// Red color for the moving LED
const uint32_t BRIGHT_RED = 0xFF0000;
const uint32_t MID_RED = 0xA00000;
const uint32_t DIM_RED = 0x500000;
const uint32_t VERY_DIM_RED = 0x200000;
// Number of positions in the animation sequence
const int NUM_POSITIONS = 12;
// How many steps to complete one full cycle around the circle
const int STEPS_PER_CYCLE = 12;

// Timing variables for non-blocking operation
unsigned long previousCenterBlinkMillis = 0;
unsigned long previousAnimationMillis = 0;
unsigned long previousOTACheckMillis = 0;
unsigned long previousMQTTReconnectMillis = 0;

const long centerBlinkInterval = 1000;  // 1000 milliseconds = 1 second
const long animationInterval = 60;      // 60 milliseconds between animation steps
const long otaCheckInterval = 500;      // Check for OTA updates every 500ms
const long mqttReconnectInterval = 5000; // Try to reconnect to MQTT every 5 seconds

bool centerLedState = false;  // Start with center LED off
bool animationRunning = true; // Control flag for animation

// Different animation modes
enum OperationMode {
  MODE_NORMAL,      // Default rotating animation
  MODE_STATIC,      // Static color display
  MODE_RAINBOW,     // Rainbow effect
  MODE_OFF          // All LEDs off
};

OperationMode currentMode = MODE_NORMAL;
uint32_t staticColor = BRIGHT_RED; // Default static color

// Initialize WiFi and MQTT clients
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setupWiFi() {
  // Connect to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  Serial.println("MQTT setup complete");
}

void reconnectMQTT() {
  // Loop until we're reconnected
  if (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Subscribe to control topic
      mqttClient.subscribe(mqtt_topic_sub);
      // Publish a connection message
      //mqttClient.publish(mqtt_topic_status, "LED Controller connected", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  // Convert payload to string for easier handling
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println(message);
  
  // Process the command
  // Format examples:
  // "mode:normal" - Set to normal animation mode
  // "mode:static:FF0000" - Set to static color (hexadecimal RGB)
  // "mode:rainbow" - Set to rainbow mode
  // "mode:off" - Turn all LEDs off
  // "brightness:100" - Set brightness (0-255)
  
  if (message.startsWith("mode:")) {
    String mode = message.substring(5);
    if (mode.startsWith("normal")) {
      currentMode = MODE_NORMAL;
      animationRunning = true;
      Serial.println("Mode set to normal animation");
    } 
    else if (mode.startsWith("static")) {
      currentMode = MODE_STATIC;
      animationRunning = false;
      // Check if color is specified
      if (mode.indexOf(":") > 0) {
        String colorHex = mode.substring(mode.lastIndexOf(":") + 1);
        // Convert hex string to color
        uint32_t color = strtol(colorHex.c_str(), NULL, 16);
        staticColor = color;
        Serial.print("Static color set to: ");
        Serial.println(colorHex);
      }
      updateStaticColor(); // Apply the static color immediately
    }
    else if (mode.startsWith("rainbow")) {
      currentMode = MODE_RAINBOW;
      animationRunning = true;
      Serial.println("Mode set to rainbow");
    }
    else if (mode.startsWith("off")) {
      currentMode = MODE_OFF;
      animationRunning = false;
      pixels.clear();
      pixels.show();
      Serial.println("All LEDs turned off");
    }
  }
  else if (message.startsWith("brightness:")) {
    int brightness = message.substring(11).toInt();
    if (brightness >= 0 && brightness <= 255) {
      pixels.setBrightness(brightness);
      pixels.show();
      Serial.print("Brightness set to: ");
      Serial.println(brightness);
    }
  }
  
  // Publish status update
  //String statusMsg = "Status: ";
  //statusMsg += getModeString();
  //mqttClient.publish(mqtt_topic_status, statusMsg.c_str());
}

// String getModeString() {
//   switch (currentMode) {
//     case MODE_NORMAL: return "normal";
//     case MODE_STATIC: return "static";
//     case MODE_RAINBOW: return "rainbow";
//     case MODE_OFF: return "off";
//     default: return "unknown";
//   }
// }

void updateStaticColor() {
  pixels.clear();
  for (int i = 0; i < LED_COUNT; i++) {
    pixels.setPixelColor(i, staticColor);
  }
  pixels.show();
}

// Rainbow cycle for all LEDs
void rainbowCycle() {
  static uint16_t j = 0;
  for (int i = 0; i < LED_COUNT; i++) {
    pixels.setPixelColor(i, wheel(((i * 256 / LED_COUNT) + j) & 255));
  }
  pixels.show();
  j = (j + 1) % 256; // Cycle through colors
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t wheel(byte wheelPos) {
  wheelPos = 255 - wheelPos;
  if (wheelPos < 85) {
    return pixels.Color(255 - wheelPos * 3, 0, wheelPos * 3);
  }
  if (wheelPos < 170) {
    wheelPos -= 85;
    return pixels.Color(0, wheelPos * 3, 255 - wheelPos * 3);
  }
  wheelPos -= 170;
  return pixels.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

void setupOTA() {
  // Set hostname for OTA
  ArduinoOTA.setHostname(otaHostname);
  
  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
    
    // Turn off all LEDs during update
    pixels.clear();
    pixels.show();
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    // Signal completion with all LEDs green
    for (int i = 0; i < LED_COUNT; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    }
    pixels.show();
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    
    // Visual progress indicator on LEDs
    int ledsToLight = map(progress, 0, total, 0, LED_COUNT);
    pixels.clear();
    for (int i = 0; i < ledsToLight; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 255)); // Blue for progress
    }
    pixels.show();
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
    
    // Signal error with all LEDs red
    for (int i = 0; i < LED_COUNT; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    }
    pixels.show();
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

void setup() {
  // Initialize Serial for debugging and I2C scan results
  Serial.begin(115200);
  Serial.println("\nI2C Scanner and LED Controller with OTA and MQTT");

  // Initialize I2C
  Wire.begin();

  pinMode(LED_PIN, OUTPUT);

  // Initialize the NeoPixel
  pixels.begin();
  pixels.setBrightness(100);  // Set brightness (0-255)
  pixels.clear();             // Set all pixels to 'off'
  pixels.show();              // Initialize all pixels to 'off'

  // Setup WiFi connection
  setupWiFi();
  
  // Setup MQTT
  setupMQTT();
  
  // Setup OTA updates
  setupOTA();

  // Scan the I2C bus
  scanI2CBus();
}

void loop() {
  // Current time
  unsigned long currentMillis = millis();
  bool updateDisplay = false;

  // Handle OTA updates
  if (currentMillis - previousOTACheckMillis >= otaCheckInterval) {
    previousOTACheckMillis = currentMillis;
    ArduinoOTA.handle();
  }

  // Check MQTT connection and reconnect if needed
  if (!mqttClient.connected()) {
    if (currentMillis - previousMQTTReconnectMillis >= mqttReconnectInterval) {
      previousMQTTReconnectMillis = currentMillis;
      reconnectMQTT();
    }
  } else {
    // Handle MQTT messages
    mqttClient.loop();
  }

  // Handle different operation modes
  switch (currentMode) {
    case MODE_NORMAL:
      // Check if it's time to toggle the center LED
      if (currentMillis - previousCenterBlinkMillis >= centerBlinkInterval) {
        // Save the last time the center LED was toggled
        previousCenterBlinkMillis = currentMillis;

        // Toggle center LED state
        centerLedState = !centerLedState;
        updateDisplay = true;
      }

      // Check if it's time to update the animation
      if (animationRunning && currentMillis - previousAnimationMillis >= animationInterval) {
        // Save the last time the animation was updated
        previousAnimationMillis = currentMillis;

        // Advance to next position
        currentLED = (currentLED % STEPS_PER_CYCLE) + 1;
        updateDisplay = true;
      }

      // Only update the display if something changed
      if (updateDisplay) {
        updateLEDs();
      }
      break;
      
    case MODE_RAINBOW:
      // Update rainbow animation at animation interval
      if (currentMillis - previousAnimationMillis >= animationInterval) {
        previousAnimationMillis = currentMillis;
        rainbowCycle();
      }
      break;
      
    case MODE_STATIC:
      // Static color - no need to update unless changed via MQTT
      break;
      
    case MODE_OFF:
      // All LEDs off - no need to update
      break;
  }
}

void updateLEDs() {
  // Clear all LEDs
  pixels.clear();

  // Set center LED based on current state
  if (centerLedState) {
    pixels.setPixelColor(CENTER_LED, MID_RED);  // MID_RED when on
  }

  // Calculate LED positions for the swoosh
  int mainLED = ((currentLED - 1) / 2) + 1;

  // Create swishy effect - the light appears to move more fluidly between LEDs
  if (currentLED % 2 == 1) {
    // For odd positions - main bright spot with trailing fade
    pixels.setPixelColor(mainLED, BRIGHT_RED);

    int prevLED = (mainLED == 1) ? 6 : mainLED - 1;
    int prevPrevLED = (prevLED == 1) ? 6 : prevLED - 1;

    pixels.setPixelColor(prevLED, MID_RED);
    pixels.setPixelColor(prevPrevLED, DIM_RED);

    int prevPrevPrevLED = (prevPrevLED == 1) ? 6 : prevPrevLED - 1;
    pixels.setPixelColor(prevPrevPrevLED, VERY_DIM_RED);
  } else {
    // For even positions - transitioning between LEDs (looks like motion blur)
    int nextLED = (mainLED == 6) ? 1 : mainLED + 1;

    pixels.setPixelColor(mainLED, BRIGHT_RED);
    pixels.setPixelColor(nextLED, MID_RED);

    int prevLED = (mainLED == 1) ? 6 : mainLED - 1;
    int prevPrevLED = (prevLED == 1) ? 6 : prevLED - 1;

    pixels.setPixelColor(prevLED, DIM_RED);
    pixels.setPixelColor(prevPrevLED, VERY_DIM_RED);
  }

  // Send the updated colors to the LEDs
  pixels.show();
}

void scanI2CBus() {
  byte error, address;
  int deviceCount = 0;

  Serial.println("Scanning I2C bus...");

  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println();

      deviceCount++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s)");
  }

  Serial.println();  // Add a blank line for readability
}