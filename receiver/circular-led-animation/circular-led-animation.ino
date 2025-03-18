#include <Adafruit_NeoPixel.h>

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
const uint32_t RED = 0xFF0000;

void setup() {
  // Initialize the NeoPixel
  pixels.begin();
  pixels.setBrightness(70); // Set brightness to 70% (0-255)
  pixels.clear(); // Set all pixels to 'off'
  pixels.show();  // Initialize all pixels to 'off'
}

void loop() {
  // Clear all LEDs
  pixels.clear();
  
  // Light up only the current LED with red
  pixels.setPixelColor(currentLED, RED);
  
  // Send the updated colors to the LEDs
  pixels.show();
  
  // Wait a moment with the LED on
  delay(200);
  
  // Turn off all LEDs
  pixels.clear();
  pixels.show();
  
  // Wait a moment with all LEDs off
  delay(100);
  
  // Move to the next LED in the circle (1-6)
  currentLED = (currentLED % 6) + 1;
}