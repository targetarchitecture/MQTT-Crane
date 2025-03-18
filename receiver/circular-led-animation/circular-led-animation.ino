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
const long centerBlinkInterval = 1000;  // 1000 milliseconds = 1 second
const long animationInterval = 60;      // 60 milliseconds between animation steps
bool centerLedState = false;            // Start with center LED off

void setup() {
  // Initialize the NeoPixel
  pixels.begin();
  pixels.setBrightness(100);  // Set brightness (0-255)
  pixels.clear();             // Set all pixels to 'off'
  pixels.show();              // Initialize all pixels to 'off'
}

void loop() {
  // Current time
  unsigned long currentMillis = millis();
  bool updateDisplay = false;

  // Check if it's time to toggle the center LED
  if (currentMillis - previousCenterBlinkMillis >= centerBlinkInterval) {
    // Save the last time the center LED was toggled
    previousCenterBlinkMillis = currentMillis;

    // Toggle center LED state
    centerLedState = !centerLedState;
    updateDisplay = true;
  }

  // Check if it's time to update the animation
  if (currentMillis - previousAnimationMillis >= animationInterval) {
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
}

void updateLEDs() {
  // Clear all LEDs
  pixels.clear();

  // Set center LED based on current state
  if (centerLedState) {
    pixels.setPixelColor(CENTER_LED, MID_RED);  // Green when on
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

  //put some delay in...
  delay(50);
}