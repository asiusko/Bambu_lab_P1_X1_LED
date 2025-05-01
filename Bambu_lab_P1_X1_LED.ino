#include <AiEsp32RotaryEncoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <arduinoFFT.h>

// buzer
#define BUZZER_PIN 1

// encoder
#define ROTARY_ENCODER_NOISE 10
#define ROTARY_ENCODER_A_PIN 5         // CLK (A pin)
#define ROTARY_ENCODER_B_PIN 6         // DT (B pin)
#define ROTARY_ENCODER_BUTTON_PIN 7    // SW (button pin)
#define ROTARY_ENCODER_VCC_PIN -1      // VCC if microcontroller VCC (then set ROTARY_ENCODER_VCC_PIN -1)
#define ROTARY_ENCODER_STEPS 4         // depending on your encoder - try 1,2 or 4 to get expected behaviour
#define ROTARY_ENCODER_NOISE 10
#define ROTARY_ENCODER_MIN_VALUE 0
#define ROTARY_ENCODER_MAX_VALUE 10

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

// oled
#define OLED_RESET -1
#define OLED_WIDTH 128
#define OLED_HEIGHT 32

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// LED strip configuration
#define LED_PIN 2
#define NUM_LEDS 48
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Microphone and FFT
#define AUDIO_IN_PIN 0
#define FFT_FILTERED_NOISE 5000
#define FFT_AMPLITUDE 1      // Depending on your audio source level, you may need to alter this value. Can be used as a 'sensitivity' control.
#define MAX_BANDS 16
#define FFT_SAMPLES 256         // Number of FFT samples (must be a power of 2)
#define FFT_SAMPLING_FREQ 12000 // Sampling frequency in Hz
#define MAX_BAND_VALUE 16

// 8 bands
// int peak[MAX_BANDS] = { 0, 0, 0, 0, 0, 0, 0, 0};
// int oldBandValues[MAX_BANDS] = { 0, 0, 0, 0, 0, 0, 0, 0};
// int bandValues[MAX_BANDS] = { 0, 0, 0, 0, 0, 0, 0, 0};
// 16 band
int peak[MAX_BANDS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int oldBandValues[MAX_BANDS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int bandValues[MAX_BANDS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
// 32 band
// int peak[MAX_BANDS] =           { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
// int oldBandValues[MAX_BANDS] =  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
// int bandValues[MAX_BANDS] =     { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
unsigned int sampling_period_us;
unsigned long newTime;
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, FFT_SAMPLING_FREQ);

// General variables
int encoderMode = 3;
int currentBrightnessIndex = 2;
int currentTemperatureIndex = 1;
int currentEqualizerIndex = 1;
int currentSensitivityIndex = 1;
int brightness = 20;
const int colorTemperatures[] = {1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500};

void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

void setup() {
  delay(2000);
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;
  }

  // oled 132x28
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  displayLEDprops();
  display.display();

  // rotary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(ROTARY_ENCODER_MIN_VALUE, ROTARY_ENCODER_MAX_VALUE, true);  //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
   * in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
   * without acceleration you need long time to get to that number
   * Using acceleration, faster you turn, faster will the value raise.
   * For fine tuning slow down.
   */
  rotaryEncoder.disableAcceleration();  //acceleration is now enabled by default - disable if you dont need it
                                        //rotaryEncoder.setAcceleration(250); //or set the value - larger number = more acceleration; 0 or 1 means disabled acceleration
  rotaryEncoder.setEncoderValue(0);

  // LEDS WS2812b
  strip.begin();
  strip.setBrightness(brightness);
  strip.show();
  displayTemperature();

  // Mic
  pinMode(AUDIO_IN_PIN, INPUT);

  // buzer
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  rotaryLoop();

  if (currentEqualizerIndex) {
    buildEqualizer();
  } else {
    displayTemperature();
  }

  displayLEDprops();
}

void buzzTimes(int counter, int freq = 750) {
   for (int iterator = 0; iterator < counter; iterator++) {
    tone(BUZZER_PIN, freq, 50);
    delay(250);
   }
}

void oledLogs(String value) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(value);
  display.display();
}

void displayLEDprops() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(String(encoderMode == 0 ? "> " : " "));
  display.println("brightness: " + String(brightness));
  display.print(String(encoderMode == 1 ? "> " : " "));
  display.println("temp gamma: " + String(colorTemperatures[currentTemperatureIndex]));
  display.print(String(encoderMode == 2 ? "> " : " "));
  display.println("equalizer: " + String(currentEqualizerIndex));
  display.print(String(encoderMode == 3 ? "> " : " "));
  display.println("Mic sensitivity: " + String(currentSensitivityIndex));
  display.display();
}

void displayTemperature() {
  // Get RGB values for the selected temperature
  uint32_t color = colorTemperatureToRGB(colorTemperatures[currentTemperatureIndex]);
  // Set all LEDs to this color
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// Convert color temperature in Kelvin to RGB
uint32_t colorTemperatureToRGB(int kelvin) {
  float temp = kelvin / 100.0;
  float red, green, blue;

  // Calculate red
  if (temp <= 66) {
    red = 255;
  } else {
    red = temp - 60;
    red = 329.698727446 * pow(red, -0.1332047592);
    if (red < 0) red = 0;
    if (red > 255) red = 255;
  }

  // Calculate green
  if (temp <= 66) {
    green = temp;
    green = 99.4708025861 * log(green) - 161.1195681661;
    if (green < 0) green = 0;
    if (green > 255) green = 255;
  } else {
    green = temp - 60;
    green = 288.1221695283 * pow(green, -0.0755148492);
    if (green < 0) green = 0;
    if (green > 255) green = 255;
  }

  // Calculate blue
  if (temp >= 66) {
    blue = 255;
  } else {
    if (temp <= 19) {
      blue = 0;
    } else {
      blue = temp - 10;
      blue = 138.5177312231 * log(blue) - 305.0447927307;
      if (blue < 0) blue = 0;
      if (blue > 255) blue = 255;
    }
  }

  return strip.Color((uint8_t)red, (uint8_t)green, (uint8_t)blue);
}


void rotaryOnButtonClick() {
  static unsigned long lastTimePressed = 0;

  if (millis() - lastTimePressed < ROTARY_ENCODER_NOISE) {
    return;
  }

  lastTimePressed = millis();
  encoderMode = encoderMode < 3 ? encoderMode + 1 : 0;

  switch (encoderMode) {
      // Brightness
      case (0):
        rotaryEncoder.setEncoderValue(currentBrightnessIndex);
        break;
      // Temp
      case (1):
        rotaryEncoder.setEncoderValue(currentTemperatureIndex);
        break;
      // Equalizer
      case (2):
        rotaryEncoder.setEncoderValue(currentEqualizerIndex);
        break;
      // Sensitivity
      case (3):
        rotaryEncoder.setEncoderValue(currentSensitivityIndex);
        break;
    }

    buzzTimes(encoderMode + 1);
}

void rotaryLoop() {
  if (rotaryEncoder.isEncoderButtonClicked()) {
    rotaryOnButtonClick();
  } else if (rotaryEncoder.encoderChanged()) {
    int encoderValue = rotaryEncoder.readEncoder();

    switch (encoderMode) {
      // Change brightness
      case (0):
        currentBrightnessIndex = encoderValue;
        brightness = 10 * encoderValue;
        strip.setBrightness(brightness);
        break;
      // Change temp
      case (1):
        currentTemperatureIndex = encoderValue > 10 ? 10 : encoderValue;
        break;
      // Change equalizer
      case (2):
        currentEqualizerIndex = encoderValue > 3 ? 3 : encoderValue;
        break;
      // Sensitivity
      case (3):
        currentSensitivityIndex = encoderValue;
        currentSensitivityIndex = currentSensitivityIndex > 10 ? 10 : currentSensitivityIndex;
        currentSensitivityIndex = currentSensitivityIndex == 0 ? 1 : currentSensitivityIndex;
        break;
    }

    buzzTimes(1, 500);
  }
}

void buildEqualizer(){
  for (int i = 0; i < MAX_BANDS; i++) {
    bandValues[i] = 0;
  }

  // Sample the audio pin
  for (int i = 0; i < FFT_SAMPLES; i++) {
    newTime = micros();
    vReal[i] = analogRead(AUDIO_IN_PIN);
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) {
        // wait
    }
  }

  // Fast Fourier Transformation
  FFT.dcRemoval();
  FFT.windowing(vReal, FFT_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, FFT_SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, FFT_SAMPLES);

  for (int i = 2; i < (FFT_SAMPLES / 2); i++) {  // Don't use sample 0 and only first FFT_SAMPLES/2 are usable.
    if (vReal[i] > FFT_FILTERED_NOISE) {         // Add a crude noise filter
      if (i<=5 )           bandValues[0]  += (int)vReal[i];
      if (i>5   && i<=6  ) bandValues[1]  += (int)vReal[i];
      if (i>6   && i<=9  ) bandValues[2]  += (int)vReal[i];
      if (i>9   && i<=11  ) bandValues[3]  += (int)vReal[i];
      if (i>11   && i<=15  ) bandValues[4]  += (int)vReal[i];
      if (i>15   && i<=19  ) bandValues[5]  += (int)vReal[i];
      if (i>19   && i<=25  ) bandValues[6]  += (int)vReal[i];
      if (i>25   && i<=33  ) bandValues[7]  += (int)vReal[i];
      if (i>33   && i<=44  ) bandValues[8]  += (int)vReal[i];
      if (i>44   && i<=58  ) bandValues[9]  += (int)vReal[i];
      if (i>58   && i<=76  ) bandValues[10]  += (int)vReal[i];
      if (i>76   && i<=99  ) bandValues[11]  += (int)vReal[i];
      if (i>99   && i<=131  ) bandValues[12]  += (int)vReal[i];
      if (i>131   && i<=172  ) bandValues[13]  += (int)vReal[i];
      if (i>172   && i<=225  ) bandValues[14]  += (int)vReal[i];
      if (i>225             ) bandValues[15]  += (int)vReal[i];
    }
  }

  // Process the FFT data into bar power
  for (int band = 0; band < MAX_BANDS; band++) {
    // Scale the bars for the display
    if (bandValues[band] > MAX_BAND_VALUE) bandValues[band] = MAX_BAND_VALUE;

    // Small amount of averaging between frames
    bandValues[band] = ((oldBandValues[band] * 1) + bandValues[band]) / 2;

    // Move peak up
    if (bandValues[band] > peak[band]) {
      peak[band] = min(MAX_BAND_VALUE, bandValues[band]);
    }

    // Draw bars
    switch (currentEqualizerIndex) {
      case 0:
        // nope its light mode
        break;
      case 1:
        rainbowEqualizer();
        break;
      case 2:
        spectrumEqualizer();
        break;
      case 3:
        centerEqualizer();
        break;
      case 4:
        pulseEqualizer();
        break;
      case 5:
        waterfallEqualizer();
        break;
    }

    // Save oldBandValues for averaging later
    oldBandValues[band] = bandValues[band];
  }
}


void rainbowEqualizer() {
  // TODO just example need to redo
  int ledBands = 4;

  for (int i = 0; i < NUM_LEDS; i++) {
    int cumulativeValue = 0;

    if (i < 10) {
      cumulativeValue = bandValues[0] + bandValues[1] + bandValues[2] + bandValues[3];
    } else if (i < 20) {
      cumulativeValue = bandValues[4] + bandValues[5] + bandValues[6] + bandValues[7];
    } else if (i < 30) {
      cumulativeValue = bandValues[8] + bandValues[9] + bandValues[10] + bandValues[11];
    } else {
      cumulativeValue = bandValues[12] + bandValues[13] + bandValues[14] + bandValues[15];
    }

    cumulativeValue = cumulativeValue * currentSensitivityIndex;
    if (cumulativeValue > MAX_BAND_VALUE * ledBands) cumulativeValue = MAX_BAND_VALUE * ledBands;

    int intensity = map(cumulativeValue, 0, MAX_BAND_VALUE * ledBands, 0, 255);

    strip.setPixelColor(i, strip.Color(intensity, random(0, intensity), 255 - intensity));
  }

  strip.show();
}

void spectrumEqualizer() {
}

void centerEqualizer() {
}

void pulseEqualizer() {
}

void waterfallEqualizer() {
}
