#include "arduinoFFT.h"

#define Rpin 11
#define Gpin 10
#define Bpin 9
#define delayLEDS 500
#define sensorPin A5

const uint16_t samples = 128;  
float sensorValue = 0;
float lowPassFilteredSignal = 0;
float highPassFilteredSignal = 0;
double vReal[samples];             
double vImag[samples];             
float samplingFrequency = 1000.0;  

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

unsigned long analogTimer = 0;
unsigned long analogPeriod = 2;  
int samplingRate = 10;

float previousFFTValue = 0;
float threshold = 1.0;  

unsigned long currentTime = 0;          
unsigned long previousTime = 0;         
const float desiredInterval = 0.5;      

void setup() {
  Serial.begin(19200);
  pinMode(Rpin, OUTPUT);
  pinMode(Gpin, OUTPUT);
  pinMode(Bpin, OUTPUT);
  pinMode(sensorPin, INPUT);
}

void loop() {
  currentTime = millis();               

  if (currentTime - previousTime >= desiredInterval * 1000) {  
    MainFunction();                     
    previousTime = currentTime;
  }
}

void MainFunction() {
  // This function is the main loop that reads the sensor, applies filters and FFT, and changes the LED color
  if (millis() - analogTimer > analogPeriod) {
    analogTimer = millis();

    for (int i = 0; i < samplingRate; i++) {
      sensorValue += (float)analogRead(sensorPin) * (5 / 1024.0);
    }
    sensorValue = sensorValue / samplingRate;

    Serial.print("Sensor Value: ");
    Serial.print(sensorValue, 4);
    Serial.print("\t");

    LowPassFilter(sensorValue); // Apply low-pass filter on the sensor value
    HighPassFilter(sensorValue); // Apply high-pass filter on the sensor value

    float lowPassFFTValue = ApplyFFT(lowPassFilteredSignal); // Apply FFT on low-pass filtered signal
    float highPassFFTValue = ApplyFFT(highPassFilteredSignal); // Apply FFT on high-pass filtered signal

    float selectedFFTValue = max(lowPassFFTValue, highPassFFTValue); // Select the higher value from both FFT results

    Serial.print("Low Pass Filtered Signal: ");
    Serial.print(lowPassFilteredSignal, 4);
    Serial.print("\t");

    Serial.print("High Pass Filtered Signal: ");
    Serial.print(highPassFilteredSignal, 4);
    Serial.print("\t");

    Serial.print("Selected FFT Value: ");
    Serial.print(selectedFFTValue, 4);
    Serial.print("\t");

    ChangeLEDColor(selectedFFTValue); // Change LED color based on the selected FFT value
    Serial.println();
  }
}

void LowPassFilter(float sensorSignal) {
  // This function applies a low-pass filter to the sensor signal
  lowPassFilteredSignal = (0.945 * lowPassFilteredSignal) + (0.0549 * sensorSignal);
}

void HighPassFilter(float sensorSignal) {
  // This function applies a high-pass filter to the sensor signal
  static float previousSensorSignal = 0;
  const float alpha = 0.5;

  highPassFilteredSignal = alpha * (highPassFilteredSignal + sensorSignal - previousSensorSignal);
  previousSensorSignal = sensorSignal;
}

float ApplyFFT(float filteredSignal) {
  // This function applies FFT on the given filtered signal and returns the highest magnitude value
  for (int i = 0; i < samples; i++) {
    vReal[i] = filteredSignal;  
    vImag[i] = 0;                      
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Find the frequency bin with the highest magnitude
  float maxMagnitude = 0;
  for (int i = 0; i < samples / 2; i++) {
    if (vReal[i] > maxMagnitude) {
      maxMagnitude = vReal[i];
    }
  }
  return maxMagnitude;
}

void ChangeLEDColor(float fftValue) {
  // This function changes the LED color based on the selected FFT value
  if (abs(fftValue - previousFFTValue) >= 0.05) {
    int colorIndex = (int)(fftValue) % 8; 
    switch (colorIndex) {
      case 0: RGBColor(255, 0, 0); Serial.print(" Color: Red"); break;
      case 1: RGBColor(255, 127, 0); Serial.print(" Color: Orange"); break;
      case 2: RGBColor(0, 0, 255); Serial.print(" Color: Blue"); break;
      case 3: RGBColor(255, 255, 0); Serial.print(" Color: Yellow"); break;
      case 4: RGBColor(0, 255, 0); Serial.print(" Color: Green"); break;
      case 5: RGBColor(0, 255, 127); Serial.print(" Color: Aqua"); break;
      case 6: RGBColor(0, 255, 255); Serial.print(" Color: Cyan"); break;
      case 7: RGBColor(75, 0, 130); Serial.print(" Color: Purple"); break;
    }
    previousFFTValue = fftValue; 
  }
}

void RGBColor(int Rcolor, int Gcolor, int Bcolor) {
  // This function sets the RGB color of the LED
  analogWrite(Rpin, Rcolor);
  analogWrite(Gpin, Gcolor);
  analogWrite(Bpin, Bcolor);
  delay(delayLEDS);  
}
