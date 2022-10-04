// Initialize RF22
#include <RF22.h>
#include <RF22Router.h>
#define MY_ADDRESS 3
#define DESTINATION_ADDRESS 1
RF22Router rf22(MY_ADDRESS);
long randNumber;
boolean successful_packet = false;
int max_delay = 3000;

// Initialize Accelerometer (ADXL335)
#include "arduinoFFT.h"
#define SAMPLES 128                  //Must be a power of 2
#define SAMPLING_FREQUENCY 20        //Hz
#define ACCEL_SAMPLES 10
#define XPIN  A2
#define YPIN  A1
#define ZPIN  A0
float xavg, yavg, zavg;             // x,y,z value at rest
int steps = 0, accelValIndex = 0;
double accelValuesR[SAMPLES];
double accelValuesIm[SAMPLES];
unsigned long prevMillisAcc = 0, accelStart = 0;
arduinoFFT FFT = arduinoFFT();      // Create FFT object

// Define a flag to control when to send a packet
bool packetReady = 0;

void setup() {
  Serial.begin(9600);
  if (!rf22.init())
    Serial.println("RF22 init failed");
  if (!rf22.setFrequency(431.0))
    Serial.println("setFrequency failed");
  rf22.setTxPower(RF22_TXPOW_20DBM);
  rf22.setModemConfig(RF22::GFSK_Rb125Fd125);

  // Manually define the routes for this network
  rf22.addRouteTo(DESTINATION_ADDRESS, DESTINATION_ADDRESS);
  randomSeed(analogRead(A0));
  // Start up Pedometer
  calibrate();
}

void loop() {
  // Update the number of steps
  getSteps();

  if(packetReady) {
    char data_read[RF22_ROUTER_MAX_MESSAGE_LEN];
    uint8_t data_send[RF22_ROUTER_MAX_MESSAGE_LEN];
    memset(data_read, '\0', RF22_ROUTER_MAX_MESSAGE_LEN);
    memset(data_send, '\0', RF22_ROUTER_MAX_MESSAGE_LEN);
    sprintf(data_read, "%d", steps);
    data_read[RF22_ROUTER_MAX_MESSAGE_LEN - 1] = '\0';
    memcpy(data_send, data_read, RF22_ROUTER_MAX_MESSAGE_LEN);
  
    successful_packet = false;
    while (!successful_packet) {
      if (rf22.sendtoWait(data_send, sizeof(data_send), DESTINATION_ADDRESS) != RF22_ROUTER_ERROR_NONE) {
        Serial.println("Send request failed");
        randNumber = random(200,max_delay);
        Serial.println(randNumber);
        delay(randNumber);
      }
      else {
        successful_packet = true;
        Serial.println("Send request succeeded");
      }
    }
  }  
}

/**
 * The getSteps() function calculates the number of steps that the user has taken using 
 * the Fourier Transform.
 */
void getSteps() {
  unsigned long currentMillisAcc = millis();
  // If the number of samples taken is less than 128 and the time interval between
  // the previous and the current measurement is higher or equal to the sampling
  // period, read a new acceleration value.
  if ((accelValIndex < SAMPLES) 
      && (currentMillisAcc - prevMillisAcc >= 1000/SAMPLING_FREQUENCY)) {
    prevMillisAcc = currentMillisAcc;
    // If you take the 1st sample, save the timer value.
    if (accelValIndex == 0) {
      accelStart = currentMillisAcc;
      packetReady = 0;
    }
    // Read acceleration value
    getAccelValue();
    // Increase by one the number of samples taken
    accelValIndex++;
  }
  // If you have 128 samples, do FFT
  if (accelValIndex >= SAMPLES) {
    currentMillisAcc = millis();
    FFT.DCRemoval(accelValuesR, SAMPLES);
    // Apply a Hamming window
    FFT.Windowing(accelValuesR, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    // Compute FFT
    FFT.Compute(accelValuesR, accelValuesIm, SAMPLES, FFT_FORWARD);
    // Compute magnitude
    FFT.ComplexToMagnitude(accelValuesR, accelValuesIm, SAMPLES);
    double f0 = FFT.MajorPeak(accelValuesR, SAMPLES, SAMPLING_FREQUENCY);
    // Filter
    if (f0 > 2.6) {
      f0 = 0;
    }
    steps += int(f0 * (currentMillisAcc - accelStart)/1000);
    accelValIndex = 0;
    // Set the flag to true -> send the number of steps to the receiver
    packetReady = 1;
  }
}

/**
 * The getAccelValue() function reads the current x,y,z values from the ADXL335 accelerometer.
 */
void getAccelValue() {
  // Find the current x,y,z values and calculate the magnitude of the acceleration vector
  float acc = 0;
  // Find the average acceleration using a predefined number of samples
  for (int i = 0; i < ACCEL_SAMPLES; i++) {
    float x = analogRead(XPIN);
    float y = analogRead(YPIN);
    float z = analogRead(ZPIN);
    acc += sqrt((x - xavg)*(x - xavg) + (y - yavg)*(y - yavg) + (z - zavg)*(z - zavg));
  }
  acc = acc / float(ACCEL_SAMPLES);
  accelValuesR[accelValIndex] = acc;
  accelValuesIm[accelValIndex] = 0;
}

/**
 * The calibrate() function reads the x,y,z values from the ADXL335 accelerometer at rest and
 * updates the xavg,yavg,zavg global values.
 */
void calibrate() {
  float xsum = 0;
  float ysum = 0;
  float zsum = 0;
  for (int i = 0; i < ACCEL_SAMPLES; i++) {
    xsum += analogRead(XPIN);
    ysum += analogRead(YPIN);
    zsum += analogRead(ZPIN);
  }
  xavg = xsum / float(ACCEL_SAMPLES);
  yavg = ysum / float(ACCEL_SAMPLES);
  zavg = zsum / float(ACCEL_SAMPLES);
}
