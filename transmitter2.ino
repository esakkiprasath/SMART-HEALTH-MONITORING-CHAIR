// Initialize RF22
#include <RF22.h>
#include <RF22Router.h>
#define MY_ADDRESS 2
#define DESTINATION_ADDRESS 1
RF22Router rf22(MY_ADDRESS);
long randNumber;
boolean successful_packet = false;
int max_delay = 3000;

// Initialize DHT11
#include "DHT.h"
#define DHTTYPE DHT11
#define DHT_DELAY 10000       // waiting time between two temperature requests
#define DHTPIN 4
unsigned long prevMillisDHT = 0;
float humidity = 0;           // stores the last valid humidity value
float temperature = 0;        // stores the last valid temperature value
DHT dht(DHTPIN, DHTTYPE);

// Initialize DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#define TMP_DELAY 10000         // waiting time between two temperature requests
// Data wire is conntec to the Arduino digital pin 2
#define ONE_WIRE_BUS 5
// Setup an oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
unsigned long prevMillisTmp = 0;
float bodyTemperature = 0;      // stores the latest body temperature value

// Initialize heart rate sensor (AD8232)
// Set the time between 2 HR calculations (30sec)
#define ECG_TIMER 30000
// Pins for leads off detection
#define LO_PLUS 8
#define LO_MINUS 9
// Shutdown pin
#define SHUTDOWN_PIN 10
int valueECG, heartBeats = 0, heartRate;
int thresholdECG = 400; // Change that
bool isECGActive = 1, heartBeatDetected = 0;
unsigned long prevHRmeasurement = 0;

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

  // Start up DHT11
  dht.begin();
  // Start up DS18B20
  sensors.begin();
  // Heart rate
  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);
}

void loop() {
  // Get temperature and humidity
  getTemperatureAndHumidity();
  // Get body temperature
  getBodyTemperature();
  // Get heart rate
  getHeartRate();

  if(packetReady) {
    char data_read[RF22_ROUTER_MAX_MESSAGE_LEN];
    uint8_t data_send[RF22_ROUTER_MAX_MESSAGE_LEN];
    memset(data_read, '\0', RF22_ROUTER_MAX_MESSAGE_LEN);
    memset(data_send, '\0', RF22_ROUTER_MAX_MESSAGE_LEN);
    sprintf(data_read, "%d %d %d %d", int(temperature*100), int(humidity*100), int(bodyTemperature*100), heartRate);
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
 * The getTemperatureAndHumidity() function reads the current temperature and humidity values
 * from the DHT11 sensor.
 */
void getTemperatureAndHumidity() {
  unsigned long currentMillisDHT = millis();
  if(currentMillisDHT - prevMillisDHT >= DHT_DELAY) {
    float h = dht.readHumidity();
    // Read temperature as Celsius
    float t = dht.readTemperature();
    // Check if any reads failed
    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    else {
      prevMillisDHT = currentMillisDHT;
      humidity = h;
      temperature = t;
    }
  }
}

/**
 * The getBodyTemperature() function reads the current temperature value from the 
 * DS18B20 waterproof sensor.
 */
void getBodyTemperature() {
  unsigned long currentMillisTmp = millis();
  if(currentMillisTmp - prevMillisTmp >= TMP_DELAY) {
    prevMillisTmp = currentMillisTmp;
    // Send a command to get temperature
    sensors.requestTemperaturesByIndex(0);
    bodyTemperature = sensors.getTempCByIndex(0);
  }
}

/**
 * The getHeartRate() function reads the current heart rate value (bpm) from the AD8232 
 * ECG sensor.
 */
void getHeartRate() {
  isECGActive = 1;
  packetReady = 0;
  // Check whether the ECG device is not active
  if ((digitalRead(LO_PLUS) == 1)||(digitalRead(LO_MINUS) == 1)) {
    digitalWrite(SHUTDOWN_PIN, LOW);
    Serial.println("!!");
    isECGActive = 0;
  }
  
  // If the ECG device is active, take measurements
  if (isECGActive) {
    digitalWrite(SHUTDOWN_PIN, HIGH); //normal mode
    // Read current ECG value
    valueECG = analogRead(A0);
    
    if ((valueECG > thresholdECG) && (!heartBeatDetected)) {
      heartBeats++;  
      heartBeatDetected = 1;
    }

    else if ((valueECG < thresholdECG)) {
      heartBeatDetected = 0;
    }

    unsigned long currentHRmeasurement = millis();
    if ((currentHRmeasurement -  prevHRmeasurement) >= ECG_TIMER) {
      heartRate = heartBeats * 2;
      prevHRmeasurement = currentHRmeasurement;
      heartBeats = 0;
      // Packet is ready -> send data to the receiver
      packetReady = 1;
    }
  }
}
