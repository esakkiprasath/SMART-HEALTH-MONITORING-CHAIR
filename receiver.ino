#include <RF22.h>
#include <RF22Router.h>

#define MY_ADDRESS 1
#define NODE_ADDRESS_1 2
#define NODE_ADDRESS_2 3
RF22Router rf22(MY_ADDRESS);

// Create variables to save the latest received values
float temperature;
float humidity;
float bodyTemperature;
int steps;
int heartRate;


void setup() {
  Serial.begin(9600);
  if (!rf22.init())
    Serial.println("RF22 init failed");
  if (!rf22.setFrequency(431.0))
    Serial.println("setFrequency failed");
  rf22.setTxPower(RF22_TXPOW_20DBM);
  rf22.setModemConfig(RF22::GFSK_Rb125Fd125);
  // Manually define the routes for this network
  rf22.addRouteTo(NODE_ADDRESS_1, NODE_ADDRESS_1);
  rf22.addRouteTo(NODE_ADDRESS_2, NODE_ADDRESS_2);
}

void loop() {
  uint8_t buf[RF22_ROUTER_MAX_MESSAGE_LEN];
  char incoming[RF22_ROUTER_MAX_MESSAGE_LEN];
  memset(buf, '\0', RF22_ROUTER_MAX_MESSAGE_LEN);
  memset(incoming, '\0', RF22_ROUTER_MAX_MESSAGE_LEN);
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (rf22.recvfromAck(buf, &len, &from)) {
    buf[RF22_ROUTER_MAX_MESSAGE_LEN - 1] = '\0';
    memcpy(incoming, buf, RF22_ROUTER_MAX_MESSAGE_LEN);
    Serial.print("Got information from node #");
    Serial.println(from - 1, DEC);
    // Read the received values
    readValues(incoming, from);
  }
}

/**
 * The readValues() function reads the prints the received values.
 */
void readValues(char* packet, uint8_t id) {
  if(id == 2) {
    char* val;
    val = strtok(packet, " ");
    for(int i = 0; i < 4; i++) {
      if(i == 0) temperature = atoi(val)/100.0;
      if(i == 1) humidity = atoi(val)/100.0;
      if(i == 2) bodyTemperature = atoi(val)/100.0;
      if(i == 3) heartRate = atoi(val);
      val = strtok(NULL, " ");
    }
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.print(" °C");
    Serial.print(" | Humidity = ");
    Serial.print(humidity);
    Serial.println("%");
    Serial.print("Body Temperature = ");
    Serial.print(bodyTemperature);
    Serial.println(" °C");
    Serial.print("Heart rate = ");
    Serial.print(heartRate);
    Serial.println(" bpm");
  }
  else {
    steps = atoi(packet);
    Serial.print("Number of steps = ");
    Serial.println(steps);
  }
}
