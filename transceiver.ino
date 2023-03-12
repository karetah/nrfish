#include <SPI.h>
#include "printf.h"
#include "RF24.h"

uint8_t ACCEL_PIN = A1;
uint8_t STEER_PIN = A0;
uint8_t ENABLE_PIN = 3;
uint8_t LEFT_PIN = 4;
uint8_t RIGHT_PIN = 2;

uint8_t ENABLE_LED = 9;
uint8_t LEFT_LED = 6;
uint8_t RIGHT_LED = 5;

uint16_t SENS_RATE = 500;
uint16_t SEND_RATE = 50;
uint16_t SERI_RATE = 500;

RF24 radio(7, 8);  // using pin 7 for the CE pin, and pin 8 for the CSN pin

uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = true;  // true = TX role, false = RX role
bool report;
struct PayloadStruct {
  uint8_t accel_send;
  uint8_t steer_send;
  uint8_t feedl_send;
  uint8_t feedr_send;
  uint8_t enable_send;
  uint8_t counter;
};
PayloadStruct payload;
PayloadStruct received;

unsigned long Tnow,Tsens,Tsend,Tlast;
String s1=";";
bool Lstate,hLstate,Rstate,hRstate,Estate,hEstate;
bool cLstate = 1;
bool cRstate = 1;
bool cEstate = 1;
uint8_t accel_value,steer_value;

bool empty,failed,ackreceived,datareceived;
unsigned long start_timer,end_timer;
uint8_t payloadsize;
uint8_t pipe,bytes;

void setup()
{
  pinMode(LEFT_PIN,INPUT_PULLUP);
  pinMode(RIGHT_PIN,INPUT_PULLUP);
  pinMode(ENABLE_PIN,INPUT_PULLUP);
  pinMode(ENABLE_LED,OUTPUT);
  pinMode(LEFT_LED,OUTPUT);
  pinMode(RIGHT_LED,OUTPUT);

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radioNumber = 0;
  // radio.setDataRate(RF24_250KBPS); // Long range 

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();  // ACK payloads are dynamically sized

  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  if (role) {
    radio.stopListening();                 // put radio in TX mode
  } else {
    radio.writeAckPayload(1, &payload, sizeof(payload));
    radio.startListening();  // put radio in RX mode
  }

}
void loop() 
{
    Tnow = millis();
    Estate = digitalRead(ENABLE_PIN);
    Lstate = digitalRead(LEFT_PIN); 
    Rstate = digitalRead(RIGHT_PIN);
    accel_value = map(analogRead(ACCEL_PIN),0,1023,0,255);
    steer_value = map(analogRead(STEER_PIN),0,1023,0,255);

    if (Tnow - Tsens > SENS_RATE) {
      //button debounce
      if (cEstate != Estate) {cEstate = Estate;} 
      if (cLstate != Lstate) {cLstate = Lstate;} 
      if (cRstate != Rstate) {cRstate = Rstate;} 
      if (cEstate == 0 && hEstate == 0) { hEstate = 1;} else if (cEstate == 0 && hEstate == 1) { hEstate = 0;}
      if (cLstate == 0 && hLstate == 0) { hLstate = 1;} else if (cLstate == 0 && hLstate == 1) { hLstate = 0;}
      if (cRstate == 0 && hRstate == 0) { hRstate = 1;} else if (cRstate == 0 && hRstate == 1) { hRstate = 0;}
      Tsens = Tnow;
    }
      payload.accel_send = accel_value;
      payload.steer_send = steer_value;
      payload.feedr_send = hRstate;
      payload.feedl_send = hLstate; 
      payload.enable_send = hEstate; 

    if (Tnow - Tsend > SEND_RATE) {
      start_timer = micros();                  // start the timer
      report = radio.write(&payload, sizeof(payload));  // transmit & save the report
      end_timer = micros();                    // end the timer
      if (report) {
        if (radio.available(&pipe)) {  // is there an ACK payload? grab the pipe number that received it
          radio.read(&received, sizeof(received)); 
          payloadsize = radio.getDynamicPayloadSize();
          payload.counter = received.counter + 1;
          digitalWrite(ENABLE_LED,received.enable_send);
          digitalWrite(LEFT_LED,received.feedl_send);
          digitalWrite(RIGHT_LED,received.feedr_send);
          ackreceived = 1;
          empty = 0;
        } else {
          empty = 1;
        }
        failed = 0;
      } else {
        failed = 1;
      }
      
        Tsend = Tnow;
    }

    if (Tnow - Tlast > SERI_RATE) {

      if (report) {
      }
      if (ackreceived) {
      } 
      if (empty) {
        Serial.println(F(" Recieved: an empty ACK packet"));  // empty ACK packet received
      }
      if (failed) {
        Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
      }
      Tlast = Tnow;
    }
}

