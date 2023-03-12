#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <Servo.h>

uint8_t SERVO_STEER_PIN = 5; // steering servo
uint8_t SERVO_LEFT_PIN = 6; // feed left servo
uint8_t SERVO_RIGHT_PIN = 3; // feed right servo
uint8_t MOTOR_EN1 = 9;
uint8_t MOTOR_IN1 = 2;
uint8_t MOTOR_IN2 = 4;

uint16_t SENS_RATE = 5;
uint16_t SEND_RATE = 100;
uint16_t SERI_RATE = 500;
void stop_motor(void);


Servo steering,feedL,feedR; // servo lib
RF24 radio(7, 8);  // using pin 7 for the CE pin, and pin 8 for the CSN pin

uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = false;  // true = TX role, false = RX role
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

uint8_t dummy0;
uint8_t Maccel;
uint8_t Steer;
uint8_t Lfeeder,Rfeeder;
uint8_t Enable;

bool empty,failed,ackreceived,datareceived;
unsigned long start_timer,end_timer;
uint8_t payloadsize;
uint8_t pipe,bytes;


void setup()
{
  pinMode(MOTOR_EN1,OUTPUT);
  pinMode(MOTOR_IN1,OUTPUT);
  pinMode(MOTOR_IN2,OUTPUT);
  steering.attach(SERVO_STEER_PIN); //steering pin
  feedL.attach(SERVO_LEFT_PIN); //feed pin
  feedR.attach(SERVO_RIGHT_PIN); //feed pin

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radioNumber = 1;
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

    if (radio.available(&pipe)) {                     // is there a payload? get the pipe number that recieved it
      bytes = radio.getDynamicPayloadSize();  // get the size of the payload
      radio.read(&received, sizeof(received)); // get incoming payload
      // save incoming counter & increment for next outgoing
      payload.counter = received.counter + 1;
      // load the payload for the first received transmission on pipe 0
      radio.writeAckPayload(1, &payload, sizeof(payload));
    }
      Maccel = received.accel_send;
      Steer = received.steer_send;
      Lfeeder = received.feedl_send;
      Rfeeder = received.feedr_send;
      Enable =  received.enable_send;


  if (Tnow - Tsens > SENS_RATE) {
  Tsens = Tnow;
  }
  if (Enable == 1) {
    payload.enable_send = 1;
   //motor section
    if (Maccel>=129) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_EN1,map(Maccel,129,255,0,255));
    }
    else if (Maccel<=125) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_EN1,map(Maccel,125,0,0,255));
    }
    else  {stop_motor();}

  // steering section
  if (Steer>=0 || Steer<=255) {
  steering.write(map(Steer,0,255,0,180));
  }

  if (Lfeeder == 1) {feedL.write(180); payload.feedl_send = 1;} else {feedL.write(90);payload.feedl_send = 0;}
  if (Rfeeder == 1) {feedR.write(180); payload.feedr_send = 1;} else {feedR.write(90);payload.feedr_send = 0;}
  }
  else {payload.enable_send = 0;}

  //serial section
  if (Tnow - Tlast > 500) {
    Tlast = Tnow;
    }
}

void stop_motor(){
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_EN1,0);
}