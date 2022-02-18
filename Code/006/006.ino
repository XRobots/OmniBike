int wheel1;
int wheel1a;
int wheel2;
int wheel2a;

//PID
#include <PID_v1.h>

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

float RLR = 0;
float RFB = 0;
float RT = 0;
float LLR = 0;
float LFB = 0;
float LT = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer

float var1;       // data received from IMU
float var2;
float IMUroll;
float IMUpitch;
int check;

float pot;
int sw1;
float steering;

long encoder0Pos;           // encoder
float encoder0PosF;
#define encoder0PinA 2
#define encoder0PinB 3

volatile boolean PastB = 0;
volatile boolean update = false;

double Pk1 = 0.1; 
double Ik1 = 0;
double Dk1 = 0;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - motor position
double Pk2 = 13;    
double Ik2 = 125;   
double Dk2 = 2.5;   

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup - balancing


void setup() {

    pinMode(5, OUTPUT);   // motors
    pinMode(6, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);

    pinMode(22, INPUT_PULLUP);    // motor enable switch
    pinMode(A0, INPUT);           // trim pot
   
    // initialize serial communication
    Serial.begin(115200);
    Serial3.begin(115200);
    
    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
    pinMode(encoder0PinB, INPUT_PULLUP);

    attachInterrupt(1, doEncoderB, RISING);

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-200, 200);
    PID1.SetSampleTime(10);

    PID2.SetMode(AUTOMATIC);              
    PID2.SetOutputLimits(-400, 400);
    PID2.SetSampleTime(10);

    
}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {  

        if (update) {
              update = false;
              PastB ? encoder0Pos++ : encoder0Pos--;
        }
      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
            previousMillis = currentMillis;



            // check for radio data
            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));   
            } 

            else {
              Serial.println("no data");
            }

            if (Serial3.available() > 1){
                    check = Serial3.parseInt();
                    if (check == 500) {                   // look for check character to check it's the start of the data
                        var1 = Serial3.parseInt();
                        var2 = Serial3.parseInt();
    
                        if (Serial3.read() == '\n') {     // end of IMU data 
                            IMUpitch = var2 / 100;         // divide by 100 to get our decimal places back
                            IMUroll = var1 / 100;
                        }
                    }
            }

            if (update) {
              update = false;
              PastB ? encoder0Pos-- : encoder0Pos++;
            }

            

            sw1 = digitalRead(22);    // motor enable switch
            pot = analogRead(A0);
            pot = (pot-512)/100;

            // threshold remote data
            // some are reversed based on stick wiring in remote
            RFB = thresholdStick(mydata_remote.RFB)/2;
            RLR = thresholdStick(mydata_remote.RLR);
            LT = thresholdStick(mydata_remote.LT);
                             
         
            // wheel 1 - main/manual drive wheel forwards/backwards
            wheel1 = RFB;

            if (wheel1 > 0) {
              wheel1 = constrain(wheel1,0,255);
              analogWrite(6, wheel1);
              analogWrite(5, 0);
            }
            else if (wheel1 < 0) {
              wheel1a = abs(wheel1);
              wheel1a = constrain(wheel1a,0,255);
              analogWrite(5, wheel1a);
              analogWrite(6, 0);
            }
            else {
              analogWrite(5, 0);
              analogWrite(6, 0);
            }

            // zero everything when the motor enable switch is off so values don't accumulate.
            if (sw1 == 1) {
              Setpoint1 = 0;
              Setpoint2 = 0;
              encoder0Pos = 0;              
            }

            // wheel2 - balancing PID
            Setpoint2 = pot + ((LT/300));
            //Serial.println(Setpoint2);
            Input2 = IMUroll;
            PID2.Compute(); 

            // wheel 2 - positioning with encoder PID
            Setpoint1 = Setpoint1 + Output2;      // use Output of balancing PID to drive position over time
            Input1 = encoder0Pos;
            PID1.Compute();           

            // change position to velocity - by changing position on every cycle

            wheel2 = Output1;       // use potitioning PID to drive wheel

            if (wheel2 > 0 && sw1 == 0) {
              wheel2 = constrain(wheel2,0,255);
              analogWrite(10, wheel2);
              analogWrite(11, 0);
            }
            else if (wheel2 < 0 && sw1 == 0) {
              wheel2a = abs(wheel2);
              wheel2a = constrain(wheel2a,0,255);
              analogWrite(11, wheel2a);
              analogWrite(10, 0);
            }
            else {
              analogWrite(10, 0);
              analogWrite(11, 0);
            } 


      
        }     // end of timed loop         
   
}       // end  of main loop



/// read encoders

void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void doEncoderB()
{
  PastB = (boolean)digitalRead(encoder0PinA);
  update = true;
}

