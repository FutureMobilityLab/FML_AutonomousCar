// Code for an arduino nano to read from the rear wheel speed sensors. The sensors are IR optical
// interruptors. The serial connection is intended to be read by a raspberry pi using the UART
// protocol. For more information see: https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/#Arduino_code
#define RL_WSS_PIN A0
#define RR_WSS_PIN A1

bool RearLeftState;
bool RearRightState;
unsigned long RearLeftStartTime;
unsigned long RearRightStartTime;
float RLWheelSpeed;
float RRWheelSpeed;
// Assume there are 20 slots in the disk and each tick is either a rise or fall of the interrupt.
const float RADPERTICK = 2*PI / 40;
const float TRACK = 0.28; // track width [m]
const int MAXTIME = 100; // maximum time to wait till assuming zero velocity [ms]

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  RearLeftStartTime = millis();
  RearRightStartTime = millis();
  RearLeftState = false;
  RearRightState = false;
  RLWheelSpeed = 0.;
  delay(1000);
  Serial.println("Setup Complete");
}

void loop() {
  // Read from IR interruptors.
  int RearLeft = analogRead(RL_WSS_PIN);
  bool RearLeftInterrupted = RearLeft < 255;
  int RearRight = analogRead(RR_WSS_PIN);
  bool RearRightInterrupted = RearRight < 255;
  unsigned long CurrentTime = millis();

  // If the rear left has changed state.
  if (RearLeftState != RearLeftInterrupted) {
    unsigned long ElapsedTime = CurrentTime - RearLeftStartTime;
    RearLeftStartTime = CurrentTime;
    // Compute wheel speed as rad/s.
    RLWheelSpeed = RADPERTICK / ElapsedTime * 1E3;
    Serial.print("RL,");
    Serial.println(RLWheelSpeed);
    RearLeftState = RearLeftInterrupted;
  } else if (CurrentTime - RearLeftStartTime > MAXTIME) {
    RearLeftStartTime = CurrentTime;
    RLWheelSpeed = 0.0;
  }

  // If the rear right has changed state.
  if (RearRightState != RearRightInterrupted) {
    unsigned long ElapsedTime = CurrentTime - RearRightStartTime;
    RearRightStartTime = CurrentTime;
    // Compute wheel speed as rad/s.
    RRWheelSpeed = RADPERTICK / ElapsedTime * 1E3;
    RearRightState = RearRightInterrupted;
  } else if (CurrentTime - RearRightStartTime > MAXTIME) {
    RearRightStartTime = CurrentTime;
    RRWheelSpeed = 0.0;
  }

  // Communicate right and left wheel speed at the same time. 
  // Note: some of the wheel speeds are at most MAXTIME ms old.
  Serial.print("RL,");
  Serial.print(RLWheelSpeed);
  Serial.print(",");
  Serial.print("RR,");
  Serial.println(RRWheelSpeed);
}
