bool RearLeftState;
bool RearRightState;
unsigned long RearLeftStartTime;
unsigned long RearRightStartTime;
// Assume there are 20 slots in the disk and each tick is either a rise or fall of the interrupt.
const float RADPERTICK = 2*PI / 40;
const float TRACK = 0.28; // track width [m]

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  RearLeftStartTime = millis();
  RearRightStartTime = millis();
  RearLeftState = false;
  RearRightState = false;
  Serial.println("Setup Complete");
}

void loop() {
  // Read from IR interruptors.
  int RearLeft = analogRead(A0);
  bool RearLeftInterrupted = RearLeft < 255;
  int RearRight = analogRead(A1);
  bool RearRightInterrupted = RearRight < 255;

  // If the rear left has changed state.
  if (RearLeftState != RearLeftInterrupted) {
    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - RearLeftStartTime;
    RearLeftStartTime = CurrentTime;
    // Compute wheel speed as rad/s.
    float RLWheelSpeed = RADPERTICK / ElapsedTime * 1E3;
    Serial.print("RL,");
    Serial.println(RLWheelSpeed);
    RearLeftState = RearLeftInterrupted;
  }

  // If the rear right has changed state.
  if (RearRightState != RearRightInterrupted) {
    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - RearRightStartTime;
    RearRightStartTime = CurrentTime;
    // Compute wheel speed as rad/s.
    float RRWheelSpeed = RADPERTICK / ElapsedTime * 1E3;
    Serial.print("RR,");
    Serial.println(RRWheelSpeed);
    RearRightState = RearRightInterrupted;
  }
}
