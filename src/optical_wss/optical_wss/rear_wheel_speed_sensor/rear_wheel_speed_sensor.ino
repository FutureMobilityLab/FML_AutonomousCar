// Code for an arduino nano to read from the rear wheel speed sensors. The sensors are IR optical
// interruptors. The serial connection is intended to be read by a raspberry pi using the UART
// protocol. For more information see: https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/#Arduino_code
#define RL_WSS_PIN 12
#define RR_WSS_PIN 13

int RearLeftState; // 1 for HIGH, 0 for LOW
int RearRightState; // 1 for HIGH, 0 for LOW
unsigned long startTime;
// Assume there are 72 slots in the disk and each tick is either a rise or fall of the interrupt.
const float RADPERTICK = 2*PI / 144;
const unsigned long SAMPLETIME = 50000; // [us]

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  RearLeftState = 0;
  RearRightState = 0;
  startTime = micros();
  pinMode(RL_WSS_PIN, INPUT);
  pinMode(RR_WSS_PIN, INPUT);
  Serial.println("Setup Complete");
}

// To make this loop run fast call Serial.println() as little as possible.
// analogRead runs about 16 times slower than digitalRead. See if we can use 
// digitalRead on the interruptors.

void loop() {
  unsigned long now = micros();
  int RLWheelStateChanges = 0;
  int RRWheelStateChanges = 0;

  // Read IR interruptors until sample time.
  while (now - startTime < SAMPLETIME) {
    // Get state of IR interruptors.
    int RearLeftNewState = digitalRead(RL_WSS_PIN);
    int RearRightNewState = digitalRead(RR_WSS_PIN);

    // If the rear left has changed state.
    if (RearLeftState != RearLeftNewState) {
      RLWheelStateChanges += 1;
      RearLeftState = RearLeftNewState;
    }

    // If the rear right has changed state.
    if (RearRightState != RearRightNewState) {
      RRWheelStateChanges += 1;
      RearRightState = RearRightNewState;
    }
    now = micros();
  }
  float RLWheelSpeed = (float)RLWheelStateChanges * RADPERTICK / SAMPLETIME * 1E6;
  float RRWheelSpeed = (float)RRWheelStateChanges * RADPERTICK / SAMPLETIME * 1E6;

  // Communicate right and left wheel speed at the same time. 
  // Note: some of the wheel speeds are at most MAXTIME ms old.
  Serial.print("RL,");
  Serial.print(RLWheelSpeed, 4);
  Serial.print(",");
  Serial.print("RR,");
  Serial.println(RRWheelSpeed, 4);

  // The sample rate should not be influenced by the delay in Serial.print.
  startTime = now;
}
