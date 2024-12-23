#include <Arduino.h>

#define MOTOR_UP 9      // PWM pin for upward movement
#define MOTOR_DOWN 10   // PWM pin for downward movement
#define DOOR_OPEN 4    // Output pin to open the door
#define DOOR_CLOSE 5   // Output pin to close the door
#define FLOOR_SENSOR A0 // Analog input for floor sensor
#define EMERGENCY_BUTTON 2 // Digital input for emergency stop (active low)
#define FLOOR_BUTTONS_PORT A1  // Port A for external floor requests
#define INSIDE_BUTTONS_PORT A2 // Port C for internal destination selection
#define LED_PORT A3        // Port for floor indicator LEDs
#define EMERGENCY()  emergencyISR()
#define fire() fireISR()
#define power() powerISR()

// System configuration
#define NUM_FLOORS 4
const int floorReadings[NUM_FLOORS] = {0, 100, 200, 300};  // Example readings, calibrate!
int currentFloor = 0;
int requestedFloor = 0;
int destinationFloor = 0;
bool emergencyStop = false;
bool doorOpen = false;
int state = 1;  // Initial state: IDLE (1), MOVING_UP (2), MOVING_DOWN (3), OPEN_DOOR (4), WAIT_CLOSE (5), EMERGENCY (6)
unsigned long doorOpenTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_UP, OUTPUT);
  pinMode(MOTOR_DOWN, OUTPUT);
  pinMode(DOOR_OPEN, OUTPUT);
  pinMode(DOOR_CLOSE, OUTPUT);
  pinMode(EMERGENCY_BUTTON, INPUT_PULLUP); // Internal pull-up for emergency button
  pinMode(FLOOR_BUTTONS_PORT, INPUT);  // External Request buttons
  pinMode(INSIDE_BUTTONS_PORT, INPUT);  // Inside Destination selection buttons
  pinMode(LED_PORT, OUTPUT);  // Floor LEDs
  // Initialization  (Set all outputs low)
  digitalWrite(MOTOR_UP, LOW);
  digitalWrite(MOTOR_DOWN, LOW);
  digitalWrite(DOOR_OPEN, LOW);
  digitalWrite(DOOR_CLOSE, LOW);
  cli(); // Disable interrupts globally to begin with (good practice)
  // Set interrupt triggers and enable individual interrupts
  attachInterrupt(digitalPinToInterrupt(7), EMERGENCY, RISING); //Example: INT7 on pin 7, trigger on rising edge
  attachInterrupt(digitalPinToInterrupt(5), fire, RISING); //Example: INT5 on pin 5, trigger on rising edge
  attachInterrupt(digitalPinToInterrupt(6), power, RISING); //Example: INT6 on pin 6, trigger on rising edge
  sei(); // Enable global interrupts
  // Initialize PWM
  // Example for Timer1 (adjust prescaler, etc. as needed):
  TCCR1A = _BV(COM1A1) | _BV(WGM11); // Non-inverting PWM, Mode 14 (fast PWM)
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // Prescaler = 8
  ICR1 = 0xFFFF; // Top value (16-bit timer)
  // Enable PWM outputs (adjust pins according to your setup)
  pinMode(9, OUTPUT); // Example PWM output 1
  pinMode(10, OUTPUT); // Example PWM output 2
  // Set PWM periods (adjust values as needed)
  PWM1_ChangePeriod(255); // Adjust function name to match your flowcode convention.
  PWM2_ChangePeriod(255); // Implement this and the one above for changing PWM periods.
  // Output 0 on all ports to minimize errors
  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
  PORTD = 0;
  PORTE &= ~(0x08);  //PORTE &= 0xF7; // Only clear bit 3 of PORTE, leave the others as-is
}

void IDLE()
{
  int portC = readFloorSensor(); // Read the floor sensor (you will need to implement this)
  // Check if the elevator is already at the requested floor
  if ((requestedFloor != 0) && (requestedFloor -1 == (portC & 0x0f)))  {
    state = 4; // Go to OPEN_DOOR state
    digitalWrite(LED_PORT, 1 << currentFloor ); // Turn on LED for current floor
    return;  //Exit IDLE - important!
  }
  bool combinedReq = (combinedreq & (1<< (portC & 0x0f)))>0 ; //Check if request at current floor
  if (combinedReq)
  {
    state = 4; // Go to OPEN_DOOR state;
    upp = 0; // Reset upp
  }
  else if (upp && (combinedreq & ~((1 << (portC & 0x0f)) - 1)) >0 )
  { // Check for higher requests if going up
    state = 2; // Go to GOUP state
    Serial.println("upp &&");
  }
  else if (upp && (combinedreq & ((1 <<(portC & 0x0f)) - 1))>0)
  { // Check for lower requests
    upp = 0;
    state = 3; // Go to GODOWN state
    Serial.println("!upp &&");
  }
  else if (!upp && (combinedreq & ((1 <<(portC & 0x0f)) - 1)) >0)
  {
    state = 3; // Go to GODOWN state
    Serial.println("!upp &&");
  }
  else if (!upp && (combinedreq & ~((1 <<(portC & 0x0f)) - 1))>0)
  {
    upp = 1;
    state = 2;// Go to GOUP state
    Serial.println("upp &&");
  }
  else
  {
    // No requests, stay in IDLE state
    upp = 1; // Set default direction to up when no requests
    state = 1;
  }
}

void loop()
{
    checkEmergency(); // Constantly check the emergency button, even during other states
    readRequests();
    switch (state)
    {
        case 1: // IDLE
          if (requestedFloor != 0)
          {
            moveElevator(requestedFloor);
          }
          break;
        case 2: // MOVING_UP
            if (analogRead(FLOOR_SENSOR) >= floorReadings[requestedFloor - 1])
          {
               stopMotor();  // Stop at the requested floor
               state = 4; // OPEN_DOOR
               currentFloor = requestedFloor;
               requestedFloor = 0;  // Clear request
            }
            break;
         case 3: // MOVING_DOWN
             if (analogRead(FLOOR_SENSOR) <= floorReadings[requestedFloor - 1])
           {
               stopMotor();
               state = 4; // OPEN_DOOR
               currentFloor = requestedFloor;
               requestedFloor = 0;
            }
            break;
        case 4: // OPEN_DOOR
          if (!doorOpen)
          { // Open the door if not already open
            openDoor();
            doorOpen = true;
            doorOpenTime = millis();  // Record the time the door opened
          }
          state = 5;  // WAIT_CLOSE (Always transition.  Door opens in a single loop cycle).
          break;
         case 5: // WAIT_CLOSE
             destinationFloor = getDestinationFloor(INSIDE_BUTTONS_PORT);
           if ((millis() - doorOpenTime >= 10000) ||  // Automatic close after 10s, OR
             (destinationFloor == currentFloor && (millis() - doorOpenTime > 1000)) || // Inside floor request is same, wait at least 1s and proceed, OR
               destinationFloor != 0 )
              { // Inside floor request made for different floor,
                closeDoor();
                doorOpen = false;
                state = destinationFloor != 0 ? (destinationFloor > currentFloor ? 2 : 3) : 1; // Move towards destination if any, or go to IDLE
                requestedFloor = destinationFloor != 0 ? destinationFloor : 0; // Move if there is a request, otherwise stay idle
                destinationFloor = 0;  // Reset destination
             }
              break;
         case 6: // EMERGENCY
          emergencyProcedure();
          break;
   }
}
void setMotorSpeed(int speed)
{
  if (speed > 0)
  {
    // Move up
    analogWrite(9, speed);
    analogWrite(10, 0);
  }
  else if (speed < 0)
  {
    // Move down
    analogWrite(9, 0);
    analogWrite(10, -speed); // Use the absolute value for PWM
  }
  else
  {
    // Stop
    analogWrite(9, 0);
    analogWrite(10, 0);
  }
}
// Emergency Handling
void checkEmergency()
{
  if (!digitalRead(EMERGENCY_BUTTON))
  {  // Active low
    if (!emergencyStop)
    {  // If emergency wasn't already triggered
      emergencyStop = true;
      state = 6; // EMERGENCY
    }
  }
}

void emergencyProcedure()
{
  // Turn on EMG LED (assuming B1 is the EMG LED pin)
  digitalWrite(B1, HIGH);
  digitalWrite(B5, HIGH); //Start opening door
  delay(5); //Add delay of 5ms
    while (digitalRead(B3) == LOW) {  // Wait for door to open
    // Consider adding a short delay or timeout to prevent infinite loop
      delay(10); // For example. You would want a proper timeout
  }
     digitalWrite(B5, LOW); // Stop the opening of the door
  // Find nearest floor
  int nearestFloor = determineNearestFloor();
  // Move to nearest floor
      // Decelerate while moving to the nearest floor
     int x = 128; // Example initial speed for deceleration
     int z = 128; // Example initial speed for deceleration
     while(currentFloor != nearestFloor){
         int portc = readFloorSensor();
        if (currentFloor < nearestFloor)
         {
            if(x>0)
          {
                 x--;
            }
           setMotorSpeed(x);
         }
        else
         {
          if(z > 0)
           {
            z--;
           }
           setMotorSpeed(-z);
         }
             switch (portc)
         {
          case 0:
            PORTD = B00111111;
            break;
          case 1:
            PORTD = B00000110;
            break;
          case 2:
            PORTD = B01011011;
            break;
          case 3:
            PORTD = B01001111;
            break;
        }
         delay(5); //Short Delay
     }
     setMotorSpeed(0);
  // Open the door at the nearest floor (This is the second door opening logic)
      digitalWrite(B5, HIGH);
      delay(5);
      while (digitalRead(B3) == LOW)
      {
          delay(10);  // Add timeout logic here as well (see openDoor() implementation)
      }
      digitalWrite(B5, LOW);
      digitalWrite(B6, LOW); // Ensure door closing mechanism is also off
  // Wait for EMG button release (assuming E7 is the EMG button pin)
  while (digitalRead(E7) == HIGH) { // While emergency button is pressed
    delay(100);  // Check button state regularly
  }
   digitalWrite(B1, LOW);  // Turn off EMG LED
   //Set B5 and B6 to 0 again
   digitalWrite(B5, LOW);
   digitalWrite(B6, LOW);
   state = 5; // Set state to closeDoor after emergency
  // ... (Logic to clear any pending requests or reset other variables)
}

void goUp()
{
  int x = 128; // Initial PWM value (adjust as needed)  - this should match max speed of motor PWM
  int portC;
  while (true)
  { //This simulates the loop at the start of goup
    portC = readFloorSensor();
    if ((combinedreq & (1 << portC)) > 0)
    {
      break; // Exit the loop if a request at this floor is found.
    }
    setMotorSpeed(x); // Set the motor speed using PWM
    // Implement logic to move the elevator one floor up
    delay(50); // Short delay (adjust as needed)  Might need longer based on elevator speed
  }
  // Deceleration loop
  while (x > 0)
  {
    if ((combinedreq & (1 << portC)) > 0)
    {  // Check for requests at current floor again!
      // If there's a request at the current floor, break out of the deceleration loop
      break;
    }
    x--;
    setMotorSpeed(x);      // Reduce motor speed gradually
    delay(5);         // Short delay for gradual deceleration
  }
  setMotorSpeed(0); // Stop motor
  currentFloor = targetFloor;
  state = 4;       // Transition to Open Door state
}

void goDown()
{
  int z = 0;      // Initial PWM value for down (start slow)
  int portC = 0;  //initial portC
  // Acceleration loop
  while (z < 128) {
    z++;
    setMotorSpeed(-z); // Negative speed for downward movement
    delay(5);        // Adjust delay as needed for smooth acceleration
  }
  setMotorSpeed(-128);
  //Loop and check sensor and update LEDs
  while (true){
    portC = readFloorSensor();
    switch (portC)
    {
      case 0:
        PORTD = B00111111;
      break;
      case 1:
        PORTD = B00000110;
      break;
      case 2:
        PORTD = B01011011;
      break;
      case 3:
        PORTD = B01001111;
      break;
    }
    if ((combinedreq & (1 << portC)) > 0)
    {
      //If there is request for current floor
      break;
    }
  }
  // Deceleration loop
  while (z > 0) {
    if ((combinedreq & (1 << portC)) > 0) {
      break; //Break if there is new request
    }
    z--;
    setMotorSpeed(-z); // Gradually decrease speed
    delay(5);
  }
  setMotorSpeed(0);  // Stop motor completely
  currentFloor = targetFloor;
  state = 4;  // Transition to OPEN_DOOR state
}

void openDoor()
{
  // Activate door opening mechanism
  digitalWrite(B5, HIGH);  // Assuming B5 controls door opening motor (adjust if needed)
  digitalWrite(B6, LOW);
  // Wait for door to fully open (using a sensor or timeout)
  while (digitalRead(B3) == LOW) { // Assuming B3 is the door open sensor (adjust if needed)
    // You might want to add a timeout here to prevent getting stuck if the sensor fails
    delay(10); //Optional delay
  }
  digitalWrite(B5, LOW);  // Deactivate door opening motor
  digitalWrite(B6, LOW);
  // Clear completed request
  combinedreq &= ~(1 << currentFloor);
  // Keep door open for 10 seconds unless closed by inside button
  unsigned long startTime = millis();
  int y = 0;
  while ( (digitalRead(E1) == LOW)  && (y < 10))
  { // E1 is the inside close button (adjust if needed)
    delay(1000); // Wait 1 second
    y++;
    // Consider adding a check for new external requests here to keep the door open longer.
  }
  //Close Door
  digitalWrite(B5, LOW);
  digitalWrite(B6, HIGH);
  while (digitalRead(B4) == LOW)
  { // B4 is close sensor. Replace/add timeout if necessary
    delay(10);
  }
  digitalWrite(B5, LOW);
  digitalWrite(B6, LOW);
  state = 1; // Return to IDLE state
}

void closeDoor()
{
  int weight;
  // Weight check and overload handling  (Adjust pin & logic based on your sensor)
  weight = analogRead(F0);  // Replace F0 with actual weight sensor pin
  while (weight > 512)
  {  // 512 is an example threshold. Calibrate as needed.
    PORTD = B01110001;      // Display "F" on 7-segment display (adjust segments as needed)
    weight = analogRead(F0); //re-read  the weight
    delay(200); //Check Weight after 200ms delay
  }
  // Update 7-segment display with current position
  // displayCurrentPosition(); // You'll need to implement this function
  showFloorOnSevenSegment(currentFloor, 0); // Example usage - assuming you implement this
  // Obstacle detection and wait (adjust pin B2 if needed)
  while (digitalRead(B2) == HIGH)
  {  // Assuming B2 is HIGH when obstacle detected
    // Wait for obstacle to be removed
    delay(100);  // Check regularly
  }
  // Activate door closing mechanism
  digitalWrite(B5, LOW);  // Assuming B5 needs to be low for closing, B6 HIGH
  digitalWrite(B6, HIGH);
  // Wait for door close sensor (B4) or timeout
  unsigned long closeStartTime = millis();
  while (digitalRead(B4) == LOW)
  {  // B4 is the close sensor.  Adjust pin if needed
    if (millis() - closeStartTime > 5000)
    { // Example 5-second timeout
      // Door close timeout. Handle the error appropriately (e.g., stop motor, display error)
      break;  // Exit loop if timeout occurs
    }
    delay(100);  // Check regularly
  }
  // Deactivate closing mechanism (important to prevent motor damage)
  digitalWrite(B5, LOW);
  digitalWrite(B6, LOW);
  state = 1; // Transition back to IDLE state
}

void moveElevator(int targetFloor)
{
  if (targetFloor > currentFloor)
  {
    analogWrite(MOTOR_UP, 200); // Start motor up (adjust speed if needed)
    analogWrite(MOTOR_DOWN, 0);
    state = 2;
  } else if (targetFloor < currentFloor)
  {
    analogWrite(MOTOR_DOWN, 200);
    analogWrite(MOTOR_UP, 0);
    state = 3;
  }
}

void stopMotor()
{
  analogWrite(MOTOR_UP, 0);  // Stop both motors
  analogWrite(MOTOR_DOWN, 0);
}

int readRequest()
{
  // Read floor requests (outside the elevator)
  int floor = getRequestedFloor(FLOOR_BUTTONS_PORT); // Assuming you have a function to decode floor from port
  if (floor != 0)
  {
    return floor;
  }
  // If no outside request, check inside requests if the door is open (state 4 = OPENDOOR)
  if (state == 4)
  {
    return getDestinationFloor(INSIDE_BUTTONS_PORT);
  }

  return 0; // No request
}

int getRequestedFloor(int portValue)
{
  if (portValue & 0x01) return 1;  // Bit 0: Floor 1
  if (portValue & 0x02) return 2;  // Bit 1: Floor 2
  if (portValue & 0x04) return 3;  // Bit 2: Floor 3
  if (portValue & 0x08) return 4;  // Bit 3: Floor 4
  return 0;  // No request
}

int getDestinationFloor(int portValue)
{
  // Implement your logic to read from PORTC (internal requests)
   if (portValue & 0x01) return 1;
    if (portValue & 0x02) return 2;
    // ... (add more)
    return 0;
}

int determineNearestFloor()
{
  int closestFloor = 0;
  int minDifference = 1024; // Initialize with max possible value
  int currentSensorReading = analogRead(FLOOR_SENSOR);
  for (int i = 0; i < NUM_FLOORS; i++)
  {
    int difference = abs(currentSensorReading - floorReadings[i]);
    if (difference < minDifference)
    {
      minDifference = difference;
      closestFloor = i + 1;
    }
  }
  return closestFloor;
}

int readFloorSensor()
{
  // Implement your logic to read from the floor sensor (e.g., PORTC)
  // and return the current floor number (0-based).
  // Example using analog sensor:
  int sensorValue = analogRead(FLOOR_SENSOR);
  // Map sensor value to floor number based on your calibration (Example)
  if (sensorValue < 100) {
    return 0;  // Ground floor
  } else if (sensorValue < 200) {
    return 1; // First floor
  } else if (sensorValue < 300){
    return 2;
  }else {
    return 3; // Top floor
  }
}
void readRequests()
{
  // Read requests from outside and inside the elevator
  byte outsideReq = PINA; // Read from PORTA (outside requests)
  byte insideReq = PING; // Read from PORTG (inside requests)
  //Combine the requests using bitwise OR.  Any bit set high in either
  //outsideReq or insideReq will be set high in combinedreq.
  combinedreq |= outsideReq;   // Use bitwise OR and assignment.  This preserves any
  combinedreq |= insideReq;   // previously set requests.
}
// Example implementation of a 7-segment display function.  You will likely
// need to adapt this based on your specific wiring and the type of 7-segment
// display you are using (common cathode or common anode).
void showFloorOnSevenSegment(int floor, int displayNum)
{ // displayNum is not used in this example
  switch (floor)
  {
    case 0: PORTD = B00111111; break;  // Example segments for "0"
    case 1: PORTD = B00000110; break;  // Example segments for "1"
    // ... add cases for other floors
    default: PORTD = B00000000; // Or display an error pattern
  }
}
