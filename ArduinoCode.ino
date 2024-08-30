#include <Servo.h>

// Define pin numbers
#define PIR_PIN     2  // Pin connected to PIR sensor output (PD2)
#define TRIG_PIN    7  // Pin connected to Ultrasonic sensor Trig (PD7)
#define ECHO_PIN    8  // Pin connected to Ultrasonic sensor Echo (PB0)
#define SERVO_PIN   9  // Pin connected to Servo motor (PB1)

// Define port and bit masks
#define PIR_PIN_MASK     (1 << PD2)
#define TRIG_PIN_MASK    (1 << PD7)
#define ECHO_PIN_MASK    (1 << PB0)
#define SERVO_PIN_MASK   (1 << PB1)

// Define constants
#define CLOSE_SERVO_POSITION 0
#define OPEN_SERVO_POSITION 90
#define DEBOUNCE_TIME 200/1000.0   // 200 milliseconds debounce time
#define BAUD_RATE 9600     // Baud rate for serial communication

// Define constants for the ultrasonic sensor
#define SOUND_SPEED_CM_PER_US 0.034  // Speed of sound in cm/us
#define TRIG_PULSE_DURATION 10       // Trigger pulse duration in microseconds
#define TRIG_PULSE_SETTLE 2          // Settling time before sending the trigger pulse

#define FIVE_SECONDS 10    // 5 seconds in half-second increments
#define TEN_SECONDS 20    // 10 seconds in half-second increments

Servo myServo;              // Create a Servo object

bool motionDetected = false; // Flag to track if motion is detected
bool motionEnded = false; // Flag to track if motion has ended
unsigned long motionEndTime = 0; // Stores the time when motion ended
long duration = 0; 
int distance = 0; 
unsigned long timerSeconds = 0; // Tracks time in half-second increments using TIMER2
unsigned long lastDebounceTime = 0;  // Last time the ISR was triggered

bool printFlag = false;  // Flag to indicate that it's time to print
int pirStatus = 0;       // Variable to store PIR status for printing
unsigned int overflowCounter = 0; // Counter to accumulate overflows

void configureTimer2() {
    cli(); // Disable global interrupts

    TCCR2A = 0;  // Clear Timer/Counter Control Registers
    TCCR2B = 0;
    TCNT2 = 0;   // Initialize counter value to 0

    OCR2A = 255;  // Set compare match register for maximum value
                   // OCR2A = 255 for maximum 8-bit count

    TCCR2A |= (1 << WGM21);   // Turn on CTC mode
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  // Set CS22, CS21, and CS20 bits for 1024 prescaler
    TIMSK2 |= (1 << OCIE2A);  // Enable Timer Compare Interrupt

    sei(); // Enable global interrupts
}

void setup() {
    DDRD &= ~PIR_PIN_MASK;   // Set PIR_PIN as input (clear the bit)
    DDRD |= TRIG_PIN_MASK;   // Set TRIG_PIN as output (set the bit)
    DDRB &= ~ECHO_PIN_MASK;  // Set ECHO_PIN as input (clear the bit)
    DDRB |= SERVO_PIN_MASK;  // Set SERVO_PIN as output (set the bit)
  
    Serial.begin(BAUD_RATE);  // Use the defined baud rate

    myServo.attach(SERVO_PIN); // Attach the servo motor to the specified pin

    myServo.write(CLOSE_SERVO_POSITION); // Start with the servo in the closed position

    attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionISR, CHANGE); // Attach an interrupt to the PIR pin to detect motion

    configureTimer2(); // Configuring Timer2
}

void loop() {
    if (printFlag) {
        // Handle printing in the main loop
        if (motionDetected || (motionEnded && (timerSeconds - motionEndTime <= FIVE_SECONDS))) {
            measureDistance();  // Measure distance using the ultrasonic sensor

            // Print the distance and PIR status to the serial monitor
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.print(" cm, PIR: ");
            Serial.println(pirStatus);
        }
        printFlag = false;  // Clear the flag after printing
    }

    // Close the servo after 10 seconds of no motion
    if (motionEnded && (timerSeconds - motionEndTime > TEN_SECONDS)) {
        myServo.write(CLOSE_SERVO_POSITION); 
        motionEnded = false; 
    }

    // Handle serial input for controlling the servo
    if (Serial.available() > 0) {
        char receivedChar = Serial.read();
        if (receivedChar == '1') {
            myServo.write(OPEN_SERVO_POSITION); 
        } else if (receivedChar == '0') {
            myServo.write(CLOSE_SERVO_POSITION);
        }
    }
}

void motionISR() {
    unsigned long currentTime = timerSeconds;  // Get the current time in half-seconds

    // Check if the debounce time has passed
    if (currentTime - lastDebounceTime >= (DEBOUNCE_TIME) { 
        if (PIND & PIR_PIN_MASK) {        // Using bitwise operation to check the PIR sensor status
            motionDetected = true;        // Set motionDetected to true if motion is detected
            motionEnded = false;          // Reset motionEnded since motion is still ongoing
            pirStatus = 1;                // Update PIR status to indicate motion detected
        } else {
            motionDetected = false;       // Set motionDetected to false if no motion is detected
            motionEnded = true;           // Set motionEnded to true
            motionEndTime = currentTime;  // Record the time when motion ends
            pirStatus = 0;                // Update PIR status to indicate no motion detected
        }

        lastDebounceTime = currentTime;  // Update the last debounce time
    }
}

ISR(TIMER2_COMPA_vect) {
    overflowCounter++;
    if (overflowCounter >= 31) { // Every 31 overflows = 0.5 seconds
        timerSeconds++;  // Increment half-second counter
        printFlag = true; // Set the print flag
        overflowCounter = 0;  // Reset the counter
    }
}

void measureDistance() {
    // Clear the trigger pin before sending a pulse
    PORTD &= ~TRIG_PIN_MASK;
    delayMicroseconds(TRIG_PULSE_SETTLE);

    // Send a 10 microsecond pulse to trigger the ultrasonic sensor
    PORTD |= TRIG_PIN_MASK;
    delayMicroseconds(TRIG_PULSE_DURATION);
    PORTD &= ~TRIG_PIN_MASK;

    // Measure the duration of the pulse on the echo pin
    duration = pulseIn(ECHO_PIN, HIGH);

    // Calculate the distance based on the duration
    // Distance in cm = (duration in microseconds) * (speed of sound in cm/us) / 2
    // Dividing by 2 accounts for the round-trip of the sound wave
    if (duration > 0) {
        distance = duration * SOUND_SPEED_CM_PER_US / 2;
    } else {
        distance = -1;  // Set to -1 to indicate an error or no valid echo received
    }
}
