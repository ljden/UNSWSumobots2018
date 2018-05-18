/* 
 * UNSW MTRNSoc Sumo Bot competition 2018
 * File created 04/04/18
 * Last modified 11/05/18
 * Author: Luke Dennis 
 *
 * Code provided by the UNSW Mechatronics Society
 *  as reference for students
 */

// #defines for easy reading
#define SOUND 0.343  // for mm: speed of sound = 0.343 mm/us
//#define SOUND 0.0343  // for cm: speed of sound = 0.0343 cm/us
#define MIN_DIST 1
#define MAX_DIST 1000 // max distance reading in mm (1m)
//#define MAX_DIST 100 // max dist reading in cm (1m)
#define MAX_TIME 2*MAX_DIST/SOUND // max distance reading (don't wait for ping)
#define MAX_SPEED 255
#define SECOND 1000
#define COUNTDOWN 3*SECOND
// Have #define so if we change field from black to white 
//  all you need to change is this variable
#define BLACK HIGH
#define WHITE LOW
#define ON_LINE WHITE

// Pin constants

// Button
const int buttonPin = 2;

// Line detectors
const int linePin = 4;

// Motor driver sheild
//  L & R assumes heatsink is @ back of vehicle
const int rightPWM = 5;
const int in1 = 6;
const int in2 = 7;
const int in3 = 8;
const int in4 = 9;
const int leftPWM = 10;

// opponent distance sensors digital
const int trigPin = 11;
const int echoPin = 12;

// Global variables

// Start sequence
bool startGame = false;
unsigned long startTime = 0;

// Ultrasonic reading 
unsigned int distance = 0;

// line sensor 
bool onLine = false;
unsigned long timeOnLine = 0; 


void setup() {
    // pin setup
    // Button
    pinMode(buttonPin, INPUT);

    // Ultrasonic
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    // motor driver
    pinMode(rightPWM, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(leftPWM, OUTPUT);
    
    // line sensor
    pinMode(linePin, INPUT);
    
    // lets you print to monitor
    Serial.begin(9600);
    
    // wait for button press to start
    // TODO: TEST THIS!!!
    while(!startGame) {
        Serial.println("Waiting for start...");
        if(digitalRead(buttonPin) == HIGH) {
            startGame = true;
            startTime = millis();
        }
    }
    
    // 3 second countdown
    while(timeDiff(startTime) < COUNTDOWN) {
        Serial.print("Starting in:");
        Serial.println((COUNTDOWN - int(timeDiff(startTime)))/SECOND + 1);
    }
    Serial.println("Game On!");
}

void loop() {
    getDistance(echoPin, trigPin); // check distance
    getLine(linePin); // check line status
    
    Serial.print("Distance -> ");
    Serial.print(distance);
    Serial.print(" On boarder line -> ");
    Serial.print(onLine);

    if (!onLine) { // if not on line
        drive(MAX_SPEED*0.5, MAX_SPEED*0.5); // Drive at 50% speed
    } else {
        drive(0,0); // Stop driving when on line
    }
}

// takes ultrasonic reading and returns true if within range
void getDistance(const int echo, const int trig) {
    long duration;

    // Flush trig 
    digitalWrite(trig, LOW);
    delayMicroseconds(2);

    // Pulse trig
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    duration = pulseIn(echo, HIGH, MAX_TIME);

    distance = duration * SOUND / 2;

    distance = range(MIN_DIST, distance, MAX_DIST); // get rid of garbage data
}

void getLine(const int line) {
    // none of your logic will have to change for black field vs white field
    // only thing that has to change is the hash define up the top
    int lineState = digitalRead(line);
    if (lineState == ON_LINE) {
        onLine = true;
    } else {
        onLine = false;
    }
}

void drive(int leftSpeed, int rightSpeed) {
    
    if (rightSpeed > 0) {
        // set right HIGH-LOW - forwards
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        // set right LOW-HIGH - reverse
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    // set absolute val of rightSpeed
    analogWrite(rightPWM, abs(rightSpeed));
    
    if (leftSpeed > 0) {
        // set right HIGH-LOW - forwards
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    } else {
        // set right LOW-HIGH - reverse
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }
    // set absolute val of leftSpeed
    analogWrite(leftPWM, abs(leftSpeed));
}

// Cap result to eliminate garbage readings
int range(int min, int num, int max) {
    if (num < min) { 
        num = 0; 
    } else if (num > max) {
        num = 0;
    }
    return num;
}

// helper function for time 
unsigned long timeNow() {
    return millis() - startTime;
}

// Helper function for checking elapsed time
unsigned long timeDiff(unsigned long oldTime) {
    return millis() - oldTime;
}
