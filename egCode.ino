/* 
 * Slim sumo's top quality code for
 * UNSW MTRNSoc Sumo Bot competition 2017
 *
 * Luke Dennis <luke.j.dennis@gmail.com>
 *
 */

#define TESTING 1 //set 0/1 for print statements and other test func

// #defines for ultrasonic and motor calculations
#define SOUND 0.343  // speed of sound = 0.343 mm/us
#define MIN_DIST 1
#define MAX_DIST 1000 // max distance reading in mm (1m)
#define MAX_TIME 2*MAX_DIST/SOUND // max distance reading in mm
#define MAX_DRIVE 255

//pin defines
// opponent distance sensors digital
const int trig1 = 12;
const int echo1 = 11;

// Motor driver sheild
const int leftPWM = 5;
const int in1 = 6;
const int in2 = 7;
const int in3 = 8;
const int in4 = 9;
const int rightPWM = 10;

// Line detectors
const int lineOut = 4;
//const int frontLine = ;
//const int backLine = ;

// Button
// const int button = ;

// Global variables
// Ultrasonic reading 
int distance;
// line sensor 
//bool onLine;


void setup() {
    // pin setupA
    // ultrasonic
    pinMode(trig1, OUTPUT);
    pinMode(echo1, INPUT);
    // motor driver
    pinMode(leftPWM, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(rightPWM, OUTPUT);
    // line sensor
    pinMode(lineOut, INPUT);
    // lets you print to monitor
    Serial.begin(9600);
}

void loop() {
    
    // The stuff that gets done again and again and again...
    for (int speed = 0; speed <= MAX_DIST; speed++) {
     Serial.println(speed);
     drive(speed);
     delay(100);
    }
}

// takes ultrasonic reading and returns true if within range
bool getDistance(const int echo, const int trig) {
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

    distance = range(MIN_DIST, distance, MAX_DIST);

    return (distance != 0) ? true : false;
}

bool getLine(const int lineOut) {
   return digitalRead(lineOut) == HIGH ? true : false; 
}

void drive(int dist) {

    // set left HIGH-LOW && PWM % of max dist
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(leftPWM, MAX_DRIVE*dist/MAX_DIST);

    // set right HIGH-LOW && PWM % of max dist
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(rightPWM, MAX_DRIVE*dist/MAX_DIST);
}

void stopDrive() {

    //set left LOW-LOW && PWM 0
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(leftPWM, 0);

    //set right LOW-LOW && PWM 0
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(rightPWM, 0);
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

