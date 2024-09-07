#include <Servo.h>

// LineScan Camera pin
#define SIPIN   5
#define CLKPIN  4
#define APIN    A2

// BLDC ESC pin 
#define PWMR    2
#define PWML    6

// LED pin
#define LEDR    14
#define LEDL    15

// Communication pin
#define SIGNAL0 7
#define SIGNAL1 8
#define SIGNAL2 9
#define RETURN0 16l
#define RETURN1 17  

// SR-04 pin
#define ECHO    13
#define TRIG    12

#define THRESHOLD_OUT   150 // 400
#define THRESHOLD_IN    50
#define THRESHOLD_STD   45
#define PIDPIXEL        70
#define currentSpeed    80
#define ULTRA_MIN       1
#define ULTRA_MAX       25
#define TIME_INTERVAL   500
#define NONE_CNT        2
#define TIME_STD        140
#define TIME_BOUND      80
#define RESET_T         570 //567

// for bldc esc
Servo MOTOR_L, MOTOR_R;

// for save rpi4 signal
uint8_t hexsignal = 0;

// for line scan camera
uint16_t line_pixel[128];
uint16_t pid_pixel[PIDPIXEL+1] = {0};
uint16_t hor_pixel[10] ={0};
uint16_t center_line[128] = {0};

// for tracking variable
uint16_t center = 0;
uint16_t center_l = 0;
uint16_t max_value = 0;

// for pid control
int P = 0;
int I = 0;
int D = 0;
int previousError = 0;
int PIDvalue = 0;
int error = 0;
float kp = .4;
float ki = 0;
float kd = 0.024;

// pid value
volatile int lsp = 0;
volatile int rsp = 0;

// for check line state
unsigned long left_line = 0;
unsigned long right_line = 0;

// line state variable
uint8_t line_state = 0; //0 is two solid, 1 is left solid , 2 is right solid

// parks state
bool park = 0;

// distance variable
volatile long distance = 0;

// time reset
unsigned long reset_time = 0;

// Core 0 setup
void setup(){
    Serial.begin(115200);
    pinMode(SIGNAL0, INPUT);
    pinMode(SIGNAL1, INPUT);
    pinMode(SIGNAL2, INPUT);
    pinMode(RETURN0, OUTPUT);
    pinMode(RETURN1, OUTPUT);
    digitalWrite(RETURN0, LOW);
    digitalWrite(RETURN1, LOW);
    delay(5000);
}

// Core 1 setup
void setup1(){
    pinMode(SIPIN, OUTPUT);
    pinMode(CLKPIN, OUTPUT);
    pinMode(APIN, INPUT);
    pinMode(LEDL, OUTPUT);
    pinMode(LEDR, OUTPUT);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    digitalWrite(SIPIN, LOW);
    digitalWrite(CLKPIN, LOW);

    MOTOR_L.attach(PWML);
    MOTOR_R.attach(PWMR);

    MOTOR_R.writeMicroseconds(1500);
    MOTOR_L.writeMicroseconds(1500);

    delay(5000);
} 

// Core0 - check distance
void loop(){
    distance = measureDistance();
}

// Core1 - lineScan, PID Control, Read signal from rpi4, etc...
void loop1(){
    linescan(); // Scanning Line, array check
    linefollow(); // PID Control
    hexsignal = (digitalRead(SIGNAL1)<<1) + (digitalRead(SIGNAL2)); // Read rpi4 Signal
    if(hexsignal == 0){ // 0 is Stop mode.
        MOTOR_R.writeMicroseconds(1500);
        MOTOR_L.writeMicroseconds(1500);
        line_state = 0;
        right_line = 0;
        left_line = 0;
        park = 0;
    }
    else if(hexsignal == 1){ // 1 is Drive mode.
        digitalWrite(RETURN0, LOW); 
        esc_drive();
    }
    delay(50); // for stable
}

void linescan(){
    // line scan camera setting
    digitalWrite(CLKPIN, LOW);
    digitalWrite(SIPIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(CLKPIN, HIGH);
    digitalWrite(SIPIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(CLKPIN, HIGH);
    digitalWrite(SIPIN, LOW);
    delayMicroseconds(1);
    // line scannig...
    max_value = 0;
    for (int i = 127; i >= 0; i--){
        digitalWrite(CLKPIN, LOW);
        digitalWrite(SIPIN, LOW);
        delayMicroseconds(3);
        digitalWrite(CLKPIN, HIGH);
        digitalWrite(SIPIN, LOW);
        delayMicroseconds(2);
        line_pixel[i] = analogRead(APIN);
    }
    digitalWrite(CLKPIN, LOW);
    digitalWrite(SIPIN, LOW);
    // restore for function
    for(int i = 0; i < 10 ; i++){
        hor_pixel[i] = line_pixel[i+63];
    }
    for(int i = 10; i < 128 ; i++){
        center_line[i] = line_pixel[i];
        if(line_pixel[i] > max_value){
            max_value = line_pixel[i];
        }
    }
    for(int i = 0; i < PIDPIXEL/2; i++){
        pid_pixel[i] = line_pixel[10 + i];
        pid_pixel[PIDPIXEL / 2 + 1 + i] = line_pixel[127 - PIDPIXEL/2 +i];
    }
}

void linefollow(){
    error = 0;
    center = 0;
    center_l = 0;
    uint16_t threshold = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    
    // check cds
    for(int i = 0; i < 10; i++){
        center += hor_pixel[i];
    }
    // check whit line
    for(int i = 0; i< 128 ; i++){
        center_l += center_line[i];
    }
    center /= 10;
    center_l /= 128;
    Serial.print(" MAX_value : ");
    Serial.print(max_value); // for debugging
    
    if(center >= THRESHOLD_STD){
        // not tunnel
        threshold = THRESHOLD_OUT;
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDL, LOW);
    }
    else{
        // tunnel
        threshold = THRESHOLD_IN;
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDL, HIGH);
    }
    for(int i = 0; i < PIDPIXEL/2; i++){
        // cal pid error, line state check
        if(pid_pixel[i] >= threshold){
            error += i;
            left++;
        }
        if(pid_pixel[PIDPIXEL/2+1+i] >= threshold){
            error -= i;
            right++;
        }
    }
    Serial.print(" LEFT: ");
    Serial.print(left);
    Serial.print(" RIGHT: ");
    Serial.print(right);
    // check line state
    if(left < NONE_CNT && left_line == 0){
        left_line = millis();
    }
    else if(left > NONE_CNT && left_line != 0){
        unsigned long temp_time = millis();
        if(temp_time - left_line > TIME_STD && temp_time - left_line < TIME_STD + TIME_BOUND){
            line_state = 2;
            reset_time = temp_time;
        }
        left_line = 0;
    }
    if(right < NONE_CNT && right_line == 0){
        right_line = millis();
    }
    else if(right > NONE_CNT && right_line != 0){
        unsigned long temp_time = millis();
        if(temp_time - right_line > TIME_STD && temp_time - right_line < TIME_STD + TIME_BOUND){
            line_state = 1;
            reset_time = temp_time;
        }
        if(temp_time - right_line > RESET_T - TIME_BOUND * 1.4 && temp_time - right_line < RESET_T + 10 && left >= NONE_CNT){
            park = 1;
        }
        right_line = 0;
    }
    if(millis() - reset_time > RESET_T && line_state != 0){
        line_state = 0;
        park = 0;
    }
    // PID
    P = error;
    I = I +error;
    D = error - previousError;

    PIDvalue = (kp * P) + (ki * I) + (kd * D);
    previousError = error;

    lsp = currentSpeed + PIDvalue/(PIDPIXEL/20);
    rsp = currentSpeed - PIDvalue/(PIDPIXEL/20);
    // limit speed
    if (lsp > 200) {
        lsp = 200;
    }
    if (lsp < 0) {
        lsp = 0;
    }
    if (rsp > 200) {
        rsp = 200;
    }
    if (rsp < 0) {
        rsp = 0;
    }
}

long measureDistance() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    long duration = pulseIn(ECHO, HIGH, 10000);
    long distance = duration * 0.034 / 2;

    return distance;
}

void esc_drive(){  
    if(distance >= ULTRA_MIN && distance <= ULTRA_MAX){
        if(line_state == 1){
            MOTOR_L.writeMicroseconds(1500);
            MOTOR_R.writeMicroseconds(1500);
            delay(300);
            MOTOR_L.writeMicroseconds(1600);
            MOTOR_R.writeMicroseconds(1400);
            delay(500);
            MOTOR_L.writeMicroseconds(1500);
            MOTOR_L.writeMicroseconds(1500);
            delay(100);
            MOTOR_L.writeMicroseconds(1580);
            MOTOR_R.writeMicroseconds(1580);
            delay(700);
            MOTOR_L.writeMicroseconds(1400);
            MOTOR_R.writeMicroseconds(1600);
            delay(450);
            MOTOR_L.writeMicroseconds(1580);
            MOTOR_R.writeMicroseconds(1580);
            delay(50);
            line_state = 0;
        }
        else if(line_state == 2){
            MOTOR_L.writeMicroseconds(1500);
            MOTOR_R.writeMicroseconds(1500);
            delay(300);
            MOTOR_L.writeMicroseconds(1400);
            MOTOR_R.writeMicroseconds(1600);
            delay(500);
            MOTOR_L.writeMicroseconds(1500);
            MOTOR_L.writeMicroseconds(1500);
            delay(100);
            MOTOR_L.writeMicroseconds(1580);
            MOTOR_R.writeMicroseconds(1580);
            delay(300);
            MOTOR_L.writeMicroseconds(1600);
            MOTOR_R.writeMicroseconds(1400);
            delay(450);
            MOTOR_L.writeMicroseconds(1580);
            MOTOR_R.writeMicroseconds(1580);
            delay(200);
            line_state = 0;
        }
        else if(line_state == 0){
            MOTOR_L.writeMicroseconds(1500);
            MOTOR_R.writeMicroseconds(1500);
            delay(3000);
        }
        center_l = 0;
    }
    MOTOR_L.writeMicroseconds(1580);
    MOTOR_R.writeMicroseconds(1580);
    if(park == 1){
        MOTOR_L.writeMicroseconds(1500);
        MOTOR_R.writeMicroseconds(1500);
        delay(500); 
        MOTOR_L.writeMicroseconds(1600);
        MOTOR_R.writeMicroseconds(1400);
        delay(700);
        MOTOR_L.writeMicroseconds(1500);
        MOTOR_R.writeMicroseconds(1500);
        delay(500);
        MOTOR_L.writeMicroseconds(1550);
        MOTOR_R.writeMicroseconds(1550);
        delay(1000);
        MOTOR_L.writeMicroseconds(1500);
        MOTOR_R.writeMicroseconds(1500);
        delay(1100);
        MOTOR_L.writeMicroseconds(1400);
        MOTOR_R.writeMicroseconds(1400);
        delay(650);
        MOTOR_L.writeMicroseconds(1400);
        MOTOR_R.writeMicroseconds(1600);
        delay(620);
        MOTOR_L.writeMicroseconds(1500);
        MOTOR_R.writeMicroseconds(1500);
        delay(100);
        digitalWrite(RETURN0, HIGH);
        delay(1600);
        MOTOR_L.writeMicroseconds(1580);
        MOTOR_R.writeMicroseconds(1580);
        delay(100);
        park = 0;
        center_l = 0;
    }
    MOTOR_L.writeMicroseconds(1500+lsp);
    MOTOR_R.writeMicroseconds(1500+rsp);
}