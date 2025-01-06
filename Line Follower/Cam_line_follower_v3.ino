#include "Servo.h"

// Motor Driver used: TA6586 

class motor{
  
  String ID;
  int IN_1_PIN;
  int IN_2_PIN;
  int ENC_A_PIN;
  int ENC_B_PIN;
  int counter;
  int PWM_LIM;
  int counts_per_rotation;

  public:
  motor(String ID, int IN_1_PIN, int IN_2_PIN, int ENC_A_PIN, int ENC_B_PIN, int counts_per_rotation = 420){
    
    // IN_1_PIN must be the PWM pin
    
    this->ID = ID;
    this->IN_1_PIN = IN_1_PIN;
    this->IN_2_PIN = IN_2_PIN;
    this->ENC_A_PIN = ENC_A_PIN;
    this->ENC_B_PIN = ENC_B_PIN;
    this->counter = 0;
    this->PWM_LIM = 100;
    this->counts_per_rotation = counts_per_rotation;

    pinMode(this->ENC_A_PIN, INPUT);
    pinMode(this->ENC_B_PIN, INPUT);
    pinMode(this->IN_1_PIN, OUTPUT);
    pinMode(this->IN_2_PIN, OUTPUT);
  }

  int get_ENC_A_PIN(){
    return this->ENC_A_PIN;
  }
  
  void read_Encoder(){
    if(digitalRead(this->ENC_B_PIN) > 0){
      counter++;
    }
    else{
      counter--;
    }
//    Serial.print(ID+": ");
//    Serial.println(counter);
  }
  
  void drive(int PWM){
    /*
    PWM>0 ==> Counter-clockwise rotation
    */
    PWM = min(PWM, PWM_LIM);
    PWM = max(PWM, -PWM_LIM);

    if(PWM>0){
      digitalWrite(IN_2_PIN, 1);
      digitalWrite(IN_1_PIN, 0);
    }
    else if(PWM<0){
      digitalWrite(IN_1_PIN, 1);
      digitalWrite(IN_2_PIN, 0);
    }
    else{
      digitalWrite(IN_1_PIN, 0);
      digitalWrite(IN_2_PIN, 0);
    }
    
  }
  
  void controlAnglePID(float targetAngle, float kp, float ki, float kd) {
    long target_count = targetAngle * (this->counts_per_rotation / 360.0); // Convert target angle to ticks
    long error = target_count - this->counter;
    static float integral = 0;
    static float prevError = 0;

    // Proportional term
    float P = kp * error;

    // Integral term
    integral += ki * error;

    // Derivative term
    float derivative = kd * (error - prevError);

    // PID control signal
    long controlSignal = P + integral + derivative;
    
    Serial.println("P: "+String((int)(P))+" I: "+String((int)(integral))+" D: "+String((int)(derivative)));
    
    // Update motor speed
    if(controlSignal>0)
    controlSignal = max(40,controlSignal);
    else if(controlSignal<0)
    controlSignal = min(-40,controlSignal);
    
    this->drive(controlSignal);

//    Serial.println(controlSignal);

    // Save current error for next iteration
    prevError = error;
  }

  
};

// String ID, int IN_1_PIN, int IN_2_PIN, int ENC_A_PIN, int ENC_B_PIN
motor left("LEFT", 6, 7, 2, 4);
motor right("RIGHT", 9, 8, 3, 5);
Servo servo;
int SERVO_PIN = 10;
int SPRAY_DOWN_ANGLE = 100;
int SPRAY_UP_ANGLE = 0;

void read_Helper_Left(){
  left.read_Encoder();
}
void read_Helper_Right(){
  right.read_Encoder();
}


void forward(int PWM = 50){
  left.drive(PWM);
  right.drive(-PWM);
}
void backward(int PWM = 50){
  left.drive(-PWM);
  right.drive(PWM);
}
void turn_right(int PWM = 50){
  left.drive(PWM);
  right.drive(PWM);
}
void turn_left(int PWM = 50){
  left.drive(-PWM);
  right.drive(-PWM);
}
void stop_motor(){
  left.drive(0);
  right.drive(0);
}

void spray_down(){
  servo.write(SPRAY_DOWN_ANGLE);
}

void spray_up(){
  servo.write(SPRAY_UP_ANGLE);
}

void setup() {
  Serial.begin(2000000); 
  Serial.setTimeout(2);

  attachInterrupt(digitalPinToInterrupt(left.get_ENC_A_PIN()), read_Helper_Left, RISING); 
  attachInterrupt(digitalPinToInterrupt(right.get_ENC_A_PIN()), read_Helper_Right, RISING); 
//  right.drive(60);
//  while(!Serial.available());
  delay(1000);
//  right.drive(0);
  servo.attach(SERVO_PIN);
  servo.write(SPRAY_UP_ANGLE);
  
}
void loop() {
//  forward();
//  return;
  while(!Serial.available());
  String input = Serial.readString();
  if(input.indexOf("VL")!=-1){
  //  Serial.println(input);
    int leftPWM = input.substring(input.indexOf("VL")+2, input.indexOf("VR")).toInt();
    int rightPWM = input.substring(input.indexOf("VR")+2, input.length()).toInt();
    Serial.println(leftPWM);
    Serial.println(rightPWM);
    if(abs(leftPWM)<40)
    leftPWM = 40*constrain(leftPWM,-1,1);
    left.drive(leftPWM);
    if(abs(rightPWM)<40)
    rightPWM = 40*constrain(rightPWM,-1,1);
    right.drive(rightPWM);
  }
  else if(input.indexOf("REV")!=-1){
    int turn_vel = input.substring(input.indexOf("REV")+3, input.length()).toInt();
    turn_right(turn_vel);
    Serial.println(input);
  }
  else if(input.indexOf("MAN")!=-1){
    int op = input.substring(input.indexOf("MAN")+3, input.length()).toInt();
    Serial.println(op);
    switch(op){
      case 5: {
        stop_motor();
        break;
      }
      case 2: {
        backward();
        break;
      }
      case 8: {
        forward();
        break;
      }
      case 4: {
        turn_left();
        break;
      }
      case 6: {
        turn_right();
        break;
      }
      case 3: {
        spray_down();
        break;
      }
      case 9: {
        spray_up();
        break;
      }
    }
    
  }
    
//    delay(2000);
//    stop_motor();    
  
//  left.controlAnglePID(180, 0.5, 0.00001, 0.1); // Example usage: control left motor to reach 90 degrees
//  right.controlAnglePID(180, 0.5, 0.00001, 0.1); 
}
