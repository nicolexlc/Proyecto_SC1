#include <NewPing.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>



#define BUZZER        4
#define RED           A5
#define TRIGGER_PIN   5
#define ECHO_PIN      6
#define IN4           7
#define IN3           8
#define PWMA          9
#define IN2           12
#define IN1           13
#define PWMB          3


int RX = 10;
int TX = 11;


#define MAX_DISTANCE  400


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
SoftwareSerial BTserial(RX, TX);

volatile int contador = 0;
int value = 0, encoder = 0;

//PID constants

double Iterm;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, val, setPoint, outputconstrain, ITerm, outMax, outMin, mapencoder, rpm;
double cumError, rateError;


void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(PWMB, OUTPUT); //MOTOR PWM
  pinMode(IN3, OUTPUT); //MOTOR PWM
  pinMode(IN4, OUTPUT); //MOTOR PWM
  pinMode(PWMA, OUTPUT); //MOTOR PWM
  pinMode(IN1, OUTPUT); //MOTOR PWM
  pinMode(IN2, OUTPUT); //MOTOR PWM


  BTserial.begin(9600);
  Serial.begin(115200);
  attachInterrupt(0, interrupcion0, RISING); //pin 2
}

void loop() {
  int distance = sonar.ping_cm();

  encoder = contador * 6; //contador * 60/numero de aspas(60 = numero de segundos en un minuto)
  mapencoder = map(encoder, 0, 1000, 0, 255);
  contador = 0;


  if (distance >= 101 ) {          // nada
    digitalWrite(BUZZER, LOW);
    digitalWrite(RED, LOW);
    setPoint = 100;
    rpm = computePID(mapencoder, 0, 255, 5, 0.01, 2);
    analogWrite(PWMA, rpm);
    analogWrite(PWMB, 255);
  }
  else if (distance >= 51 ) {          // led rojo
    digitalWrite(BUZZER, LOW);
    digitalWrite(RED, HIGH);
    analogWrite(PWMA, 191);
    analogWrite(PWMB, 191);
  } else if (distance >= 26 ) {          // buzzer
    digitalWrite(BUZZER, HIGH);
    digitalWrite(RED, LOW);
    analogWrite(PWMA, 127);
    analogWrite(PWMB, 127);
  } else if (distance >= 0) {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(BUZZER, LOW);
    digitalWrite(RED, LOW);
  }



  delay(499); //1 segundo numero de revoluciones
  
  Serial.print(encoder); //Numero de aspas , en este caso 2 contador * 60/2

  Serial.print(" RPM ");

  Serial.print(distance); //serial
  Serial.print(" cm   ");



  Serial.print(" Error: ");
  Serial.print(error);
  Serial.print("  ");
  Serial.print(" rpm2: ");
  Serial.print(mapencoder);
  Serial.print("  ");
  Serial.println(rpm);




  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);


  BTserial.print("E");
  BTserial.print(distance);
  BTserial.print(",");
  BTserial.print(mapencoder);
  BTserial.print(",");
  BTserial.print(rpm);
  BTserial.print("\n");


  if (BTserial.available()) {
    Serial.write(BTserial.read());
  }
  if (Serial.available()) {
    BTserial.write(Serial.read());
  }


}

void interrupcion0 () {
  contador ++;
}




double computePID(double inp, double Min, double Max, double kp, double ki, double kd) {

  currentTime = millis(); //get current time
  elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation

  error = inp - setPoint; // determine error
  cumError += ki * error * elapsedTime; // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = kp * error + cumError + kd * rateError; //PID output

  lastError = error; //remember current error
  previousTime = currentTime; //remember current time

  if (out > Max) {
    out = Max;
  } else if (out < Min) {
    out = Min;
  }

  if (cumError > Max) {
    cumError = Max;
  } else if (cumError < Min) {
    cumError = Min;
  }

  return out; //have function return the PID output

}
