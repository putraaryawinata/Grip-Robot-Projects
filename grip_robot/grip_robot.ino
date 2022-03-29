#include <math.h>
#include <Servo.h>
Servo servoA;
Servo servoB;
Servo servoC;

//Inverse Kinematics
double pi = 3.14159265;
double convertDegtoRad(double degree) {
  return degree * pi / 180;
}
double convertRadtoDeg(double radian) {
  return radian * 180 / pi;
}
double length_1 = 4; //change this value to the actual length of lever 1
double length_2 = 4; //change this value to the actual length of lever 2
double theta2(double x, double y){
  double theta;
  theta = acos((x*x + y*y - length_1*length_1 - length_2*length_2) / (2*length_1*length_2));
  return theta;
}
double theta1(double x, double y) {
  double theta;
  theta = atan2(y,x) - atan2(length_2 * sin(theta2(x, y)), length_1 + length_2 * cos(theta2(x, y)));
  return theta;
}

//Code of Setup and Loop
void setup() {
  // put your setup code here, to run once:
  servoA.attach(3);
  servoB.attach(5);
  servoC.attach(9);
  Serial.begin(9600);
}

void Position(double a, double r, double z) {
  double b, c;
  b = convertRadtoDeg(theta1(r,z));
  c = convertRadtoDeg(theta2(r,z));
  servoA.write(a);
  servoB.write(b);
  servoC.write(c);
  String monitoring = "Current position: a=" + String(a) + "; r=" + String(r) + "; z=" + String(z);
  Serial.println(monitoring);
}

void loop() {
  // put your main code here, to run repeatedly:
  Position(0, 0, 0);
  delay(1000);
}
