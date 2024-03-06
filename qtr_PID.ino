#include <AFMotor.h>
#include <QTRSensors.h>

AF_DCMotor motor1(4);  // Motor 1 -> Sol ön
AF_DCMotor motor2(3);  // Motor 2 -> Sağ ön
AF_DCMotor motor3(1);  // Motor 3 -> Sol arka
AF_DCMotor motor4(2);  // Motor 4 -> Sağ arka

#define rightBaseSpeed 100
#define leftBaseSpeed 100
#define rightMaxSpeed 170
#define leftMaxSpeed  170


QTRSensors qtr;

const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];
unsigned int position;

// PID Sabitleri
double Kp = 6;  // Oransal sabit
double Ki = 0.2;  // İntegral sabit
double Kd = 0.78; // Türev sabit

double previousError = 0;
double integral = 0;

byte BASE_SPEED = 80;

void setup() {
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {44, 42, 40, 38, 36, 34, 32, 30}, SensorCount);
  qtr_calibrate();

  delay(2000);
}

void loop() {
  qtr.read(sensorValues); // Sensör değerlerini oku

  position = qtr.readLineBlack(sensorValues); // Siyah çizgiyi takip et
  
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println("\n ******************************************************** \n");

  int pidDegeri = pidHesapla(position);
  Serial.println(pidDegeri);
  motorHizlariniAyarla(pidDegeri);

}

int pidHesapla(int position) {
  // PID hata hesaplama
  int hata = position - 3500;
  integral += hata;

  // PID hesaplama
  int pidDegeri = (Kp * hata) + (Ki * integral) + (Kd * (hata - previousError));

  // Bir sonraki iterasyon için önceki hatayı güncelle
  previousError = hata;

  return pidDegeri;
}

void motorHizlariniAyarla(int pidDegeri) {
  // PID değerine göre motor hızlarını ayarla

  int rightMotorSpeed = rightBaseSpeed + (pidDegeri * 0.0019);
  int leftMotorSpeed = leftBaseSpeed - (pidDegeri * 0.0019);


  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < rightBaseSpeed - 50 ) rightMotorSpeed = rightBaseSpeed;
  if (leftMotorSpeed < leftBaseSpeed - 50) leftMotorSpeed = leftBaseSpeed;
  Serial.println("Position: " + String(position));
  Serial.println("Sağ motor pwm : " + String(rightMotorSpeed));
  Serial.println("Sol motor pwm : " + String(leftMotorSpeed));
  Serial.println("\n ******************************************************** \n");


  motor1.setSpeed(leftMotorSpeed);
  motor2.setSpeed(rightMotorSpeed);
  motor3.setSpeed(leftMotorSpeed);
  motor4.setSpeed(rightMotorSpeed);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  
/*
  if(rightMotorSpeed - leftMotorSpeed >= 50 ){
    
    motor1.setSpeed(BASE_SPEED);
    motor2.setSpeed(BASE_SPEED);
    motor3.setSpeed(BASE_SPEED);
    motor4.setSpeed(BASE_SPEED);
    
    motor1.run(BACKWARD); //SOL ÖN
    motor2.run(FORWARD);
    motor3.run(BACKWARD); //SOL ARKA
    motor4.run(FORWARD);
  }else if(leftMotorSpeed - rightMotorSpeed >= 50 ){
    
    motor1.setSpeed(BASE_SPEED);
    motor2.setSpeed(BASE_SPEED);
    motor3.setSpeed(BASE_SPEED);
    motor4.setSpeed(BASE_SPEED);
    
    motor1.run(FORWARD); 
    motor2.run(BACKWARD);//SAĞ ÖN
    motor3.run(FORWARD); 
    motor4.run(BACKWARD);//SAĞ ARKA
  }else {

    motor1.run(FORWARD); 
    motor2.run(FORWARD);
    motor3.run(FORWARD); 
    motor4.run(FORWARD);
  }
*/
  

  


}


void qtr_calibrate() {

  Serial.println("QTR-8RC Kalibrasyon Başladı.");
  for (byte i = 0; i < 250; i++) {
  
    qtr.calibrate();

  }
  Serial.println("QTR-8RC Kalibrasyon Bitti.");

  Serial.println("\nQTR-8RC Calibration Min Values;");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.println("QTR-8RC Calibration Max Values;");
  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}



void bekle() {
  // Motorları durdurur.

  motor1.setSpeed(BASE_SPEED);
  motor2.setSpeed(BASE_SPEED);
  motor3.setSpeed(BASE_SPEED);
  motor4.setSpeed(BASE_SPEED);
 
  Serial.println("Dur");
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


