#include <AFMotor.h>  
#include <NewPing.h>
#include <Servo.h>

// Konstanta dan Definisi Pin
#define TRIG_PIN A5 
#define ECHO_PIN A4 
#define MAX_DISTANCE 200 // Maksimum jarak sensor (cm)
#define MAX_SPEED 170 // Kecepatan maksimum motor DC
#define BASE_SPEED 100 // Kecepatan dasar motor
#define TURN_SPEED 120 // Kecepatan saat berbelok
#define WALL_DISTANCE 20 // Jarak target dari dinding (cm)

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 

AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);

Servo myservo;

// Variabel PID
float Kp = 1.5;  // Gain Proportional
float Ki = 0.0;  // Gain Integral
float Kd = 0.5;  // Gain Derivative

float previousError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  myservo.attach(10);
  myservo.write(90); // Posisikan servo ke tengah
  delay(1000);
}

void loop() {
  int frontDistance = readPing(); // Baca jarak di depan

  // Jika jarak depan cukup jauh, terus maju dengan kontrol PID
  if (frontDistance > WALL_DISTANCE) {
    followWallPID();
  } else {
    // Jika ada rintangan di depan, cari jalan dengan memutar
    avoidObstacle();
  }

  delay(50); // Interval pengulangan
}

// Fungsi untuk membaca jarak menggunakan sensor ultrasonik
int readPing() { 
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = MAX_DISTANCE; // Jika tidak terdeteksi, set jarak maksimum
  Serial.print("Distance: ");
  Serial.println(cm);
  return cm;
}

// Fungsi untuk menghitung output PID
float computePID(int target, int current) {
  float error = target - current;          // Selisih antara target dan kondisi saat ini
  integral += error;                       // Akumulasi error untuk komponen integral
  float derivative = error - previousError; // Perubahan error (derivative)
  previousError = error;                   // Simpan error untuk iterasi berikutnya

  // Hitung nilai PID
  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

// Fungsi untuk mengikuti dinding menggunakan PID
void followWallPID() {
  int sideDistance = readPingSide(); // Jarak ke dinding samping
  float pidOutput = computePID(WALL_DISTANCE, sideDistance);

  // Hitung kecepatan motor berdasarkan PID
  int motorSpeed1 = BASE_SPEED + pidOutput;
  int motorSpeed2 = BASE_SPEED - pidOutput;

  // Pastikan kecepatan dalam batas
  motorSpeed1 = constrain(motorSpeed1, 0, MAX_SPEED);
  motorSpeed2 = constrain(motorSpeed2, 0, MAX_SPEED);

  // Set kecepatan motor
  motor1.setSpeed(motorSpeed1);
  motor2.setSpeed(motorSpeed2);

  motor1.run(FORWARD);
  motor2.run(FORWARD);

  Serial.print("Follow Wall - Motor Speeds: ");
  Serial.print("Motor1 = ");
  Serial.print(motorSpeed1);
  Serial.print(", Motor2 = ");
  Serial.println(motorSpeed2);
}

// Fungsi untuk membaca jarak dari sisi (kanan)
int readPingSide() {
  myservo.write(45); // Arahkan servo ke kanan
  delay(500);
  int distance = readPing();
  myservo.write(90); // Kembalikan servo ke depan
  return distance;
}

// Fungsi untuk menghindari rintangan di depan
void avoidObstacle() {
  moveStop();       // Hentikan robot
  delay(200);

  int distanceRight = lookRight(); // Lihat ke kanan
  int distanceLeft = lookLeft();   // Lihat ke kiri

  if (distanceRight > distanceLeft) {
    turnRight(); // Jika kanan lebih bebas, belok kanan
  } else {
    turnLeft();  // Jika kiri lebih bebas, belok kiri
  }
}

// Fungsi untuk melihat ke kanan
int lookRight() {
  myservo.write(45); 
  delay(500);
  int distance = readPing();
  myservo.write(90);
  return distance;
}

// Fungsi untuk melihat ke kiri
int lookLeft() {
  myservo.write(135); 
  delay(500);
  int distance = readPing();
  myservo.write(90);
  return distance;
}

// Fungsi untuk berhenti
void moveStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}

// Fungsi untuk berbelok ke kanan
void turnRight() {
  motor1.setSpeed(TURN_SPEED);
  motor2.setSpeed(TURN_SPEED);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  delay(500); // Sesuaikan waktu belok
  moveStop();
}

// Fungsi untuk berbelok ke kiri
void turnLeft() {
  motor1.setSpeed(TURN_SPEED);
  motor2.setSpeed(TURN_SPEED);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  delay(500); // Sesuaikan waktu belok
  moveStop();
}
