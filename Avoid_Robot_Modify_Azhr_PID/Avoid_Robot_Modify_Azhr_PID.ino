#include <AFMotor.h>  
#include <NewPing.h>
#include <Servo.h>

// Konstanta dan Definisi Pin
#define TRIG_PIN A5 
#define ECHO_PIN A3 
#define MAX_DISTANCE 200 // 20 cm
#define MAX_SPEED 170 // Kecepatan maksimum motor DC maks 255
#define MAX_SPEED_OFFSET 20

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 

AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);

Servo myservo;   

// PID Constants
float Kp = 2.0, Ki = 0.5, Kd = 1.0;
float previousError = 0, integral = 0;

// Variabel target
const int targetDistance = 5; // Jarak target (cm)
const int lowerBound = 3;     // Batas bawah jarak (cm)
const int upperBound = 5;     // Batas atas jarak (cm)
const int stopThreshold = 10; // Jarak berhenti awal (cm)
const int maxSpeed = 190;     // Kecepatan maksimum motor
const int minSpeed = 50;      // Kecepatan minimum motor

unsigned long startTime = millis(); // Catat waktu awal
const unsigned long timeout = 5000; // Waktu maksimum (5 detik)

int LWall = 30; // 30cm
int LRB = 25; // 25cm
boolean goesForward = false;
int distance = 100;
int speedSet = 0;
int dw = LWall - LRB; // menghitung sisa dari panjang dari wall dengan panjang chasis robot
void setup() {
  Serial.begin(9600); // Inisialisasi komunikasi serial untuk debugging
  myservo.attach(10);  
  myservo.write(115); // cari titik tengah dari servo ke arah depan untuk ultrasonik
  delay(2000);
  for(int i = 0; i<=5; i++){ // membaca jarak 5 kali
    distance = sonar.ping_cm();
    delay(75);
  }
}

void loop() {
  //dek distance R&L 0cm
  int distanceR = 0;
  int distanceL = 0; 
  delay(40);
  if (distance <= stopThreshold) { // Jika jarak terlalu dekat, maka lakukan manuver menghindar
    moveStop();
    delay(100);
    moveBackwardWall();
    delay(300); // Menambahkan sedikit waktu mundur agar cukup untuk menghindar
    moveStop();
    int depan = sonar.ping_cm();
    // Mulai kendali PID
    while (true) {
      // Jika jarak sudah stabil di 5 cm, motor berhenti
      if (depan == targetDistance) {
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        Serial.println("Jarak stabil di 5 cm. Motor berhenti.");
        break;
      }

      // Kendali PID untuk menjaga jarak 3–5 cm
      float error = targetDistance - depan;
      integral += error;
      float derivative = error - previousError;
      float output = Kp * error + Ki * integral + Kd * derivative;

      // Batasi output ke kecepatan yang diizinkan
      int speed = constrain(abs(output), minSpeed, maxSpeed);

      // Tentukan arah motor berdasarkan error
      if (distance < lowerBound) {
        // Terlalu dekat, motor mundur
        motor1.setSpeed(speed);
        motor2.setSpeed(speed);
        motor1.run(BACKWARD);
        motor2.run(BACKWARD);
      } else if (distance > upperBound) {
        // Terlalu jauh, motor maju
        motor1.setSpeed(speed);
        motor2.setSpeed(speed);
        motor1.run(FORWARD);
        motor2.run(FORWARD);
      }

      if (millis() - startTime > timeout) {
        Serial.println("Timeout! Keluar dari loop.");
        motorLeft.setSpeed(0);
        motorRight.setSpeed(0);
        break;
      }
    }
    /*
    if(depan < dw){
      //maju sedikit untuk mendapatkan posisi yg bagus guna menghasilkan belokan yang pas ke kanan atau kiri bebas
      moveForward();
      if(depan >= dw || depan <= dw ){
        moveStop();
      }
    }else{
      moveStop();
    }
    delay(200);
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);
    if (distanceR >= distanceL) {
      turnRight();
    } else {
      turnLeft();
    }
    moveStop(); */
  }
  else {
    moveForwardLong(); // Jika tidak ada rintangan, terus maju full speed
  }
  distance = readPing(); // Update jarak terbaru
}

// Fungsi untuk melihat jarak ke kanan
int lookRight() {
  myservo.write(50); 
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115); 
  return distance;
}

// Fungsi untuk melihat jarak ke kiri
int lookLeft() {
  myservo.write(170); 
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115); 
  return distance;
}

// Fungsi untuk membaca jarak menggunakan sensor sonar
int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    Serial.println("Ultrasonik error");
    cm = 250;
  }
  Serial.print("Distance: "); // Menambahkan debugging output
  Serial.println(cm);
  return cm;
}

// Fungsi untuk menghentikan gerakan
void moveStop() {
  motor1.run(RELEASE); 
  motor2.run(RELEASE);
} 

// Fungsi untuk bergerak maju
void moveForward() {
  if (!goesForward) {
    goesForward = true;
    motor1.run(FORWARD);      
    motor2.run(FORWARD);
    
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet);
      delay(5);
    }
  }
}
void moveForwardLong() {
  if (!goesForward) {
    goesForward = true;
    motor1.run(FORWARD);      
    motor2.run(FORWARD);
    
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 50) {
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet);
      delay(100);
    }
  }
}
// Fungsi untuk bergerak mundur
void moveBackward() {
  goesForward = false;
  motor1.run(BACKWARD);      
  motor2.run(BACKWARD);

  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    delay(5);
  }
}  
//dibuat dua karena mundur nya ada yg cepat untuk rem
void moveBackwardWall() {
  goesForward = false;
  motor1.run(BACKWARD);      
  motor2.run(BACKWARD);

  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 50) {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    delay(100);
  }
}  

// Fungsi untuk berbelok ke kanan
void turnRight() {
  motor1.run(BACKWARD); // apakah mundur perlu di set kecepat juga atau diam saja?
  motor2.run(FORWARD);
  int derajat = 50; // ibaratnya derajat sebab kecepat mempengaruhi terhadap hasil nilai derjat yg dihasilkan.
  for (speedSet = 0; speedSet < derajat; speedSet += 1) {
    motor2.setSpeed(speedSet);
    delay(100);
  } 
  delay(450);
  //belum tentu harus maju langsung cari kenapa harus maju dulu apa ada yg perlu dilakukan 
  // apakah posisi sudah lurus ke kanan atau tidak 
  int kanan = lookRight();
  int kiri = lookLeft();
  motor1.run(FORWARD);      
  motor2.run(FORWARD);
} 
 
// Fungsi untuk berbelok ke kiri
void turnLeft() {
  motor1.run(FORWARD);     
  motor2.run(BACKWARD); // apakah mundur perlu di set kecepat juga atau diam saja?
  int derajat = 50; // ibaratnya derajat sebab kecepat mempengaruhi terhadap hasil nilai derjat yg dihasilkan.  
  for (speedSet = 0; speedSet < derajat; speedSet += 1) {
    motor1.setSpeed(speedSet);
    delay(100);
  } 
  delay(450); // cek dulu kalau perlu menggunakna ultra sonik sisi kanan kiri dengan ketentuan 
  //bahwa nilai kanan dan kiri ada space atau kanan balance dengan kiri lalu dibaca 
  //belum tentu harus maju langsung cari kenapa harus maju dulu apa ada yg perlu dilakukan 
  // apakah posisi sudah lurus ke kanan atau tidak 
  motor1.run(FORWARD);     
  motor2.run(FORWARD);
}  
