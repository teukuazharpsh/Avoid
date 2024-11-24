#include <AFMotor.h>  
#include <NewPing.h>
#include <Servo.h>

// Konstanta dan Definisi Pin
#define TRIG_PIN A5 
#define ECHO_PIN A3 
#define MAX_DISTANCE 200 // 20 cm
#define MAX_SPEED 170 // Kecepatan maksimum motor DC maks 255
#define MAX_SPEED_OFFSET 20
int zonaKiri = 0;
int zonaKanan = 0;
int zonaDepan = 0;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
int jarak[181];          // Array untuk menyimpan jarak (0-180 derajat)

AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);

Servo myservo;   

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

// Fungsi membaca jarak berdasarkan sudut servo 
int bacaUltrasonik(int sudut) {
    myservo.write(sudut);       // Putar servo ke sudut tertentu
    delay(500);               // Tunggu servo stabil
    int jarak = sonar.ping_cm(); // Konversi durasi ke jarak (cm)
    delay(500);
    myservo.write(115); 
    return jarak;
}
void loop() {
  //dek distance R&L 0cm
  int distanceR = 0;
  int distanceL = 0; 
  delay(40);
  if (distance <= 5) { // Jika jarak terlalu dekat, maka lakukan manuver menghindar
    moveStop();
    delay(100);
    moveBackwardWall();
    delay(300); // Menambahkan sedikit waktu mundur agar cukup untuk menghindar
    moveStop();
    int depan = sonar.ping_cm();
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
      // Iterasi untuk setiap derajat servo dari 0 hingga 180
      for (int derajat = 0; derajat <= 180; derajat++) {
          myservo.write(derajat);         // Gerakkan servo ke sudut tertentu
          delay(50);                    // Tunggu servo stabil
          jarak[derajat] = bacaUltrasonik(); // Baca jarak dan simpan ke array
      }
      // Analisis data jarak untuk setiap zona
      for (int i = 0; i <= 180; i++) {
          if (i >= 120 && i <= 180) {
              zonaKiri += jarak[i];  // Zona kiri: derajat 120-180
          } else if (i >= 60 && i < 120) {
              zonaDepan += jarak[i]; // Zona depan: derajat 60-120
          } else if (i >= 0 && i < 60) {
              zonaKanan += jarak[i]; // Zona kanan: derajat 0-60
          }
      }
      turnRight();
    } else {
      turnLeft();
    }
    moveStop(); 
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

// Fungsi untuk membaca jarak dari sensor ultrasonik
int bacaUltrasonik() {
    int jarakmaks = 150;
    int jarak = sonar.ping_cm();
    if (jarak > jarakmaks || jarak <= 0) { // Jika tidak ada objek atau di luar jangkauan
        jarak = jarakmaks;                // Tetapkan jarak maksimum
    }
    return jarak;
}
