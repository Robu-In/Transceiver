#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

Adafruit_SSD1306 disp1(-1);
Adafruit_SSD1306 disp2(-1);

Adafruit_MPU6050 mpu;

RF24 radio(11, 12); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

struct pack{
  int spot=0, x=0, y=0, jsw=1, trim=0;
};

struct drone{
  int batt;
};

pack data;
drone stat;

void setup() {
  Serial.begin(9600);
  //Wire.setClock(400000);

  pinMode(5, INPUT_PULLUP);   //SW 1 L
  pinMode(6, INPUT_PULLUP);   //SW 1 H
  pinMode(7, INPUT_PULLUP);   //SW 2 L
  pinMode(8, INPUT_PULLUP);   //SW 2 H

  pinMode(A0, INPUT);         //JY X
  pinMode(A1, INPUT);         //JY Y
  pinMode(4, INPUT_PULLUP);   //JY SW
  
  pinMode(A2, INPUT);         //POT  OUT
  pinMode(A5, INPUT);         //SPOT OUT

  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);

  if (!disp1.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {                   // Large Display Check & Initialize
    Serial.println("Failed to initialize Large Display");
    for(;;);
  }

  if (!disp2.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {                   // Small Display Check & Initialize
    Serial.println("Failed to initialize Small Display");
    for(;;);
  }

  if (! mpu.begin()) {                                              // MPU 6050 Check & Initialize
    Serial.println("Failed to initialize MPU6050 chip");
    for(;;);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  disp1.clearDisplay();                                           //Clear Initial Buffers
  disp2.clearDisplay();

  disp1.setTextSize(1);
  disp1.setTextColor(WHITE);
  disp1.setCursor(42, 0);
  disp1.println("Booting");
  disp1.display();
  disp2.display();
  
  delay(2000);

  if(analogRead(A5) >= 5){
    while(analogRead(A5) >= 2){
      disp1.clearDisplay(); 
      disp1.setTextColor(WHITE);
      disp1.setCursor(35, 0);
      disp1.println("! Warning !");
      disp1.setCursor(15, 20);
      disp1.println("Throttle not Zero");
      disp1.display();
      delay(50);
    }
  }

}

sensors_event_t a, g, temp;                                       // Gyro Variables
int flag = 0;                                                     // To zero out gyro                                                    
float x_cor= 0, y_cor= 0;                                         // Correction Values 

void loop(){

  if(digitalRead(5) == digitalRead(6)){
    calc_0();

    delay(25);
    radio.stopListening();
    radio.write(&data, sizeof(data));

    // delay(25);
    // radio.startListening();
    // if (radio.available()) {
    //   while (radio.available()){
    //     radio.read(&stat, sizeof(stat));
    //   }
    // }

    selector_screen();
    flag=0;

  }

  else if(digitalRead(5) == 0){
    calc_1();

    delay(25);
    radio.stopListening();
    radio.write(&data, sizeof(data));

    // delay(25);
    // radio.startListening();
    // if (radio.available()) {
    //   while (radio.available()){
    //     radio.read(&stat, sizeof(stat));
    //   }
    // }

    //serial_debug();
    screen_1();
    flag=0;
  }

  else if(digitalRead(6) == 0){
    while (flag == 0 && digitalRead(6) == 0){
      zero_out();
    }
    if(flag == 1){
      mpu.getEvent(&a, &g, &temp);
      calc_2();

      delay(25);
      radio.stopListening();
      radio.write(&data, sizeof(data));

      // delay(25);
      // radio.startListening();
      // if (radio.available()) {
      //   while (radio.available()){
      //     radio.read(&stat, sizeof(stat));
      //   }
      // } 


      serial_debug();
      screen_2();
    }
  }
}

void selector_screen(){
  disp1.clearDisplay();
  disp1.setTextSize(1);
  disp1.setTextColor(WHITE);
  disp1.setCursor(10, 0);
  disp1.println("Select Mode");
  disp1.setCursor(10, 10);
  disp1.println("Up for Manual");
  disp1.setCursor(10, 20);
  disp1.println("Down for Gyro");
  disp1.display();

  disp2.clearDisplay();
  disp2.setTextSize(2);
  disp2.setTextColor(WHITE);
  disp2.setCursor(5, 0);
  disp2.cp437(true); 
  disp2.write(24);
  disp2.display();
}

void screen_1(){
  disp1.clearDisplay();
  disp1.setTextSize(1);
  disp1.setTextColor(WHITE);
  disp1.setCursor(32, 0);
  disp1.println("Manual Mode");
  disp1.display();

  disp2.clearDisplay();
  disp2.display();
}

void screen_2(){
  disp1.clearDisplay();
  disp1.setTextSize(1);
  disp1.setTextColor(WHITE);
  disp1.setCursor(40, 0);
  disp1.println("Gyro Mode");
  disp1.display();

  disp2.clearDisplay();
  disp2.setTextSize(1);
  disp2.setTextColor(WHITE);

  disp2.setCursor(5, 0);
  disp2.println("X = ");
  disp2.setCursor(25, 0);
  disp2.println(a.acceleration.x + x_cor);

  disp2.setCursor(5, 10);
  disp2.println("Y = ");
  disp2.setCursor(25, 10);
  disp2.println(a.acceleration.y + y_cor);

  disp2.display();
}

void zero_out(){
  disp1.clearDisplay();
  disp1.setTextSize(1);
  disp1.setTextColor(WHITE);
  disp1.setCursor(40, 0);
  disp1.println("Gyro Mode");
  disp1.setCursor(5, 20);
  disp1.println("Press JSW when ready");
  disp1.display();

  mpu.getEvent(&a, &g, &temp);

  x_cor = 0 - a.acceleration.x;
  y_cor = 0 - a.acceleration.y;

  if(digitalRead(4) == 0){
    flag=1;
  }
}

void calc_0(){
  data.spot = 0;
  data.x    = analogRead(A0);
  data.y    = analogRead(A1);
  data.jsw  = digitalRead(4);
  data.trim = analogRead(A2);
}

void calc_1(){
  data.spot = analogRead(A5);
  data.x    = analogRead(A0);
  data.y    = analogRead(A1);
  data.jsw  = digitalRead(4);
  data.trim = analogRead(A2);
}

void calc_2(){
  data.spot = analogRead(A5);
  if(a.acceleration.x + x_cor >= -5 && a.acceleration.x + x_cor <= 5){
    data.y = map(a.acceleration.x + x_cor, -5, 5, 800, 200);
  }
  else if(a.acceleration.x + x_cor < -5){
    data.y = 800;
  }
  else if(a.acceleration.x + x_cor >  5){
    data.y = 200;
  }

  if(a.acceleration.y + y_cor >= -8   && a.acceleration.y + y_cor <= 8){
    data.x = map(a.acceleration.y + y_cor, -8, 8, 1023, 0);
  }
  else if(a.acceleration.y + y_cor < -8){
    data.x = 1023;
  }
  else if(a.acceleration.y + y_cor >  8){
    data.x = 0;
  }
  data.jsw  = digitalRead(4);
  data.trim = analogRead(A2);
}

void serial_debug(){
  Serial.print(data.spot);
  Serial.print("  ");
  Serial.print(data.x);
  Serial.print("  ");
  Serial.print(data.y);
  Serial.print("  ");
  Serial.print(data.jsw);
  Serial.print("  ");
  Serial.print(data.trim);
  Serial.print("  ");
  Serial.print(stat.batt);
  Serial.println("  ");
}




