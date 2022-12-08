/*
Nama: Zanjabila Bahrusy Syuhada
NIM: 181354032
Program Desain ZN2 - Control Bicopter
Selasa, 19 Januari 2021
Prodi D-IV Teknik Elektronika Politeknik Negeri Bandung
Tugas Akhir Matakuliah Sistem Kendali Digital, 
 */
// --------------------------------------------------------\\
//[1] Memasukan Library dan Header File
      #include <Wire.h>
      
//1a. Header file untuk sensor MPU6050
      #include <MPU6050_tockn.h>
      MPU6050 mpu6050(Wire);

//1b. Deklarasi LCD
      #include <LiquidCrystal.h>
      LiquidCrystal lcd(6, 7, 5, 4, 3, 2);

//1c. Deklarasi ESC
      #include <Servo.h>
      Servo esc_L;
      Servo esc_R;
//[2]
//2a. Deklarasi SV, PV, MV
      float SV, PV, PID;
      int MV, MV_L, MV_R; // Sinyal PWM ( 0 -255)
      int start;
      
//2b. Deklarasi Perhitungan P-Control
      float Kp, e, P;
      double potValue;
      float eint,eint_1;
      float eint_update;
      float edif;
      float et, et_1;  
         
//2c. Deklarasi Varible pada Time Sampling
      unsigned long t;
      double t_1, Ts;
      
//2d. Deklarasi untuk Plotting
      float interval_elapsed, interval_limit;


void setup() {
//[3] Inisialisasi nilai awal variable
//3a. Memulai komunikasi I2C (terutama membuka jalur regiter 4) 
      Wire.begin();

//3b. Memulai komunikasi serial
      Serial.begin(9600);
      
//3c. Memulai komunikasi MPU6050
      mpu6050.begin();
      mpu6050.calcGyroOffsets(true);
      
//3d. Memulai Komunikasi dengan LCD
      lcd.begin(16,2);
      lcd.setCursor(0,0);
      lcd.print("Kp:");
      lcd.setCursor(0,1);
      lcd.print("MV:");

//3e. Memulai ESC
      esc_L.attach(10);
      esc_R.attach(9);

      esc_R.writeMicroseconds(1000);    // Send the signal to the ESC
      esc_L.writeMicroseconds(1000);    // Send the signal to the ESC
      delay(3000);
      esc_R.writeMicroseconds(1000);    // Send the signal to the ESC
      esc_L.writeMicroseconds(1000);    // Send the signal to the ESC
      delay(2000);

//3f. Set nilai dan parameter
      //Kp = 1.1;//1.2;
      interval_elapsed=0;
      interval_limit=0.01;
      t=0; 
      
}

//[4] Main program:
void loop() {
//[5] Ziegler - nichols 2
      SV = 0;
      
//5a. Baca nilai Gain
      Kp=analogRead(0)*0.001955;
      //Kp=map(potValue,0,1023,0,2);

//5b. Baca nilai PV
      mpu6050.update();
      PV = mpu6050.getAngleX();

//5c. Hitung error
      e = PV - SV;

//5d. Hitung Gain P
      P = Kp*e;
      MV = P;
      MV_L = 1075-MV;
      MV_R = 1075+MV;

//5e. Membatasi Output MV
      //Kanan
      if(MV_R>1120){  //max 2000
        MV_R=1120;
      }
      else if(MV_R<1000){ //min 1000
        MV_R=1000;
      }
      
      //Kiri
      if(MV_L>1120){  //max 2000
        MV_L=1120;
      }
      else if(MV_L<1000){ //min 1000
        MV_L=1000;
      }

//5f. Menuliskan MV
      esc_R.writeMicroseconds(MV_R); //Kanan
      esc_L.writeMicroseconds(MV_L); //Kiri

//5g. Menghitung time sampling
      t_1=t;
      t=millis();
      Ts=(t-t_1)/1000;
      
//5h. Cek Penjumlahan Ts
      interval_elapsed=interval_elapsed+Ts;
       if(interval_elapsed >= interval_limit){
        Serial.print(SV);
        Serial.print(" ");
        Serial.print(90);
        Serial.print(" ");
        Serial.print(PV);
        Serial.print(" "); 
        Serial.println(-90);
        
        lcd.setCursor(5,0); 
        lcd.print(Kp);
        lcd.setCursor(5,1); 
        lcd.print(MV);
        }
        else{
          interval_elapsed=interval_elapsed;
          }
    }
