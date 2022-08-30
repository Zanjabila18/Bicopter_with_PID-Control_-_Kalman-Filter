/*
Nama: Zanjabila Bahrusy Syuhada
NIM: 181354032
Program PID Control Bicopter (dengan Kalman Filter)
Rabu, 20 Januari 2021
Prodi D-IV Teknik Elektronika Politeknik Negeri Bandung
Tugas Akhir Matakuliah Sistem Kendali Digital, 
 */
// --------------------------------------------------------\\
//[1] Memasukan Library dan Header File
      #include <Wire.h>
      
//1a. Header file untuk sensor MPU6050
      #include "MPU6050.h"
      
//1b. Header file untuk komunikasi I2C MPU6050      
      #include "I2Cdev.h"

//1c. Header file untuk komunikasi I2C      
      #include "wire.h"

//1d. Header File untuk instruksi matematika
      #include "math.h"

//1e. Deklarasi LCD
      #include <LiquidCrystal.h>
      LiquidCrystal lcd(6, 7, 5, 4, 3, 2);

//1f. Deklarasi ESC
      #include <Servo.h>
      Servo esc_L;
      Servo esc_R;
            
//[2] Deklarasi variable
      #define RESTRICT_PITCH
      MPU6050 accelgyro;

//2a. Deklarasi variable matriks Q dan R
      float Qacc,Qgyro,R;

//2b. Deklarasi variable perhitungan prediksi nilai kalman filter 
//    dan error covariance
      float Xkacc,Xkpacc,Xkgyro,Pkp00,Pkp01,dXgyro;
      float Pkp[2][2],KG[2],y,Wk;

//2c. Deklarasi variable perubahan waktu
      unsigned long dtact;
      double dtlast,dt;

//2d. Deklarasi variable nilai gyroscope dan accelerometer
      int16_t ax,ay,az; // tipe data integer 16 bit
      int16_t gx,gy,gz;
      int16_t accX,accY,accZ;
      double gyroX,gyroXlast,gyroXangle;
      double kalmanX,PVroll,roll;
      int New_PVroll;

//2e. Deklarasi SV, PV, MV
      float SV, PV, PID;
      int MV, MV_L, MV_R; // Sinyal PWM ( 0 -255)
      int start;
      
//2f. Deklarasi Perhitungan PID-Control
      float Kp, e, Ti, Ki, Td, Kd, P, I, D;
      int potValue, throttle;
      float eint,eint_1;
      float eint_update;
      float edif;
      float et, et_1;

//2g. Deklarasi Varible pada Filter
      float PVf, PVf_1, wc, RC, a;   
         
//2h. Deklarasi Varible pada Time Sampling
      unsigned long t;
      double t_1, Ts;
      
//2i. Deklarasi untuk Plotting
      float interval_elapsed, interval_limit;

//[3] Fungsi objek/Subroutine perhitungan kalman filter
      float kalman_calculation(float Xtacc,float Xtgyro,float dt){
        dXgyro = Xtgyro - Xkgyro;
        Xkacc = Xkacc +(dt*dXgyro)+Wk;

        Pkp[0][0]=Pkp[0][0]+(dt*(dt*Pkp[1][1]-Pkp[0][1]-Pkp[1][0]+Qacc));
        Pkp[0][1]=Pkp[0][1]-(dt*Pkp[1][1]);
        Pkp[1][0]=Pkp[1][0]-(dt*Pkp[1][1]);
        Pkp[1][1]=Pkp[1][1]+(dt*Qgyro);

        KG[0]=Pkp[0][0]/(Pkp[0][0]+R);
        KG[1]=Pkp[1][0]/(Pkp[0][0]+R);

        Xkacc=Xkacc+(KG[0]*(Xtacc-Xkacc));
        Xkgyro=Xkgyro+(KG[1]*(Xtacc-Xkacc));

        Pkp00=Pkp[0][0];
        Pkp01=Pkp[0][1];

        Pkp[0][0]=Pkp[0][0]-(KG[0]*Pkp00);
        Pkp[0][1]=Pkp[0][1]-(KG[0]*Pkp01);
        Pkp[1][0]=Pkp[1][0]-(KG[1]*Pkp00);
        Pkp[1][1]=Pkp[1][1]-(KG[1]*Pkp01);

        return Xkacc;
      }

//[4] Fungsi data pengiriman I2C ke mikro master(utama)
      void requestEvent(){
        Wire.write(New_PVroll);
      }

//[5] Fungsi set angle
      void setAngleX(float roll2){
        Xkacc=roll2;  
      }
      
//[6] Fungsi set kalman gyroscope
      float kalman_gyro(){
        return dXgyro;
      }
      
void setup() {
//[7] Inisialisasi nilai awal variable
//7a. Nilai awal variable perubahan waktu
      dt = 0;
      dtlast = 0;

//7b. Nilai parameter matriks Q dan R
      Qacc = 0.001;
      Qgyro = 0.003;
      R = 0.3;

//7c. Nilai awal predisksi kalman filter dan dan error covariance
      Xkacc = 0.0;
      Xkpacc = 0.0;
      Xkgyro = 0.0;
      Pkp[0][0] = 0.0;
      Pkp[0][1] = 0.0;
      Pkp[1][0] = 0.0;
      Pkp[1][1] = 0.0;

//7d. Memulai komunikasi I2C (terutama membuka jalur regiter 4) 
      Wire.begin();
      Wire.begin(4);
      Wire.onRequest(requestEvent);

//7e. Memulai komunikasi serial
      Serial.begin(9600);

//7f. Inisialisasi komunikasi I2C
      Serial.print("Initializing I2C devices...");
      accelgyro.initialize();

//7g. Uji koneksi
      Serial.print("Testing device connections...");
      Serial.print(accelgyro.testConnection()?"MPU6050 connection succesful":"MPU6050 connection failed");

//7h. Memulai Komunikasi dengan LCD
      lcd.begin(16,2);
      lcd.setCursor(0,0);
      lcd.print("SV:");
      lcd.setCursor(0,1);
      lcd.print("PV:");
    
//7i. Memulai ESC
      esc_L.attach(10);
      esc_R.attach(9);

      esc_R.writeMicroseconds(1000);    // Send the signal to the ESC
      esc_L.writeMicroseconds(1000);    // Send the signal to the ESC
      delay(3000);
      esc_R.writeMicroseconds(1000);    // Send the signal to the ESC
      esc_L.writeMicroseconds(1000);    // Send the signal to the ESC
      delay(2000);

//7j. Set nilai dan parameter
      Kp = 0.7;//0.516;//0.474;//0.546;//1.1;//1.2;
      Ti = 1.2;//1.1466;//1.4;//0.753958;//1.300695;
      Td = 0.5;//0.28665;//0.4;//0.1884895;// 0.32517375;
      
      Ts = 0.02;
      throttle=1075;
      
      interval_elapsed=0;
      interval_limit=0.01;
      t=0; 

      // Agar menghindari nilai infinity 'NAND', ketika Ti = 0 */
      if (Ti==0){
        Ki=0;
      }
      else{
        Ki=Kp/Ti;
      }
      Kd=Kp*Td;
      et_1=0;
      eint_1=0; //nilai awal luasan error   
}

//[8] Main program:
void loop() {
//8a. Menghitung waktu setiap satu kali looping
      dtlast = dtact;
      dtact = millis();
      dt = dtact;
      dt = (dt - dtlast)/1000;

//8b. Mengambil data sensor MPU6050
      accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

//8c. Memindahkan data ke variable lain (**agar data sebelumnya(murni) tetap terjaga)         
      accX = ax;
      accY = ay;
      accZ = az;

//8d. Kalibrasi sensor MPU6050 (dalam derajat)
      roll = (atan2(accY,accZ)*180)/3.14;
      gyroX = gx;
      gyroX = gyroX/100.0;

//8e. Set kalman filter pada saat lebih kecil dari -90 dan lebih besar dari 90 derajat   
      if ((roll<-90 && kalmanX>90) || (roll>90 && kalmanX<-90)){
        setAngleX(roll);
        kalmanX=roll;
        gyroXangle=roll;        
      }

//8f. Hitung kalman filter pada saat -90 d.d 90 derajat      
      else{
        kalmanX = kalman_calculation(roll,gyroX,dt);
      }

//8g. Baca nilai sudut selanjutnya
      gyroXangle = gyroXangle + (kalman_gyro()*dt);

//[9] Bila pembacaan lebih kecil dari -180 dan lebih besar dari 180 derajat
      if (gyroXangle<-180 || gyroXangle>180){
        gyroXangle = kalmanX;
      }
//[10] Pengondisian nilai output kalman filter
      PVroll = kalmanX;
      PVroll = PVroll+50;
      New_PVroll = PVroll;

//[11] PID Control
      //SV = 0;
      potValue = analogRead(A0); 
      if(potValue>186){
        SV = map(potValue, 187, 1020, 0, 32);  
        }
      if(potValue<186){
        SV = map(potValue, 185, 10, 0, -32);
        }
       
      if(SV<-32){
        SV=-32;
        }
        else if(SV>32){
          SV=32;
          }
          else{
            SV=SV;
            }

//11a. Baca nilai PV
      PV = kalmanX;

//11b. Hitung error
      et = PV - SV;

//11c. Hitung Proporsional
      P = Kp*et;

//11d. Hitung Integral
      eint_update=((et+et_1)*Ts)/2;
      eint=eint_1+eint_update;
      I=Ki*eint;

//11e. Hitung Differensial
      edif=(et-et_1)/Ts;
      D=Kd*edif;

//11f. Hitung Keluaran MV
      MV = P+I+D;
      MV_L = throttle-MV;
      MV_R = throttle+MV;

//11e. Membatasi Output MV
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

//11f. Menuliskan MV
      esc_R.writeMicroseconds(MV_R); //Kanan
      esc_L.writeMicroseconds(MV_L); //Kiri

//11g. Menghitung time sampling
//      t_1=t;
//      t=millis();
//      Ts=(t-t_1)/1000;
      
//11h. Cek Penjumlahan Ts
      interval_elapsed=interval_elapsed+Ts;
       if(interval_elapsed >= interval_limit){
        Serial.print(SV);
        Serial.print(" ");
        Serial.print(90);
        Serial.print(" ");
        Serial.print(PV);
        Serial.print(" "); 
        Serial.println(-90);
        
        lcd.setCursor(4,0); 
        lcd.print(SV);
        lcd.setCursor(9,0); 
        lcd.print((char)223);
        lcd.setCursor(3,1); 
        lcd.print(PV);
        lcd.setCursor(9,1); 
        lcd.print((char)223);
        }
        else{
          interval_elapsed=interval_elapsed;
          }
      et_1=et;
      eint_1=eint;  
    }
