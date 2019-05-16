#include<Wire.h>
#include <SdFat.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Pino A4 -> SDA do MPU
// Pino A5 -> SCL do MPU
//Endereco I2C do MPU6050
const int MPU = 0x68;

// Pino 4 -> TX do Gps
// Pino 3 -> RX do Gps
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

//Variaveis para armazenar valores do GPS e MPU
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int valor = 0;

// Pino 8 -> CS do SD
// Pino 11 -> MOSI do SD
// Pino 12 -> MISO do SD
// Pino 13 -> SCK do SD
SdFat sdCard;
SdFile dadosAcc;
SdFile dadosGPS;
const uint8_t chipSelect = 8;

int start_millis;

void lerGPS() {
  
    if (ss.available() > 0) {
      gps.encode(ss.read());
      if (gps.location.isUpdated()) {
        
        if (!dadosGPS.open("dados_gps.txt", O_RDWR | O_CREAT | O_AT_END)) {
          sdCard.errorHalt("Erro na abertura do arquivo DADOS_GPS.TXT!");
        }

        Serial.print("Time = "); Serial.print(gps.time.value()); // Raw time in HHMMSSCC format (u32)
        dadosGPS.print(gps.time.value());

        Serial.print("Milis = "); Serial.print(millis()); 
        dadosGPS.print(";"); dadosGPS.print(millis());
        
        Serial.print(" | Lat = "); Serial.print(gps.location.lat(), 6);
        dadosGPS.print(";"); dadosGPS.print(gps.location.lat(), 6);
      
        Serial.print(" | Lng = "); Serial.println(gps.location.lng(), 6);
        dadosGPS.print(";"); dadosGPS.println(gps.location.lng(), 6);

        dadosGPS.close();
      }
    }
  }

void lerMPU() {

  if (!dadosAcc.open("dados_acc.txt", O_RDWR | O_CREAT | O_AT_END))
  {
    sdCard.errorHalt("Erro na abertura do arquivo DADOS_ACC.TXT!");
  }
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU, 14, true);
    
  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Serial.print("Milis = "); Serial.print(millis()); 
  dadosAcc.print(millis());
  
  //Envia valor X do acelerometro para a serial
  Serial.print(" | AcX = "); Serial.print(AcX);
  dadosAcc.print(";"); dadosAcc.print(AcX);

  //Envia valor Y do acelerometro para a serial
  Serial.print(" | AcY = "); Serial.print(AcY);
  dadosAcc.print(";"); dadosAcc.print(AcY);

  //Envia valor Z do acelerometro para a serial
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  dadosAcc.print(";"); dadosAcc.print(AcZ);

  //Envia valor da temperatura para a serial
  //Calcula a temperatura em graus Celsius
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53);
  dadosAcc.print(";"); dadosAcc.print(Tmp/340.00+36.53);

  //Envia valor X do giroscopio para a serial
  Serial.print(" | GyX = "); Serial.println(GyX);
  dadosAcc.print(";"); dadosAcc.println(GyX);

  dadosAcc.close();
}

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);

    if(!sdCard.begin(chipSelect,SPI_HALF_SPEED))sdCard.initErrorHalt();

//  Inicializa o MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);

  int start_millis = millis();

}

void loop() {

  if (start_millis - millis() > 60000){
    start_millis = millis();
    lerGPS();
  }
  
  lerMPU();
  
  /*Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
  lerGPS();
  lerMPU();

  if (valor > 50) {
      Serial.print("fechou");
      dadosAcc.close();
      while (1){}
    }
  valor ++;*/
}