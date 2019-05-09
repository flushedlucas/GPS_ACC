#include <SoftwareSerial.h> //biblioteca para emular o TX e o RX
#include <TinyGPS.h> //biblioteca do GPS
 
SoftwareSerial serial1(1,0); // RX, TX
TinyGPS gps1; //Criando o GPS
 
void setup() {
 serial1.begin(9600); //valeocidade do GPS, não é possivel modificar
 Serial.begin(9600); // Velocidade do serial
 
 
Serial.println("Aguardando a concecção com os satelites");
}
void loop() {
 bool recebido = false; // indicará quando o GPS conseguiu se conectar aos satélites
 
while (serial1.available()) { //realiza diversas leituras até conseguir se conectar
 char cIn = serial1.read();
 recebido = gps1.encode(cIn);
 }
 
// quando conseguir se conectar aos satélites ele irá realizar as leituras
 if (recebido) {
 Serial.println("----------------------------------------");
 
 //Latitude e Longitude
 long latitude, longitude;
 unsigned long idadeInfo;
 gps1.get_position(&latitude, &longitude, &idadeInfo); // obtem a latitude, longitude e a idade da informação
 
// se a latitude for algo valido ela será impressa
 if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
 Serial.print("Latitude: ");
 Serial.println(float(latitude) / 100000, 6);
 }
 
// se a longitude for algo valido ela será impressa
 if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
 Serial.print("Longitude: ");
 Serial.println(float(longitude) / 100000, 6);
 }
 
 // se a data da informação for algo valido ela será impressa
 if (idadeInfo != TinyGPS::GPS_INVALID_AGE) {
 Serial.print("Idade da Informacao (ms): ");
 Serial.println(idadeInfo);
 }
 
 
 //Obtem o Dia e Hora no GMT, ou seja, há 3 horas de diferença com o horário do Brasil
 int ano;
 byte mes, dia, hora, minuto, segundo, centesimo;
 gps1.crack_datetime(&ano, &mes, &dia, &hora, &minuto, &segundo, &centesimo, &idadeInfo); //obtendo a data, horário e a idade da informação
 
 // imprimindo os dados
 Serial.print("Data (GMT): ");
 Serial.print(dia);
 Serial.print("/");
 Serial.print(mes);
 Serial.print("/");
 Serial.println(ano);
 
Serial.print("Horario (GMT): ");
 Serial.print(hora);
 Serial.print(":");
 Serial.print(minuto);
 Serial.print(":");
 Serial.print(segundo);
 Serial.print(":");
 Serial.println(centesimo);
 
 
 //Obtendo a altitude
 float altitudeGPS;
 altitudeGPS = gps1.f_altitude();
 
// se a altitude for algo valido e diferente de 1000000Cm ela será impressa
 if ((altitudeGPS != TinyGPS::GPS_INVALID_ALTITUDE) && (altitudeGPS != 1000000)) {
 Serial.print("Altitude (cm): ");
 Serial.println(altitudeGPS);
 }
 
 
 //obtem a velocidade de locomoção do gps em km/h
 float velocidade;
 velocidade = gps1.f_speed_kmph();
 
Serial.print("Velocidade (km/h): ");
 Serial.println(velocidade, 2); //Conversão de Nós para Km/h
 
//obtem o sentito do movimento em centesima de graus
 unsigned long sentido;
 sentido = gps1.course();
 
Serial.print("Sentido (grau): ");
 Serial.println(float(sentido) / 100, 2);
 
 
 //obtem a quantidade de satelites e a precisão
 unsigned short satelites;
 unsigned long precisao;
 satelites = gps1.satellites();
 precisao = gps1.hdop();
 
if (satelites != TinyGPS::GPS_INVALID_SATELLITES) {
 Serial.print("Satelites: ");
 Serial.println(satelites);
 }
 
if (precisao != TinyGPS::GPS_INVALID_HDOP) {
 Serial.print("Precisao (centesimos de segundo): ");
 Serial.println(precisao);
 }
 }
}
