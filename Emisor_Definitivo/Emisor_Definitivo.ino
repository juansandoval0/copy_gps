#include <SoftwareSerial.h>
#include "EBYTE.h"
#include <TinyGPSPlus.h>
#include <Wire.h>
#include "SSD1306.h"

//GPS
static const int RXPin2 = 13, TXPin2 = 15;
static const uint32_t GPSBaud = 9600;
String Satelites;
int Nsatelites;
String SatelitesSTR;
String Latitud;
String Longitud;
String LatHome;
String LonHome;
int Cont1 = 0;
double Longitud_H;
;
double Latitud_H;
bool HomeSet;
bool GPS_Lock;
float Top_Speed_1;
double DistanciaKm;
String DistanciaKmString;
int i;
int Chan;


// The TinyGPSPlus object
TinyGPSPlus gps;
SoftwareSerial ESerial2(RXPin2, TXPin2);

#define PIN_RX 14  //D5 on the board (Connect this to the EBYTE TX pin)
#define PIN_TX 12  //D6 on the board (connect this to the EBYTE RX pin)
#define PIN_M0 5   //D1 on the board (connect this to the EBYTE M0 pin)
#define PIN_M1 4   //D2 on the board (connect this to the EBYTE M1 pin)
#define PIN_AX 16  //D0 on the board (connect this to the EBYTE AUX pin)


// Estrucutra de los datos que se envian
struct DATA {
  unsigned long Contador_Paquetes;
  float Longitud_Actual_1;
  float Latitud_Actual_1;
  String Nombre_1;
  float Longitud_Home_1;
  float Latitud_Home_1;
  float Velocidad_1;
  String Estado;
  bool Lock;
};

DATA MyData;


// Inicia el transmisor Lora
SoftwareSerial ESerial1(PIN_RX, PIN_TX);

// Crea el objeto Trasceiver Lora
EBYTE Transceiver(&ESerial1, PIN_M0, PIN_M1, PIN_AX);

void setup() {

  Serial.begin(115200);  //inicia el puerto serial del controlador
  delay(10);
  ESerial1.begin(9600);  //inicia el lora
  delay(10);
  ESerial2.begin(GPSBaud);  //inicia el gps
  delay(100);


  Serial.println("Starting Sender");
  delay(10);
  Serial.println("Starting Sender");
  delay(10);
  Serial.println("Starting Sender");
  delay(10);

  // this init will set the pinModes for you (aun no se como funciona bien esto)
  Serial.println(Transceiver.init());

  // Configuracion del Lora

  Serial.println(Transceiver.GetAirDataRate());
  Serial.println(Transceiver.GetChannel());
  Transceiver.SetAddressH(1);
  Transceiver.SetAddressL(1);
  Chan = 15;
  Transceiver.SetChannel(Chan);
  //save the parameters to the unit,
  Transceiver.SaveParameters(PERMANENT);


  // for both sender and receiver and make sure air rates, channel
  // and address is the same
  Transceiver.PrintParameters();
  delay(5000);
}






void loop() {

  // GPS---------------------------------------------------------------------------

  Nsatelites = gps.satellites.value();
  Longitud = gps.location.lat();
  Latitud = gps.location.lng();
  DistanciaKmString = DistanciaKm;
  Satelites = "Satelites :" + Nsatelites;
  SatelitesSTR = Nsatelites;
  Cont1 = Cont1 + 1;



  delay(150);

  //si tiene satelites imprimir datos

  if (Nsatelites > 0) {

    Serial.println("");
    Serial.println("************************************************************************");
    Serial.println("");
    Serial.print("Numero de Satelites: ");
    Serial.println(gps.satellites.value());  // Number of satellites in use (u32)
    Serial.print("Hora: ");
    Serial.print(gps.time.hour() - 5);  // Hour (0-23) (u8)
    Serial.print(":");
    Serial.print(gps.time.minute());  // Minute (0-59) (u8)
    Serial.print(":");
    Serial.print(gps.time.second());  // Second (0-59) (u8)
    Serial.println("");
    Serial.println(gps.location.lat(), 6);  // Latitude in degrees (double)
    Serial.print("Longitud: ");
    Serial.println(gps.location.lng(), 6);  // Longitude in degrees (double)
    Serial.println();
    Serial.println();
    Serial.println(gps.charsProcessed());
    Serial.println(gps.sentencesWithFix());
    Serial.println(gps.failedChecksum());
    Serial.println();
    Serial.println(HomeSet);
    Serial.println(Nsatelites);
    Serial.println(Cont1);
  }

  smartDelay(200);
  delay(500);

  //cuando tiene mas de 3 satelites, tiene el gps asegurado
  if (Nsatelites > 3) {
    GPS_Lock = true;                  //avisa que ya anclo el gps
    Longitud_H = gps.location.lng();  //guarda la longitud del home
    Latitud_H = gps.location.lat();   //guarda la Latitudo del home
    DistanciaKm = gps.distanceBetween(gps.location.lat(), gps.location.lng(), Latitud_H, Longitud_H) / 1;
  }

  //enviar datos si tiene gps asegurado

  if (GPS_Lock = true) {

    Serial.print("Enviando Datos");

    MyData.Contador_Paquetes++;
    MyData.Longitud_Actual_1 = gps.location.lng();
    MyData.Latitud_Actual_1 = gps.location.lat();
    MyData.Nombre_1 = "J.C.";
    MyData.Longitud_Home_1 = Longitud_H;
    MyData.Latitud_Home_1 = Latitud_H;
    MyData.Velocidad_1 = gps.speed.kmph();
    MyData.Estado = "Lock";
    MyData.Lock = "True";


    //Velocidad mas alta

    if (gps.speed.kmph() > Top_Speed_1) {

      Top_Speed_1= gps.speed.kmph();
    }
  } else {

    Serial.println("Esperando GPS");
    Serial.println(MyData.Contador_Paquetes);
    MyData.Estado = "Lock";
    MyData.Lock = false;
  }


  Transceiver.SendStruct(&MyData, sizeof(MyData));

  // note, you only really need this library to program these EBYTE units
  // you can call write directly on the EBYTE Serial object
  // ESerial.write((uint8_t*) &Data, PacketSize );


  delay(3000);

  if (millis() > 6000 && gps.charsProcessed() < 10) {
    Serial.println(F("Error en el GPS, revisar cableado"));
  }
}


// esperar a que el gps tenga senal
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ESerial2.available())
      gps.encode(ESerial2.read());
  } while (millis() - start < ms);
}
