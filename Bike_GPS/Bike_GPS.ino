#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "SSD1306.h"
#include <FastLED.h>
#include <FastLED.h>

//LEDS test
#define DATA_PIN 15
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS 24
CRGB leds[NUM_LEDS];
#define BRIGHTNESS 96
#define FRAMES_PER_SECOND 120

//GPS
static const int RXPin = 23, TXPin = 22;
static const uint32_t GPSBaud = 9600;
SSD1306 display(0x3c, 5, 4);  //GPIO 5 = D1, GPIO 4 = D2
#define flipDisplay false
String Satelites;
int Nsatelites;
String SatelitesSTR;
String Latitud;
String Longitud;
String LatHome;
String LonHome;
int Cont1 = 0;
double LatHomeDuble;
double LongHomeDouble;
bool HomeSet;
double DistanciaKm;
String DistanciaKmString;

// The TinyGPSPlus object
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {

  //inciar leds
  delay(3000);
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  //iniciar puerto serial y gps

  Serial.begin(115200);
  delay(10);
  ss.begin(GPSBaud);
  delay(10);

  display.init();
  if (flipDisplay) display.flipScreenVertically();

  /* Mostramos la pantalla de bienvenida */

  Serial.println();
  Serial.println();
  Serial.print("HOLA YAYI");
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "HOLA YAYI");
  display.display();
  delay(500);
  display.clear();
}

void loop() {

  Nsatelites = gps.satellites.value();
  Longitud = gps.location.lat();
  Latitud = gps.location.lng();
  DistanciaKmString = DistanciaKm;
  Satelites = "Satelites :" + Nsatelites;
  SatelitesSTR = Nsatelites;

  display.drawString(0, 0, "Satelites: " + SatelitesSTR + "  Ciclos: " + Cont1);
  display.drawString(0, 12, "Long: " + Latitud + "- Lat: " + Longitud);
  display.drawString(0, 24, "m to Home: " + DistanciaKmString);
  display.drawString(0, 45, "Home: " + LonHome + " , " + LatHome);
  display.display();


  Serial.println("");
  Serial.println("************************************************************************");
  Serial.println("");
  Serial.print("Numero de Satelites: ");
  Serial.println(gps.satellites.value());  // Number of satellites in use (u32)
  Serial.print("Latitud: ");
  Serial.println(gps.location.lat(), 6);  // Latitude in degrees (double)
  Serial.print("Longitud: ");
  Serial.println(gps.location.lng(), 6);  // Longitude in degrees (double)
  Serial.println("");
  Serial.print("Fecha: ");
  Serial.print(gps.date.year());  // Year (2000+) (u16)
  Serial.print("/");
  Serial.print(gps.date.month());  // Month (1-12) (u8)
  Serial.print("/");
  Serial.print(gps.date.day());  // Day (1-31) (u8)
  Serial.println("");
  Serial.print("Hora: ");
  Serial.print(gps.time.hour() - 5);  // Hour (0-23) (u8)
  Serial.print(":");
  Serial.print(gps.time.minute());  // Minute (0-59) (u8)
  Serial.print(":");
  Serial.print(gps.time.second());  // Second (0-59) (u8)
  Serial.println("");
  Serial.print("Velocidad Metros por Segundo: ");
  Serial.println(gps.speed.mps());  // Speed in meters per second (double)
  Serial.print("Velocidad Kilometros por Hora: ");
  Serial.println(gps.speed.kmph());  // Speed in kilometers per hour (double)
  Serial.print("Rumbo en grados: ");
  Serial.println(gps.course.deg());  // Course in degrees (double)
  Serial.print("Altitud en metros: ");
  Serial.println(gps.altitude.meters());  // Altitude in meters (double)
  Serial.print("Precision HDOP: ");
  Serial.println(gps.hdop.value());  // Horizontal Dim. of Precision (100ths-i32)
  Serial.println();
  Serial.println(gps.charsProcessed());
  Serial.println(gps.sentencesWithFix());
  Serial.println(gps.failedChecksum());
  Serial.println();
  Serial.println(HomeSet);
  Serial.println(Nsatelites);
  Serial.println(Cont1);

  smartDelay(200);

  if (millis() > 6000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
  }


  //Guarda el punto de home despues de hacer 10 ciclos y asegurar que tenga mas de 5 satelites
  Cont1 = Cont1 + 1;

  if (Cont1 >= 20 && Nsatelites >= 5 && HomeSet == false) {
    delay(20);
    LatHome = gps.location.lat();
    LonHome = gps.location.lng();
    LongHomeDouble = gps.location.lng();
    LatHomeDuble = gps.location.lat();
    HomeSet = true;
  }
  if (HomeSet == true) {
    DistanciaKm = gps.distanceBetween(gps.location.lat(), gps.location.lng(), LatHomeDuble, LongHomeDouble) / 1;

    FastLED.clear();
    for (int i = 0; i < 24; i++) {
      fadeToBlackBy(leds, NUM_LEDS, 90);
      leds[i] += CHSV(96, 255, 192);
      FastLED.delay(3000 / FRAMES_PER_SECOND);
      FastLED.show();
    }
    for (int i = 24; i > 0; i--) {
      fadeToBlackBy(leds, NUM_LEDS, 90);
      leds[i] += CHSV(96, 255, 192);
      FastLED.delay(3000 / FRAMES_PER_SECOND);
      FastLED.show();
    }
    FastLED.clear();

  } else {

    FastLED.clear();
    for (int i = 0; i < 24; i++) {
      fadeToBlackBy(leds, NUM_LEDS, 90);
      leds[i] += CHSV(0, 255, 192);
      FastLED.delay(3000 / FRAMES_PER_SECOND);
      FastLED.show();
    }
  }
  display.clear();
}

// esperar a que el gps tenga senal
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
