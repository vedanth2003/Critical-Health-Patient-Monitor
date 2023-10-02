#include <SoftwareSerial.h>
SoftwareSerial gsm_Serial(8,9);
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <LiquidCrystal.h>
LiquidCrystal lcd(10,11,12,13,14,15);
int pb=2;
int buz=1;
#include "DHT.h"
#define DHTPIN 3
#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE);
#include <Wire.h>
int xval,yval,sval,gval,pbval,temperatureF;
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100]; //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100]; //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2,prvspo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate,prvhb; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
#include <TinyGPS.h>
TinyGPS gps;
float flat=0, flon=0;
void read_gps()
{
bool newData = false;
unsigned long chars;
unsigned short sentences, failed;
for (unsigned long start = millis(); millis() - start < 1000;)
{
while (gsm_Serial.available())
{
char c = gsm_Serial.read();
if (gps.encode(c))
newData = true;
}
}

if (newData)
{

unsigned long age;
gps.f_get_position(&flat,&flon,&age);

}
}

void setup()
{
Serial1.begin(115200); // initialize serial communication at 115200 bits per second:
lcd.begin(16,2);
gsm_Serial.println("AT");
delay(1500);
gsm_Serial.println("AT+CMGF=1");
gsm_Serial.begin(9600);
pinMode(A2,OUTPUT);
pinMode(A1,OUTPUT);
pinMode(pb,INPUT_PULLUP);
pinMode(buz,OUTPUT);
accel.begin();
dht.begin();
digitalWrite(A2,0);
digitalWrite(A1,0);
pinMode(0, INPUT); // Setup for leads off detection LO +
pinMode(3, INPUT); // Setup for leads off detection LO -
lcd.print("WELCOME");
Serial1.print("WELCOME:");
delay(1000);
// lcd.clear();
if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
{
//Serial.println(F("MAX30105 was not found. Please check wiring/power."));
while (1);
}


byte ledBrightness = 60; //Options: 0=Off to 255=50mA
byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
wifi_init();
}

void loop()
{
bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

//read the first 100 samples, and determine the signal range
for (byte i = 0 ; i < bufferLength ; i++)
{
while (particleSensor.available() == false) //do we have new data?
particleSensor.check(); //Check the sensor for new data

redBuffer[i] = particleSensor.getRed();
irBuffer[i] = particleSensor.getIR();
particleSensor.nextSample(); //We're finished with this sample so move to next sample
// Serial.print(F("red="));
// Serial.print(redBuffer[i], DEC);
// Serial.print(F(", ir="));
// Serial.println(irBuffer[i], DEC);
}
maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

while (1)
{
for (byte i = 25; i < 100; i++)
{
redBuffer[i - 25] = redBuffer[i];
irBuffer[i - 25] = irBuffer[i];
}

for (byte i = 75; i < 100; i++)
{
while (particleSensor.available() == false) //do we have new data?
particleSensor.check(); //Check the sensor for new data
redBuffer[i] = particleSensor.getRed();
irBuffer[i] = particleSensor.getIR();
particleSensor.nextSample();
if(validHeartRate == 1 && heartRate<160)
{
prvhb=heartRate;

}
if(validSPO2==1)
{
prvspo2=spo2;
}
int temperatureF = particleSensor.readTemperatureF();
sensors_event_t event;
accel.getEvent(&event);
int t = dht.readTemperature();
int h=dht.readHumidity();
int xval=event.acceleration.x;
int yval=event.acceleration.y;
int pbval=digitalRead(pb);
//Serial.println("S:" + String(ss));
Serial1.println("X:" + String(xval));
Serial1.println("Y:"+ String(yval));
Serial1.println("T:" + String(t));
Serial1.println("YH:"+ String(h));
Serial1.println("Panic:"+String(pbval));
if(particleSensor.getIR()>20000)
{
lcd.clear();
lcd.setCursor(0,0);
lcd.print("T:");
lcd.setCursor(2,0);
lcd.print(temperatureF);
lcd.setCursor(5,0);
lcd.print("H:");
lcd.setCursor(7,0);
lcd.print(prvhb);
lcd.setCursor(10,0);
lcd.print("s:");
lcd.setCursor(12,0);
lcd.print(prvspo2);
lcd.setCursor(0,1);
lcd.print("X:");
lcd.setCursor(2,1);
lcd.print(xval);
lcd.setCursor(5,1);
lcd.print("Y:");
lcd.setCursor(7,1);
lcd.print(yval);
lcd.setCursor(10,1);
lcd.print("T:");
lcd.setCursor(12,1);
lcd.print(t);
Serial1.print(temperatureF);
Serial1.print(prvspo2);
Serial1.print(prvhb);
delay(500);
upload(temperatureF,prvhb,prvspo2,sval,xval,yval);
if((digitalRead(10) == 1)||(digitalRead(11) == 1))
{
Serial.println('!');
delay(100);
}
else{
// send the value of analog input 0:
Serial.println(analogRead(28));
}
//Wait for a bit to keep serial data from saturating
if(temperatureF>100 || (prvhb>0 && prvhb>120) || (prvspo2>0 && prvspo2<70)|| (sval>300)||(gval>300))
{
send_sms(1);
digitalWrite(A2,1);
digitalWrite(A1,1);
}
else
digitalWrite(A2,0);
digitalWrite(A1,0);
}
else
{
lcd.clear();
lcd.print("T:NA H:NA");
lcd.setCursor(0,1);
lcd.print("S:NA" );
Serial1.println("T:NA H:NA S:NA");

}
}
maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}
if(sval>300||gval>300 || xval<-6 || xval>6 || yval>6 || yval<-6 || pbval==0)
{
digitalWrite(buz,1);
send_sms(1);
upload(temperatureF,prvhb,prvspo2,sval,xval,yval);
gsm_Serial.println("sending sms");
digitalWrite(buz,0);

}
}
void send_sms(int sts)
{
gsm_Serial.println("Sending SMS...");
gsm_Serial.println("AT");
delay(1000);
gsm_Serial.println("ATE0");
delay(1000);
gsm_Serial.println("AT+CMGF=1");
delay(1000);
gsm_Serial.print("AT+CMGS=\"9494051222\"\r\n");// Replace x with mobile number
delay(1000);
if(sts==1)
gsm_Serial.println("SOS: Abnoraml Soldier Conditon : ");
gsm_Serial.println("https://www.google.com/maps/search/?api=1&query=" + String(flat,6)+ "," + String(flon,6));
//gsm_Serial.println("https://www.google.com/maps/search/?api=1&query=" + String(16.2835)+ "," + String(81.1994));
delay(100);
gsm_Serial.println((char)26);// ASCII code of CTRL+Z
delay(6000);
gsm_Serial.println("AT");
delay(1000);
gsm_Serial.println("ATE0");
delay(1000);
gsm_Serial.println("AT+CMGF=1");
delay(1000);
gsm_Serial.print("AT+CMGS=\"9347232567\"\r\n");// Replace x with mobile number
delay(1000);
if(sts==1)
gsm_Serial.println("SOS: Abnormal health condition At ");
gsm_Serial.println("https://www.google.com/maps/search/?api=1&query=" + String(flat,6)+ "," + String(flon,6));
//gsm_Serial.println("https://www.google.com/maps/search/?api=1&query=" + String(16.2835)+ "," + String(81.1994));
delay(100);
gsm_Serial.println((char)26);// ASCII code of CTRL+Z
delay(2000);
}

void wifi_init()
{
Serial1.println("AT+RST");
delay(4000);
Serial1.println("AT+CWMODE=3");
delay(4000);
Serial1.print("AT+CWJAP=");
Serial1.write('"');
Serial1.print("CHPM"); // ssid/user name
Serial1.write('"');
Serial1.write(',');
Serial1.write('"');
Serial1.print("12345678"); //password
Serial1.write('"');
Serial1.println();
delay(1000);
}
void upload(int x, int y, int z,int p,int q,int r) //ldr copied int to - x and gas copied into -y
{

String cmd = "AT+CIPSTART=\"TCP\",\"";
cmd += "184.106.153.149"; // api.thingspeak.com
cmd += "\",80";
Serial1.println(cmd);
delay(1000);
String getStr ="GET /update?api_key=MAOX09U6EIPW3ZP9&field1=";
getStr += String(x);
getStr +="&field2=";
getStr += String(y);
getStr +="&field3=";
getStr += String(z);
getStr +="&field4=";
getStr += String(p);
getStr +="&field5=";
getStr += String(q);
getStr +="&field6=";
getStr += String(r);
getStr +="&field7=";
getStr += String(flat,6);
getStr +="&field8=";
getStr += String(flon,6);
getStr += "\r\n\r\n";
cmd = "AT+CIPSEND=";
cmd += String(getStr.length());
Serial1.println(cmd);
delay(1000);
Serial1.println(getStr);
}