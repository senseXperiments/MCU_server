

#include "SenseBoxMCU.h"
#include <Wire.h>
#include <senseBoxIO.h>
#include "Adafruit_MQTT.h"        // Adafruit.io MQTT library
#include "Adafruit_MQTT_Client.h" // Adafruit.io MQTT library

char serverip[] = "127.0.0.1";
char ssid[] = "SSID";
char password[] = "Password";


float accRange = 16.0/2048.0; // depends on range set


float accelX;
float accelY;
float accelZ;
float accelTot;


Bee* b = new Bee();

HDC1080 hdc;

// Adafruit MQTT

#define AIO_SERVER      serverip
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""
#define AIO_x        "x"
#define AIO_y        "y"
#define AIO_z        "z"
#define AIO_tot      "tot"


WiFiClient client;

// create the objects for Adafruit IO
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish X_feed = Adafruit_MQTT_Publish(&mqtt, "accelerometer/x");
Adafruit_MQTT_Publish Y_feed = Adafruit_MQTT_Publish(&mqtt, "accelerometer/y");
Adafruit_MQTT_Publish Z_feed = Adafruit_MQTT_Publish(&mqtt, "accelerometer/z");
Adafruit_MQTT_Publish Tot_feed = Adafruit_MQTT_Publish(&mqtt, "accelerometer/tot");



void read(byte addr, byte reg, byte *data, byte len)

{

  Wire1.beginTransmission(addr); // start transmission

  Wire1.write(reg);              // write register byte

  Wire1.endTransmission();       // stop transmission

  Wire1.requestFrom(addr, len);  // request x bytes

  while (Wire1.available() == 0); // wait for data bytes

  delay(1); // wait 1ms

  for (byte i = 0; i < len; i++)

  {

    *data++ = Wire1.read();      // read data byte

  }

}



void write(byte addr, byte data1, byte data2)

{

  Wire1.beginTransmission(addr); // start transmission

  Wire1.write(data1);            // write 1st data byte

  Wire1.write(data2);            // write 2nd data byte

  Wire1.endTransmission();       // stop transmission

}




void setup() {

/**
 *BEGIN ACCELEROMETER CODE
 */
  // init serial library

  Serial.begin(9600);

//  while (!Serial); // wait for serial monitor

  Serial.println("Test BMX055");



  // init I2C/Wire library

  Wire1.begin();



  // init BMX055

  write(I2C_MAGNET, 0x4B, 0x83); // PWR=0x81       soft-reset

  delay(20); // wait 20ms

  write(I2C_ACCEL, 0x0F, 0x0C);  // PMU_RANGE=0x0C +/-16g

  write(I2C_ACCEL, 0x10, 0x09);  // PMU_BW=0x09    15.63hz

  write(I2C_ACCEL, 0x11, 0x00);  // PMU_LPW=0x00   normal, sleep 0.5ms
  
  delay(500); // wait 500ms
  
/**
 * END ACCELEROMETER CODE
 */
  
  b->connectToWifi(ssid, password);
delay(1000);
  hdc.begin();
}

void loop() {


/**
 *BEGIN ACCELEROMETER CODE
 */

/*

  Test BMX055



  Test progam for Bosch BMX055, connected to Wire1 (I2C).

*/

  byte data[6];   // receive buffer

  int XA, YA, ZA; // accelerometer axis

  int XG, YG, ZG; // gyroscope axis

  int XM, YM, ZM; // magnetometer axis



  // read accelerometer data and convert them

  read(I2C_ACCEL, 0x02, data, 6);



  XA = (data[1] << 4) | ((data[0] & 0xF0) >> 4); // 12 bit

  if (XA > 2047) {

    XA -= 4096;

  }

  YA = (data[3] << 4) | ((data[2] & 0xF0) >> 4); // 12 bit

  if (YA > 2047) {

    YA -= 4096;

  }

  ZA = (data[5] << 4) | ((data[4] & 0xF0) >> 4); // 12 bit

  if (ZA > 2047) {

    ZA -= 4096;

  }


  accelX = (float)XA*accRange;

  accelY = (float)YA*accRange;

  accelZ = (float)ZA*accRange;

  accelTot = 9.81 * sqrt((sq(accelX)+sq(accelY)+sq(accelZ)));
  
  MQTT_connect();
        X_feed.publish(accelX);

        Y_feed.publish(accelY);

        Z_feed.publish(accelZ);
        
        Tot_feed.publish(accelTot);

   delay(100);

}

// define functions

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println(ret);
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }

  Serial.println("MQTT Connected!");
}
