/*  10.060 - Web server

   This example sketch creates a web server that is running on the ESP32.

   Use a browser to connect to the ESP32 webserver. The landing page shows
   the temperature and humidity. You can click on the hyperlinks to control the
   state of the built-in LED (GPIO2)

   This sketch was written by Peter Dalmaris using information from the
   ESP32 datasheet and examples. You can find the original example notes below. I have
   extended the example to show the BME280 readings.


   Components
   ----------
    - ESP32 Dev Kit v4
    - BME280

    IDE
    ---
    Arduino IDE with ESP32 Arduino Code
    (https://github.com/espressif/arduino-esp32)


    Libraries
    ---------
    - WiFi
    - Adafruit_Sensor
    - Adafruit_BME280
    
   Connections
   -----------

   Connect the sensor breakout like this:

    ESP32   |    BME280
   -------------------
    3.3V   |    Vin
    GND    |    GND
    GPIO22 |    SCL
    GPIO21 |    SDA


    Other information
    -----------------

    1. ESP32 Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
    2. WiFi library: https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi
    3. Web server library: https://github.com/espressif/arduino-esp32/tree/master/libraries/WebServer
    4. Learn about HTTP: https://en.wikipedia.org/wiki/Hypertext_Transfer_Protocol

    Created on April 8 2019 by Peter Dalmaris

*/

/*  ORIGINAL EXAMPLE NOTES

  WiFi Web Server LED Blink

  A simple web server that lets you blink an LED via the web.
  This sketch will print the IP address of your WiFi Shield (once connected)
  to the Serial monitor. From there, you can open that address in a web browser
  to turn on and off the LED on pin 5.

  If the IP address of your shield is yourAddress:
  http://yourAddress/H turns the LED on
  http://yourAddress/L turns it off

  This example is written for a network using WPA encryption. For
  WEP or WPA, change the Wifi.begin() call accordingly.

  Circuit:
   WiFi shield attached
   LED attached to pin 5

  created for arduino 25 Nov 2012
  by Tom Igoe

  ported for sparkfun esp32
  31.01.2017 by Jan Hendrik Berlin

*/

/* Modify for DHT22 temperature and humidity sensor
 *  Forrest Lee Erickson
 *  Date: 20201012
 *   DHT22 Connections
 *  Date 20201015 Add I2CLCD Display 
   -----------

    Connect the sensor breakout like this:

    ESP32   |    DHT22
    -------------------
     3.3V   |    Pin 1
     GPIO32 |    Pin 2
     -      |    Pin 3
     GND    |    Pin 4     
    Connect a 10 K Ohm resistor between Pin 2 of the sensor and 3.3V on the ESP32 board.

   Connections
   -----------
    ESP32         |    I2C backpack
    -----------------------------
        GND       |      GND
        5V        |      5V
        GPIO42    |      SDA
        GPIO39    |      SCL
 * 
 */

//Setup WiFi interface and set server port number
#include <WiFi.h>
//const char* ssid     = "ardwifi";     // change this for your own network
//const char* password = "ardwifi987";  // change this for your own network

//const char* ssid     = "VRX";     // Lab wifi router Netgear WNR3500 SN:
//const char* password = "textinsert";  // Lab wifi router

const char* ssid     = "NETGEAR_11N";     // Netgear WAC104 SN: 4SL373BC00087
const char* password = "Heavybox201";  // Lab wifi router
//const char* password = "Heavybox202";  // bad pw.

const int DELAY_DHT22 = 2500;         //DHT22 sampling rate is 0.5HZ.
unsigned long lastReadDHT22 = 0;      // Variable to track reads on HDT22

WiFiServer server(80);

// For BME280 Temp, pressure and humidity
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

// For DHT22 Temp and humidity sensor
#include <SimpleDHT.h>
int pinDHT22 = 32;
SimpleDHT22 dht22(pinDHT22);

//For on board blue LED.
const int led_gpio = 2;

//For BME success
bool status;

#include <LiquidCrystal_I2C.h>
//LCD Display setup parameters.
const int numberLines = 4;
const int numberChar = 20;
const int addressLCD = 0x27;

LiquidCrystal_I2C lcd(addressLCD, numberChar, numberLines); // If this address is not working for your I2C backpack,
// run the address scanner sketch to determine the actual
// address.

void setup()
{
  //LCD splash screen.
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Web Temp Server");
  lcd.setCursor(0, 3);
  lcd.print("***C!AS***");
  
  pinMode(led_gpio, OUTPUT);      // set the LED pin mode
  
  //Serial splash message
//  Serial.begin(115200);
  Serial.begin(19200);
  delay(100);
  // We start by connecting to a WiFi network
  Serial.println("LED Temp Humidity Monitor");
  Serial.println("Caution! Amused Scientist");
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  delay(200);
  lcd.setCursor(0, 1);
  lcd.print("Connecting to AP    ");
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(0, 2);
  int cursorLocation = 0;
  while (WiFi.status() != WL_CONNECTED) {    
    delay(1000);
    Serial.print(".");    
    cursorLocation++;
    if (cursorLocation > numberChar){
      cursorLocation = 0;
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 2);
    }else {
      lcd.print(".");      
    }
  }//end with WiFi connected.
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  //Send IP address to LCD.
  lcd.setCursor(0, 3);
  lcd.print("IP: ");
  lcd.setCursor(4, 3);
  lcd.print(WiFi.localIP());

  server.begin();  
  
    status = bme.begin(0x76);   // BME280 sensors are usually set to address 0x76 or 0x77
  // If your BME280 sensor module has an SD0 pin, then:
  // SD0 unconnected configures the address to 0x77
  // SD0 to GND configures the address to 0x76
  // If there is no SD0 pin, try either address to find out which
  // one works with your sensor.
}//end setup

void loop() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Refresh: 5");
            client.println();
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");
            client.println("<head>");
            client.println("<title>LED Temp Humidity Monitor</title>");
//            client.println("<link rel=\"apple-touch-icon\" sizes=\"180x180\" href=\"/apple-touch-icon.png\">");
//            client.println("<link rel=\"icon\" type=\"image/png\" sizes=\"32x32\" href=\"/favicon-32x32.png\">");
//            client.println("<link rel=\"icon\" type=\"image/png\" sizes=\"16x16\" href=\"/favicon-16x16.png\">");
//            client.println("<link rel=\"manifest\" href=\"/site.webmanifest\">");

            client.println("</head>");

            // the content of the HTTP response follows the header:
            client.print("<p>Click <a href=\"/H\">here</a> to turn the LED on pin ");
            client.print(led_gpio);
            client.print(" on and display Farenheit.</p>");
            client.print("<p>Click <a href=\"/L\">here</a> to turn the LED on pin ");
            client.print(led_gpio);
            client.print(" off and display Celcius.</p>");

            if (status) {
            client.print("<hl>");
            client.print("<p>BME Temperature:");
            client.print(bme.readTemperature());
            client.print(" &deg;C</p>");
            client.print("<p>BME Humidity:");
            client.print(bme.readHumidity());
            }
            
            float temperature = 0;
            float humidity = 0;
            int err = SimpleDHTErrSuccess;
            if (millis()>DELAY_DHT22 + lastReadDHT22) {            
              lastReadDHT22 = millis();
              if ((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
                Serial.print("Read DHT22 failed, err="); Serial.println(err);delay(2000);
                //If LED on report in F, else C
                }
              if (digitalRead(led_gpio)){
                temperature =(temperature *9.0/5.0)+32.0;
                Serial.print("Temprature after conversion= "); Serial.println(temperature);                
              }
              //Update LCD Display
              lcd.setCursor(0, 1);
              lcd.print("Temperature: ");
              lcd.setCursor(13, 1);
              lcd.print(temperature);
              lcd.setCursor(0, 2);
              lcd.print("Humidity: ");
              lcd.setCursor(12, 2);
              lcd.print(humidity);
            }// time to read
            client.print("<hl>");
            client.print("<p>DHT22 Temperature:");
            client.print(temperature);
            if (digitalRead(led_gpio)){
                temperature =(temperature *9.0/5.0)+32.0;
                client.print(" &deg;F</p>");
              } else {
                client.print(" &deg;C</p>");
              }             
            client.print("<p>DHT22 Humidity:");
            client.print(humidity);
            client.println(" %</p>");
            client.print("<footer><hr><h5><p>&copy Forrest Erickson as Amused Scientist 2020.</h5></footer>");
            
            client.println("</html>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(led_gpio, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(led_gpio, LOW);                // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }// web client connected
}//loop
