/*  10.061_-_Web_server_Temp_LCD

   Modify for DHT22 temperature and humidity sensor and button by Forrest Lee Erickson
   
   This example sketch creates a web server that is running on the ESP32.
   It measures temprature and humidity with DHT22
   It displays the temperature and humidity on an I2C 20x4 LCD display.
   The served web page allows the user to light the blue LED on the ESP32 Dev kit v4
   The served web page lights an LED with the temprature is in F
   A physical button changes the dsiplay from C to F and back.
   
************************************************************************************
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

    ESP32   |    Button
   -------------------
    GPIO25 |    pin 1
    GND    |    pin 2
  
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

/* Modify for DHT22 temperature and humidity sensor and button
 *  Forrest Lee Erickson
 *  Date: 20201012
 *   DHT22 Connections
 *  Date 20201015 Add I2CLCD Display 
 *  Date 20201019 Add button to change temprature from C to F
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

// Button with interupt service
const byte interruptPin = 25;
volatile int interruptCounter = 0;
int numberOfInterrupts = 0;
bool led_state = false;  // Keep track of the state of the LED

// Debouncing parameters
long debouncing_time = 1000; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;
 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
   if((long)(micros() - last_micros) >= debouncing_time * 1000) {
    interruptCounter++;
  }
  last_micros = micros();
  portEXIT_CRITICAL_ISR(&mux);
}// end IRAM_ATTR handleInterrupt


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

  pinMode(interruptPin, INPUT_PULLUP);  // Button. Using an extarnal pull up instead of internal
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING);
  
  //Serial splash message
  Serial.begin(115200);
//  Serial.begin(19200);
  delay(100);
  // We connect to a WiFi network
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
// Button
  if(interruptCounter>0){
 
      portENTER_CRITICAL(&mux);
      interruptCounter--;
      portEXIT_CRITICAL(&mux);

      led_state = !led_state;

      digitalWrite(led_gpio, led_state);   // turn the LED on (HIGH is the voltage level)
      
      numberOfInterrupts++;
      Serial.print("An interrupt has occurred. Total:");
      Serial.println(numberOfInterrupts);
  }
  
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
            client.println("Refresh: 5; URL = '/'"); //Redirect with out H or L
            client.println();
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");
            client.println("<head>");
            client.println("<title>LED Temp Humidity Monitor</title>");
//            client.println("<link rel=\"apple-touch-icon\" sizes=\"180x180\" href=\"/apple-touch-icon.png\">");
//            client.println("<link rel=\"icon\" type=\"image/png\" sizes=\"32x32\" href=\"/favicon-32x32.png\">");
//            client.println("<link rel=\"icon\" type=\"image/png\" sizes=\"16x16\" href=\"/favicon-16x16.png\">");

//            client.println("<link rel=\"icon\" type=\"image/ico\" sizes=\"16x16\" href=\"data:image/ico;base64,
            client.println("<link rel=\"icon\" img src=\"data:image/ico; base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAB+UlEQVQ4T62T30uTYRSAn2/RqkVTt4nbCL2oZVg4cMqWoGxzICPsB3wiCCripTcyhF1MEi9ChemVeuVfIAglSrMm4UC6MCejIldim4whzbQZ1DZlX/hBky5tvXeH97zPcw7vOQK0PwF8gJLznRzwVID27D88/qPKnQKk84n/zi4eoFI9lkKh16ysVOD11sp4n+8jfX1fWFvT0tVlLSidzq9MTkaoqUmTSKjo6WlAUKsfSen0MwYHa5mYqJaTd3aWODlRYDBk0OkekM1eoLLyJ1tbAaambpLPC7S1JbHZnGcAUbzH/Px1zObvbGwEaW1tIhgM4XY3EQjoEcUEc3NvsFpbWF/XoFBIMqhQgcXiIhwuY3j4Aw5HCrvdTjy+xOKigf7+OvT6DJHIS3S6LMvLesbHb7O6Wn4G0GofcnCgJBx+RSp1SbZ2d8fRaHJUVd2XWzMaf+HxfKK3N0ZJyTHNzQ4EhUKUToPDQyUm0w+i0QChUDn5PKjVJ1gshzQ0tLC5WUZFRYZk8opcxfb2C0ZG7iDU1bmksbF3dHTYGBj4TGfnLiaTWzaWlh6zv/8cv/8WR0cX5fvp6Rty/15vlPp6F4Io2qTR0fdyMDv7lmj0GkNDdwtfFwiE2Nu7jN9fzcxMmMbGb8RiV/F4zCwsGCl+kP7HKBe9TEWt829OgMUkQtJXXQAAAABJRU5ErkJggg==\">");
//            client.println("<link rel=\"icon\" content=\"data:image/ico;base64,

//            client.println("<link rel=\"manifest\" href=\"/site.webmanifest\">");
//            client.println("<link rel=\"icon\" type=\"image/ico\" sizes=\"16x16\" img src=\"data:image/ico;
//            client.println("<link rel=\"icon\" type=\"image/ico\" sizes=\"16x16\" href=\"data:image/ico;

            client.println("</head>");
            // the content of the HTTP response follows the header in the body:
            client.println("<body>");
            client.println("<h1>LED Temp Humidity Monitor</H1>");
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
            client.println("<p>");           
            //favicon
            client.println("<img src=\"data:image/ico;base64,AAABAAMAEBAAAAEAIABoBAAANgAAACAgAAABACAAKBEAAJ4EAAAwMAAAAQAgAGgmAADGFQAAKAAAABAAAAAgAAAAAQAgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAFUAAH5VAAD5VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA+VUAAH5VAAD5VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD5VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP84VFT/DNvb/1APD/8cq6v/KYSE/1UAAP9VAAD/VQAA/xe4uP8foKD/Riws/xTCwv8B/Pz/DNzc/ztOTv9VAAD/RDIy/xe6uv9PERH/KoGB/wzb2/9VAAD/VQAA/1AQEP8B/Pz/J4iI/yx6ev8H6ur/Kn9//ymBgf8C+Pj/Riws/1QCAv9PERH/VQAA/0glJf8A/v7/FcDA/xW+vv8RzMz/AP///0M0NP9UAwP/TBoa/1UAAP9OFBT/BfDw/zBxcf9QDg7/AP///08SEv9VAAD/EM7O/xe5uf8vcXH/EM/P/wrh4f9VAAD/VQAA/0waGv8mjIz/Bu3t/wH8/P9COjr/UA4O/wD///9EMjL/VQAA/yp/f/8VwMD/VQAA/wrh4f8jlZX/VQAA/00YGP8H6en/AP39/xi2tv9APz//VQAA/1AODv8A////OVNT/1UAAP9FLy//Afz8/z5DQ/8A////PUhI/1UAAP85VFT/AP7+/0M1Nf9VAAD/VAIC/1UAAP9QDg7/AP///y5zc/9VAAD/VQAA/wre3v8M2tr/BPPz/1IHB/9VAAD/Sx4e/wP29v8nior/KIiI/w7V1f9QDQ3/UQoK/xDMzP8niYn/VQAA/1UAAP8se3v/C97e/x6jo/9VAAD/VQAA/1UAAP8/QkL/ENDQ/wH6+v8L3t7/QD09/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA+VUAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA+VUAAH5VAAD5VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA+VUAAH4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAKAAAACAAAABAAAAAAQAgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAJUAAB8VQAA5lUAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA5lQAAHwAAAACVAAAfFUAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1QAAHxVAADmVQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA5lUAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1QBAf9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/MG1t/wve3v8lj4//VQAA/1UAAP8se3v/Haam/xe5uf9RCwv/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/MWtr/xa9vf8jlpb/Nlxc/1UAAP9VAAD/O01N/xa8vP8E8/P/AP7+/wrh4f8lkJD/TRcX/1UAAP9VAAD/VQAA/1UAAP8I5eX/AP///wD///9APT3/VQAA/yaNjf8A////AP///ztMTP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP8Xurr/AP///wD///8kkJD/UgoK/x2np/8A////AP///wD///8A////AP///wD///8H6ur/Qjg4/1UAAP9VAAD/VQAA/xi2tv8A////AP///z9DQ/9VAAD/RDEx/wD///8A////I5SU/1UAAP9VAAD/VQAA/1UAAP9VAAD/UwYG/wP09P8A////AP///0A8PP88Skr/AP///wD///8A////AP7+/wzb2/8N1tb/AP39/wD///8G7e3/Sxsb/1UAAP9VAAD/TxIS/yt9ff8xbGz/VAEB/1UAAP9VAAD/DtTU/wD///8M29v/VQAA/1UAAP9VAAD/VQAA/1UAAP9BOzv/AP///wD///8I5eX/VAIC/1UAAP8foKD/AP///xutrf9IJCT/VQAA/1UAAP9EMzP/A/X1/wD///8jlJT/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP8teHj/AP///wD///8ohob/K319/yt9ff8rfX3/K319/xm0tP8A////AP///ySSkv9VAAD/VQAA/1ENDf8yaGj/VQAA/1UAAP9VAAD/VQAA/1UAAP8Sx8f/AP///wzb2/9VAAD/VQAA/1IICP9DNTX/UBAQ/1UAAP9VAAD/VQAA/0sdHf8A/f3/AP///wD///8A////AP///wD///8A////AP///wD///8A////QD4+/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/OlFR/wH7+/8A////COjo/1UAAP9VAAD/Sx0d/wD///8A////ThUV/1UAAP9VAAD/VQAA/xPGxv8A////Afv7/wnj4/8J4+P/CePj/wfo6P8A////AP///wfo6P9UAgL/VQAA/1UAAP9VAAD/VQAA/1UAAP9UBAT/O05O/xe5uf8A////AP///wD///8Yt7f/VQAA/1UAAP9LHR3/AP///wD///9DNTX/VQAA/1UAAP9VAAD/LXZ2/wD///8C+Pj/UA4O/1UAAP9VAAD/OFVV/wD///8A////IJ6e/1UAAP9VAAD/VQAA/1UAAP9UAgL/Mmho/wrg4P8A////AP///wD///8A////BPPz/0UxMf9VAAD/VQAA/0sdHf8A////AP///zhVVf9VAAD/VQAA/1UAAP9IJyf/AP///wD///83WVn/VQAA/1UAAP8goKD/AP///wD///86UVH/VQAA/1UAAP9VAAD/VAIC/x2oqP8A////AP///wD///8A////AP///xHKyv9EMzP/VQAA/1UAAP9VAAD/Sx0d/wD///8A////LnV1/1UAAP9VAAD/VQAA/1UAAP8N1tb/AP///xyqqv9VAAD/VAIC/wfo6P8A////Avf3/1EMDP9VAAD/VQAA/1UAAP81X1//AP///wD///8A////Avf3/x+hof9BOzv/VQAA/1UAAP9VAAD/VQAA/1UAAP9LHR3/AP///wD///8jlpb/VQAA/1UAAP9VAAD/VQAA/yiGhv8A////BPPz/1IICP9DNjb/AP///wD///8Ytrb/VQAA/1UAAP9VAAD/VQAA/x2np/8A////AP///xmysv9MGhr/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/0sdHf8A////AP///xi2tv9VAAD/VQAA/1UAAP9VAAD/Qjc3/wD///8A////O01N/yqBgf8A////AP///zFpaf9VAAD/VQAA/1UAAP9VAAD/Haio/wD///8B+vr/UgkJ/1UAAP9VAAD/VQAA/1UAAP9SCAj/VQAA/1UAAP9VAAD/Sx0d/wD///8A////DdbW/1UAAP9VAAD/VQAA/1UAAP9UAQH/COXl/wD///8goKD/EczM/wD///8A/v7/Sx0d/1UAAP9VAAD/VQAA/1UAAP8wbW3/AP///wD///88SEj/VQAA/1UBAf9EMzP/IJ6e/xe6uv9VAAD/VQAA/1UAAP9LHR3/AP///wD///8D9fX/VQEB/1UAAP9VAAD/VQAA/1UAAP8il5f/AP///wD///8A////AP///xDPz/9VAAD/VQAA/1UAAP9VAAD/VQAA/1ELC/8L29v/AP///wD+/v8K4eH/Buzs/wD///8A////AP///0M0NP9VAAD/VQAA/0sdHf8A////AP///wD///9NGBj/VQAA/1UAAP9VAAD/VQAA/z1HR/8A////AP///wD///8A////KYKC/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/0kkJP8M2tr/AP///wD///8A////AP///wD///8A////HKqq/1UAAP9VAAD/UQ0N/yiFhf8ar6//DNra/0M1Nf9VAAD/VQAA/1UAAP9VAAD/UwYG/yCgoP8Zs7P/E8bG/w3Y2P9EMzP/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1ELC/8td3f/EczM/wP09P8D9fX/DNra/x+hof88SUn/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAADmVQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA5lQAAHxVAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9UAAB8AAAAAlQAAHxVAADmVQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAADmVAAAfAAAAAIAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAKAAAADAAAABgAAAAAQAgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABVAAAPVgAAXFUAAMZVAAD2VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD2VQAAxlYAAFxbAAAOAAAAAFUAAA9UAACVVQAA91UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1QAAPhUAACVWwAADlYAAFxVAAD3VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9UAAD4VgAAXFUAAMZVAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAAxlUAAPZVAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA9lUAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9UAgL/VQEB/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9SCAj/M2Vl/xPHx/8asLD/Qjc3/1UAAP9VAAD/VQAA/0M1Nf82Wlr/L3Bw/yiHh/88TEz/VQEB/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/0ogIP8epaX/KIeH/zZbW/9DNTX/TRgY/1UAAP9VAAD/VQAA/08SEv88S0v/HqWl/wni4v8B/Pz/Afz8/wzc3P8jlZX/Qjc3/1ENDf9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP89Rkb/Buzs/wD///8A////CuLi/0gmJv9VAAD/VQAA/yWOjv8A////AP///wD///8dqKj/UA4O/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/zdXV/8A////AP///wD///8B/Pz/K319/1UAAP9TBQX/PEpK/xm0tP8D9vb/AP///wD///8A////AP///wD///8A////BPPz/yCfn/9FLS3/VQEB/1UAAP9VAAD/VQAA/1UAAP8yaWn/Avf3/wD///8A////AP///yx5ef9VAAD/VQAA/0M1Nf8D9fX/AP///wD///8N2Nj/Sx4e/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/yCenv8A////AP///wD///8C+fn/Qjg4/1MFBf8wbm7/B+jo/wD///8A////AP///wD///8A////AP///wD///8A////AP///wH9/f8QzMz/SCYm/1UAAP9VAAD/VQAA/1UAAP87TU3/Bu3t/wD///8A////AP///yt/f/9VAAD/VQAA/04WFv8VwMD/AP///wD///8B/Pz/QT09/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VAMD/wri4v8A////AP///wD///8Rysr/TRkZ/0ogIP8L3t7/AP///wD///8A////AP///wD///8B+vr/BfHx/wXv7/8C+fn/AP///wD///8A////Ec3N/08REf9VAAD/VQAA/1UAAP9QDQ3/J4mJ/wXx8f8A/v7/DNra/0YuLv9VAAD/VQAA/1MEBP8pgoL/Afz8/wD///8A////J4mJ/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/Ri0t/wD///8A////AP///wD+/v8kkpL/UgcH/1MHB/8rfHz/A/f3/wD///8A////Avj4/xi0tP8td3f/OFVV/zpRUf8uc3P/EcrK/wD///8A////AP///ySPj/9UAgL/VQAA/1UAAP9VAAD/UgoK/0cqKv9BOjr/SiAg/1UBAf9VAAD/VQAA/1UAAP8+RUX/B+vr/wD///8A////DdfX/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/LnR0/wD///8A////AP///wTy8v83Wlr/VQAA/1UAAP9PEhL/FcDA/wD///8L3t7/OlBQ/08SEv9UAgL/VQAA/1UAAP9UAgL/Sx4e/wnj4/8A////AP///wvf3/9IJyf/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9QDQ3/EM/P/wD///8A////AP///z9CQv9HKir/Ryoq/0cqKv9HKir/Ryoq/0cqKv9HKir/Fb+//wD///8A////AP///wve3v9JIiL/VQAA/1UAAP9VAAD/SSMj/xTDw/9DNjb/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/x+goP8A////AP///wXx8f84VVX/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/LHp6/wD///8A////AP///wD9/f8A/f3/AP39/wD9/f8A/f3/AP39/wD9/f8A/f3/AP///wD///8A////AP///xyrq/9UAgL/VQAA/1UAAP9VAAD/VQAA/08REf9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/UQ0N/xHMzP8A////AP///wL39/8yaWn/VQAA/1UAAP9VAAD/TRcX/zRiYv9COTn/TxAQ/1UAAP9VAAD/VQAA/1UAAP9VAAD/TRkZ/wH6+v8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///zpRUf9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9MHBz/HKqq/wH9/f8A////AP///wP29v8zZmb/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP7+/0wbG/9VAAD/VQAA/1UAAP9VAAD/VQAA/xi1tf8A////AP///wD///8C+fn/A/b2/wP29v8D9vb/A/b2/wP29v8B/Pz/AP///wD///8A////Buzs/1IHB/9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9RCgr/SiAg/zFqav8L3t7/AP///wD///8A////AP///wbs7P89R0f/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///0A+Pv9VAAD/VQAA/1UAAP9VAAD/VQAA/zVeXv8A/v7/AP///wH7+/8dpqb/M2Rk/zNjY/8zY2P/M2Nj/y9wcP8N19f/AP///wD///8A////IJ2d/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/UA0N/zpRUf8hmpr/C97e/wD///8A////AP///wD///8A////AP///xe6uv9QEBD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///zRiYv9VAAD/VQAA/1UAAP9VAAD/VQAA/0gmJv8I5+f/AP///wD+/v8jlJT/UggI/1UAAP9VAAD/VQAA/0ohIf8K4eH/AP///wD///8A/v7/PUlJ/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UBAf9APDz/FMTE/wXv7/8A/v7/AP///wD///8A////AP///wD///8A////C9/f/0kkJP9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///yiFhf9VAAD/VQAA/1UAAP9VAAD/VQAA/08REf8Zs7P/AP///wD///8Ry8v/TRkZ/1UAAP9VAAD/VQAA/ztMTP8A////AP///wD///8L3d3/SiAg/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQEB/zxMTP8H6Oj/AP///wD///8A////AP///wD///8A////AP///wT09P8Vvr7/SCYm/1UAAP9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///xypqf9VAAD/VQAA/1UAAP9VAAD/VQAA/1QDA/8rfn7/Afz8/wD///8D9vb/QTo6/1UAAP9VAAD/VQAA/yCenv8A////AP///wD///8cqqr/UA8P/1UAAP9VAAD/VQAA/1UAAP9VAAD/QTo6/wfo6P8A////AP///wD///8A////AP///wD+/v8H6Oj/Hamp/zVfX/9NFhb/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///xHLy/9VAQH/VQAA/1UAAP9VAAD/VQAA/1UAAP88SUn/Bu3t/wD///8A////KIeH/1UAAP9VAAD/UwUF/wbs7P8A////AP///wH7+/8td3f/VQEB/1UAAP9VAAD/VQAA/1UAAP9UAwP/F7q6/wD///8A////AP///wD///8A////B+jo/yp/f/9GLS3/UA4O/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///wza2v9PERH/VQAA/1UAAP9VAAD/VQAA/1UAAP9NFhb/DdjY/wD///8A////C93d/1MFBf9VAAD/PkVF/wD///8A////AP///wfr6/8+RET/VQAA/1UAAP9VAAD/VQAA/1UAAP9KISH/Ct/f/wD///8A////AP///wXu7v8mjY3/TxER/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///wrh4f9IJib/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/IJ2d/wD///8A////AP///0E9Pf9VAAD/IpiY/wD///8A////AP///w7V1f9OExP/VQAA/1UAAP9VAAD/VQAA/1UAAP9FMDD/CeTk/wD///8A////AP///zNmZv9TBQX/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///wfo6P9BPDz/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/PUZG/wD///8A////AP///yOYmP9PERH/DtXV/wD///8A////AP///yGamv9VAQH/VQAA/1UAAP9VAAD/VQAA/1UAAP9JJCT/CuDg/wD///8A////A/Pz/1MFBf9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9QDQ3/UwQE/1UAAP9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///wXv7/86UVH/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VAQE/wjo6P8A////AP///w3Z2f85U1P/B+rq/wD///8A////AP///z1FRf9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9TBgb/E8fH/wD///8A////AP///0E6Ov9UAwP/VQAA/1UAAP9TBQX/ThYW/z5ERP8TxcX/MG5u/1UAAP9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///wP39/8zZ2f/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/yOVlf8A////AP///wTz8/8Ry8v/Avr6/wD///8A////CObm/1IJCf9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/N1pa/wD9/f8A////AP///wza2v8rfX3/Nl1d/zNmZv8mi4v/FMHB/wL4+P8A////CuDg/04TE/9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///wD9/f8rfHz/VQEB/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/z9AQP8B/Pz/AP///wD///8A////AP///wD///8A////IZqa/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/UwYG/x+goP8A////AP///wD///8B+/v/BPPz/wP29v8A/v7/AP///wD///8A////AP///y13d/9VAAD/VQAA/1UAAP9VAAD/RDEx/wD///8A////AP///wD///8kkpL/UwYG/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/0sdHf8O1NT/AP///wD///8A////AP///wD///8B/Pz/PUhI/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/04UFP8br6//Avr6/wD///8A////AP///wD///8A////AP///wD///8A////AP///wza2v9MGRn/VQAA/1UAAP9VAAD/Ri4u/wH6+v8A////AP///wD///8dp6f/UQ0N/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1ELC/8gn5//AP///wD///8A////AP///wD///8M29v/Sx8f/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9MHBz/KYOD/wrg4P8A////AP///wD///8A////AP///wD///8A////AP///w/R0f9BOzv/VQAA/1UAAP9VAAD/UwcH/0UvL/84WFj/KIaG/xi1tf8foqL/TxMT/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UBAf9GLS3/Mmho/yt8fP8lkJD/HqWl/xe5uf8oiIj/UQwM/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/UwYG/0gmJv8wb2//Fru7/wfn5/8D9PT/Buzs/xDNzf8imJj/OlBQ/0sfH/9SCAj/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAPZVAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA9lUAAMZVAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAAxlYAAFxUAAD4VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD3VgAAXFsAAA5UAACVVAAA+FUAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAPdUAACVVQAADwAAAABbAAAOVgAAXFUAAMZVAAD2VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD/VQAA/1UAAP9VAAD2VQAAxlYAAFxVAAAPAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA==\">");
            client.println("</p>");
            client.print("<footer><hr><h5><p>&copy Forrest Erickson as Amused Scientist 2020.</h5></footer>");

            client.println("</body>");            
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
