

/***************************************************
  Adafruit ESP8266 Sensor Module
  
  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino
  Works great with Adafruit's Huzzah ESP board:
  ----> https://www.adafruit.com/product/2471
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// Libraries
#include <ESP8266WiFi.h>
// For HTTP server, but not for minimal use.
//#include <WiFiClient.h>
//#include <ESP8266WebServer.h>
//#include <ESP8266mDNS.h>



#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
//#include <Adafruit_MQTT_FONA.h>


#include <DHT.h>
#include <DHT_U.h>

// DHT 11 sensor
#define DHTPIN 5
#define DHTTYPE DHT22 

// WiFi parameters
#define WLAN_SSID       "dlink-fc9d3b"
#define WLAN_PASS       "leonardodavinci"

// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "foice"
#define AIO_KEY         "ff3ecb0ecb674ccd8fe41dd7f4e27b0e"

// DHT sensor
DHT dht(DHTPIN, DHTTYPE, 15);

// Functions
void connect();

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

const bool _connect = true;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);


/****************************** Feeds ***************************************/



// Setup feeds for temperature & humidity
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt,  AIO_USERNAME "/feeds/humidity");


/*************************** Sketch Code ************************************/

//ESP8266WebServer server(80);

const int led = 13;


void setup() {

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output

  // Init sensor
  dht.begin();

  Serial.begin(115200);
  Serial.println(F("Adafruit IO Example"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());

//  if (MDNS.begin("esp8266")) {
//    Serial.println("MDNS responder started");
//  }
 

  // connect to adafruit io
  if (_connect){
      connect();
  }
}

unsigned long timeLastCheck = 0;
void loop() {
  
  unsigned long intervalCheck = 600000;
  unsigned long timeNow = millis();
  if ( timeLastCheck == 0 || timeNow-timeLastCheck > intervalCheck )
  { 

      Serial.println(timeLastCheck);
      Serial.println(timeNow-timeLastCheck);
      
      
      // ping adafruit io a few times to make sure we remain connected
      if(! mqtt.ping(3)) {
        // reconnect to adafruit io
        if(! mqtt.connected()) {
            connect(); 
          }
      }
    
    
    
      // Grab the current state of the sensor
      float humidity_data = (float)dht.readHumidity();
      float temperature_data = (float)dht.readTemperature();
      bool write_always = false;
      if ( write_always || ( false == isnan(temperature_data) &&  false == isnan(humidity_data) && humidity_data < 100.0 && temperature_data < 500.00) ) {
          timeLastCheck = timeNow;
        
        
          //float temp = dht.readTemperature();
          //float humi = dht.readHumidity();
        
          String string_temp =  String(temperature_data, 2); 
          String string_humi =  String(humidity_data, 2); 
        
          String stringOut =  String(string_temp+" ˚C "+string_humi+"%");
        
          //char*  output = strcat("Temperature",str(temp));
          
          Serial.println(stringOut);
        
          //Serial.println(F("Humidity"));
          //Serial.println(humi);
//          server.handleClient();
          
          if (_connect){
                digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
                                                  // but actually the LED is on; this is because 
                                                  // it is acive low on the ESP-01)
                delay(100);                      // Wait for a second
                digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
                // Publish data
                if (! temperature.publish(temperature_data))
                  Serial.println(F("Failed to publish temperature"));
                else
                  Serial.println(F("Temperature published!"));
              
                if (! humidity.publish(humidity_data))
                  Serial.println(F("Failed to publish humidity"));
                else
                  Serial.println(F("Humidity published!"));
          }
      }
  }

}

// connect to adafruit io via MQTT
void connect() {

  Serial.print(F("Connecting to Adafruit IO... "));

  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {

    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(60000);

  }

  Serial.println(F("Adafruit IO Connected!"));

}

