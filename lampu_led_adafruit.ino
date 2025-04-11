#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

// WiFi credentials
#define WIFI_SSID "POCO F4"
#define WIFI_PASS "akutidaksuka"

// Adafruit IO credentials
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "AkhnafRyan"
#define AIO_KEY         "aio_pEMl91zuhpKtCkgUbZLL88qX39Sv"

// Define the D3 LED pin (GPIO0 on ESP8266)
#define LED_PIN D3

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup the feed you're subscribing to - this will receive commands from IFTTT/Telegram
Adafruit_MQTT_Subscribe ledControl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/lamp");

// Setup a feed for publishing LED status back to Adafruit IO
Adafruit_MQTT_Publish ledStatus = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lamp");

void setup() {
  // Set up serial monitor
  Serial.begin(115200);
  delay(10);
  
  // Initialize the LED pin as an output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED off
  
  // Connect to WiFi
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Setup MQTT subscription for the LED control feed
  mqtt.subscribe(&ledControl);
  
  // Publish initial LED status
  if (mqtt.connected()) {
    ledStatus.publish("OFF");
  }
}

// Function to connect and reconnect to MQTT as needed
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    if (--retries == 0) {
      // basically wait forever until reconnection
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

void loop() {
  // Ensure MQTT connection is alive
  MQTT_connect();
  
  // Read any incoming MQTT subscriptions
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &ledControl) {
      // Convert payload to string
      char *receivedCommand = (char *)ledControl.lastread;
      String command = String(receivedCommand);
      Serial.print("Received: ");
      Serial.println(command);
      
      // Process the command
      if (command.equalsIgnoreCase("ON")) {
        digitalWrite(LED_PIN, HIGH);
        ledStatus.publish("ON");
        Serial.println("LED turned ON");
      } 
      else if (command.equalsIgnoreCase("OFF")) {
        digitalWrite(LED_PIN, LOW);
        ledStatus.publish("OFF");
        Serial.println("LED turned OFF");
      }
      else if (command.equalsIgnoreCase("STATUS")) {
        // Report current LED status
        if (digitalRead(LED_PIN) == HIGH) {
          ledStatus.publish("ON");
          Serial.println("Status: LED is ON");
        } else {
          ledStatus.publish("OFF");
          Serial.println("Status: LED is OFF");
        }
      }
    }
  }
  
  // Ping MQTT broker to keep connection alive
  if (!mqtt.ping()) {
    mqtt.disconnect();
  }
}