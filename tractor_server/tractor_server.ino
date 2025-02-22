#include <esp_now.h>
#include <WiFi.h>

//for oled
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#define BUTTON_PIN 12       // Pin for the button
#define GREEN_LED_PIN 14    // Pin for the green LED
#define BLUE_LED_PIN 27     // Pin for the blue LED
#define YELLOW_LED_PIN 13   // Pin for the yellow LED
#define BUZZER_PIN 19       // Pin for the buzzer

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
#define OLED_RESET -1
Adafruit_SH1106 oled((int8_t)OLED_RESET);

// REPLACE WITH THE MAC Address of your seeder esp32 module (adres tego z czarnym kablem)
uint8_t broadcastAddressSeeder[] = {0x3c, 0x71, 0xbf, 0x13, 0x6c, 0xdc};

// Debounce and button variables
int lastSteadyState = LOW;       // the previous steady state from the input pin
int lastFlickerableState = LOW;  // the previous flickerable state from the input pin
int currentState;   
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
int tramlineNumber = 0;

// Structure to send data
typedef struct struct_tractor {
    bool tramlineActive;
} struct_tractor;

// Structure to receive data
typedef struct struct_seeder {
    int turbineRPM;
    int WOMRPM;
    bool mechanismTurning;
    bool tramlineActive;
} struct_seeder;

// Create struct instances
struct_tractor tractorData;
struct_seeder seederData;

// Variable to store if sending data was successful
String success;

// Variables to manage LED states
bool lastMessageDelivered = false;
unsigned long lastBlinkTime = 0;
const int blinkInterval = 1000;

// Fault status variable
String faultStatus = "no fault";

bool enableTurbineAlarm = false;
bool enableWOMAlarm = false;

//Variables for sending data
unsigned long lastMessageSendingTime = 0;
const int sendingInterval = 200;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (status == ESP_NOW_SEND_SUCCESS) {
        success = "Delivery Success :)";
        lastMessageDelivered = true;
    } else {
        success = "Delivery Fail :(";
        lastMessageDelivered = false;
    }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&seederData, incomingData, sizeof(seederData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("turbineRPM: ");
    Serial.println(seederData.turbineRPM);
    Serial.print("WOMRPM: ");
    Serial.println(seederData.WOMRPM);
    Serial.print("mechanismTurning: ");
    Serial.println(seederData.mechanismTurning);
    Serial.print("tramlineActive: ");
    Serial.println(seederData.tramlineActive);

    if (seederData.tramlineActive) {
      digitalWrite(YELLOW_LED_PIN, HIGH);
    } else {
      digitalWrite(YELLOW_LED_PIN, LOW);
    }

    updateDisplay();
    updateFaultStatus();
    

}

void setup() {
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register for Send callback
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddressSeeder, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Register for a callback function that will be called when data is received
    //esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    // Initialize OLED display with address 0x3C for 128x64
    oled.begin(SH1106_SWITCHCAPVCC, 0x3C);
    oled.display();
    oled.clearDisplay(); // Clear display

    // Initialize tractor data
    tractorData.tramlineActive = false;

    // Set pin modes
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    // test the buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);


    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

}

void loop() {
    unsigned long currentMillis = millis();

    // Read button state with debounce
    int currentState = digitalRead(BUTTON_PIN);

    if (currentState != lastFlickerableState) {
    lastDebounceTime = millis();
    lastFlickerableState = currentState;
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if(lastSteadyState == HIGH && currentState == LOW)
      {
        tramlineNumber = (tramlineNumber + 1)%6;

          updateDisplay();

          Serial.print("Button ścieżka: ");
          Serial.println(tramlineNumber+1);

          if(tramlineNumber == 2 || tramlineNumber == 3) tractorData.tramlineActive = true;
          else tractorData.tramlineActive = false;
      }
      lastSteadyState = currentState;
    }

    // Send message via ESP-NOW
    if (millis() - lastMessageSendingTime >= sendingInterval)
    {
      esp_err_t result = esp_now_send(broadcastAddressSeeder, (uint8_t *) &tractorData, sizeof(tractorData));
      lastMessageSendingTime = millis();
    }

    if (lastMessageDelivered) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(BLUE_LED_PIN, LOW);
    } else {
        digitalWrite(GREEN_LED_PIN, LOW);
        if (millis() - lastBlinkTime >= blinkInterval) {
            digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN));
            lastBlinkTime = millis();
        }
    }

}

void updateFaultStatus() {
  bool buzzerActive = false;

  if (enableWOMAlarm && seederData.mechanismTurning && seederData.WOMRPM < 50) {
    faultStatus = "WOM turned off";
    buzzerActive = true;
  } else if (enableTurbineAlarm && seederData.mechanismTurning && seederData.turbineRPM < 50) {
    faultStatus = "Turbine turned off";
    buzzerActive = true;
  } else if (enableWOMAlarm && !seederData.mechanismTurning && seederData.WOMRPM >= 50) {
    faultStatus = "WOM while lifted";
    buzzerActive = true;
  } else {
    faultStatus = "no fault";
  }


  if (buzzerActive) {
    digitalWrite(BUZZER_PIN, millis() % 500 < 250);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

}

void updateDisplay()
{
  if(faultStatus == "no fault")
  {
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);
    oled.print("RPM: ");
    oled.println(seederData.turbineRPM);

    oled.setTextSize(1);
    oled.setCursor(10, 40);
    oled.print("Przejazd: ");

    oled.setTextSize(4);
    oled.setCursor(80, 30);
    oled.print(tramlineNumber + 1); // Display tramline number from 1 to 6
    oled.display();
  } else if ( faultStatus == "WOM turned off") {
    oled.clearDisplay();
    oled.setTextSize(3);
    oled.setTextColor(WHITE);
    oled.setCursor(10, 15);
    oled.print("WOM");
    oled.display();
  } else if ( faultStatus == "Turbine turned off") {
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(10, 20);
    oled.print("Dmuchawa");
    oled.display();
  } else if ( faultStatus == "WOM while lifted") {
    oled.clearDisplay();
    oled.setTextSize(3);
    oled.setTextColor(WHITE);
    oled.setCursor(10, 15);
    oled.print("WOM");
    oled.display();
  }
}
