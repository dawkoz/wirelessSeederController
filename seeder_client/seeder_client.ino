#include <esp_now.h>
#include <WiFi.h>

// Relay control pin
#define RELAY_PIN 12
#define TURBINE_INDUCTIVE_SENSOR_PIN 14
#define MECHANISM_HALL_SENSOR_PIN 27

// REPLACE WITH THE MAC Address of your tractor esp32 module (mac bialego kabla)
uint8_t broadcastAddressTractor[] = {0x80, 0x7d, 0x3a, 0xf3, 0x4a, 0xa0};

//Variables for sending data
unsigned long lastMessageSendingTime = 0;
const int sendingInterval = 200;

// Structure to send data
typedef struct struct_seeder {
    int turbineRPM;
    int WOMRPM;
    bool mechanismTurning;
    bool tramlineActive;
} struct_seeder;

// Structure to receive data
typedef struct struct_tractor {
    bool tramlineActive;
} struct_tractor;

// Create struct instances
struct_seeder seederData;
struct_tractor tractorData;

// Variable to store if sending data was successful
String success;

// Variables for turbine RPM calculation
volatile int pulseCountTurbine = 0;             // hole in metal detection
unsigned long previousMillisTurbine = 0;
const unsigned long intervalTurbine = 2000;

// Variables for turbine RPM calculation
volatile int pulseCountMechanism = 0;             // magnet detection
unsigned long previousMillisMechanism = 0;
const unsigned long intervalMechanism = 3000;

void IRAM_ATTR turbine_hole_detect() {
    pulseCountTurbine++;
}

void IRAM_ATTR mechanism_magnet_detect() {
    pulseCountMechanism++;
}


int calculateRPM(int pulses, unsigned long intervalMs) {
    return (pulses * (60000 / intervalMs));
}

bool checkIsMechanismTurning(int pulses)
{
  if(pulses>=2) return true;
  else return false;
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (status == ESP_NOW_SEND_SUCCESS) {
        success = "Delivery Success :)";
    } else {
        success = "Delivery Fail :(";
    }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&tractorData, incomingData, sizeof(tractorData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("tramlineActive: ");
    Serial.println(tractorData.tramlineActive);

    if (tractorData.tramlineActive) {
      digitalWrite(RELAY_PIN, LOW);  // Turn on MOSFET
    } else {
      digitalWrite(RELAY_PIN, HIGH);   // Turn off MOSFET
    }
}

void setup() {
    // Init Serial Monitor
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
    memcpy(peerInfo.peer_addr, broadcastAddressTractor, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    // Initialize seeder data
    seederData.turbineRPM = 0;
    seederData.WOMRPM = 0;
    seederData.mechanismTurning = false;
    seederData.tramlineActive = false;
    
    // Set pin modes
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);  // Ensure RELAY is off initially

    pinMode(TURBINE_INDUCTIVE_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TURBINE_INDUCTIVE_SENSOR_PIN), turbine_hole_detect, RISING);

    pinMode(MECHANISM_HALL_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MECHANISM_HALL_SENSOR_PIN), mechanism_magnet_detect, RISING);  // For the magnet sensor
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillisTurbine >= intervalTurbine) {
        previousMillisTurbine = currentMillis;

        seederData.turbineRPM = calculateRPM(pulseCountTurbine, intervalTurbine);

        pulseCountTurbine = 0;

        Serial.print("RPM: ");
        Serial.println(seederData.turbineRPM);
    }

    if (currentMillis - previousMillisMechanism >= intervalMechanism) {
        previousMillisMechanism = currentMillis;

        seederData.mechanismTurning = checkIsMechanismTurning(pulseCountMechanism);

        pulseCountMechanism = 0;

        Serial.print("Mechanism active: ");
        Serial.println(seederData.mechanismTurning);
    }

    seederData.WOMRPM = 540; // Example value
    seederData.tramlineActive = tractorData.tramlineActive; // Reflect tractor's tramline status

    // Send message via ESP-NOW
    if (millis() - lastMessageSendingTime >= sendingInterval)
    {
      esp_err_t result = esp_now_send(broadcastAddressTractor, (uint8_t *) &seederData, sizeof(seederData));
      lastMessageSendingTime = millis();
    }

}
