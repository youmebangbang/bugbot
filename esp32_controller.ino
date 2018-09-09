//#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>


#define CHANNEL 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

esp_now_peer_info_t slave;

char* readController();

int But0, But1, But2, But3, But4, But5, But6, But7, But8, But9, But10, But11, joyXL, joyYL, joyXR, joyYR;
//SSD1306  display(0x3c, 21, 22);
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

int butSpeed = 0;


void setup() {

  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.clearDisplay();
  display.setRotation(2);
  display.println("BangBang\nController\nwith\nESP-NOW");
  display.display();

  pinMode(2, INPUT_PULLUP);   //button 0
  pinMode(4, INPUT_PULLUP);   //button 1
  pinMode(5, INPUT_PULLUP);   //button 2
  pinMode(12, INPUT_PULLUP);   //button 3
  pinMode(13, INPUT_PULLUP);   //button 4
  pinMode(14, INPUT_PULLUP);  //button 5
  pinMode(15, INPUT_PULLUP);  //button 6
  pinMode(16, INPUT_PULLUP);  //button 7
  pinMode(17, INPUT_PULLUP);   //button 8
  pinMode(18, INPUT_PULLUP);  //button 9
  pinMode(19, INPUT_PULLUP);  //button 10
  pinMode(23, INPUT_PULLUP);  //button 11
  pinMode(36, INPUT);  //joyXL
  pinMode(39, INPUT);  //joyYL
  pinMode(35, INPUT);  //joyXR
  pinMode(34, INPUT);  //joyYR



  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  InitESPNow();
  esp_now_register_send_cb(OnDataSent);

  bool foundSlave = false;
  ScanForSlave();
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    bool isPaired = manageSlave();
    if (isPaired) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Slave\npaired!");
      display.display();


    } else {
      Serial.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }
  delay(3000);


}
void loop() {

  char* outData = readController();
  sendData(outData);
  free(outData);

  delay(100);
}


char* readController() {

  char * conString = (char*)malloc(100 * sizeof(char));

  But0 = digitalRead(2);
  But1 = digitalRead(4);
  But2 = digitalRead(5);
  But3 = digitalRead(12);
  But4 = digitalRead(13);
  But5 = digitalRead(14);
  But6 = digitalRead(15);
  But7 = digitalRead(16);
  But8 = digitalRead(17);
  But9 = digitalRead(18);
  But10 = digitalRead(19);
  But11 = digitalRead(23);

  joyXR = analogRead(36);
  joyYR = analogRead(39);
  joyXL = analogRead(35);
  joyYL = analogRead(34);

  sprintf(conString, "%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d^", But0, But1, But2, But3, But4, But5, But6, But7, But8, But9, But10, But11, joyXL, joyYL, joyXR, joyYR);

  Serial.println(conString);

  if (!But0) {
    butSpeed++;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Speed:");
    display.println(butSpeed);
    display.display();
  }
  if (!But1) {
    butSpeed--;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Speed:");
    display.println(butSpeed);
    display.display();
  }

  return conString;

}

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Scanning\nslaves");
  display.display();

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    const esp_now_peer_info_t *peer = &slave;
    const uint8_t *peer_addr = slave.peer_addr;
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(peer);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  const esp_now_peer_info_t *peer = &slave;
  const uint8_t *peer_addr = slave.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

void sendData(char outputString[]) {
  const uint8_t *peer_addr = slave.peer_addr;

  Serial.println(outputString);
  esp_err_t result = esp_now_send(peer_addr, (const uint8_t*)outputString, strlen(outputString));

  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //  char macStr[18];
  //  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  //  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  //Serial.print(outputString); Serial.println(" sent.");

}


