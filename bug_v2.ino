#include <WiFi.h>
#include <esp_now.h>

#define CHANNEL 1

const int freq = 30000;
const int pwmChannel_1 = 0;
const int pwmChannel_2 = 1;
const int resolution = 8;

// Motor A
int motor1Pin1 = 12;
int motor1Pin2 = 13;
int enable1Pin_1 = 14;

// Motor B
int motor1Pin3 = 26;
int motor1Pin4 = 27;
int enable1Pin_2 = 15;

bool gotData = false;

char* inputController = NULL;
char* cBut0;
char* cBut1;
char* cBut2;
char* cBut3;
char* cBut4;
char* cBut5;
char* cBut6;
char* cBut7;
char* cBut8;
char* cBut9;
char* cBut10;
char* cBut11;

char* cJoyXL;
char* cJoyXR;
char* cJoyYL;
char* cJoyYR;

int But0, But1, But2, But3, But4, But5, But6, But7, But8, But9, But10, But11, joyXL, joyYL, joyXR, joyYR = 1;
int lSpeed, rSpeed = 0;
int ySpeedModifier, xSpeedModifier = 0;
int butSpeed = 0;


void InitESPNow();
void configDeviceAP();
void OnDataRecv(const uint8_t *, const uint8_t *, int );

enum dirs {
  forward, reverse, stopped
};

dirs dir;

void setup()
{
  Serial.begin(115200);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin_1, OUTPUT);
  pinMode(motor1Pin3, OUTPUT);
  pinMode(motor1Pin4, OUTPUT);
  pinMode(enable1Pin_2, OUTPUT);

  ledcSetup(pwmChannel_1, freq, resolution);
  ledcSetup(pwmChannel_2, freq, resolution);

  ledcAttachPin(enable1Pin_1, pwmChannel_1);
  ledcAttachPin(enable1Pin_2, pwmChannel_2);

  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}


void loop()
{

  for (;;)
  {
  }
}

void InitESPNow() {
  while (true) {
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
      Serial.println("ESPNow Init Success");
      break;
    }
    else {
      Serial.println("ESPNow Init Failed");
      // Retry InitESPNow, add a counte and then restart?
      // InitESPNow();
      // or Simply Restart
    }
    delay(100);
  }
}

void configDeviceAP() {
  char* SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {

  inputController = (char*)data;
  //Serial.println(inputController);

  char* tokTemp = NULL;
  char* tokenString = inputController;

  cBut0 = strtok_r(tokenString, "|", &tokTemp);
  cBut1 = strtok_r(NULL, "|", &tokTemp);
  cBut2 = strtok_r(NULL, "|", &tokTemp);
  cBut3 = strtok_r(NULL, "|", &tokTemp);
  cBut4 = strtok_r(NULL, "|", &tokTemp);
  cBut5 = strtok_r(NULL, "|", &tokTemp);
  cBut6 = strtok_r(NULL, "|", &tokTemp);
  cBut7 = strtok_r(NULL, "|", &tokTemp);
  cBut8 = strtok_r(NULL, "|", &tokTemp);
  cBut9 = strtok_r(NULL, "|", &tokTemp);
  cBut10 = strtok_r(NULL, "|", &tokTemp);
  cBut11 = strtok_r(NULL, "|", &tokTemp);
  cJoyXL = strtok_r(NULL, "|", &tokTemp);
  cJoyYL = strtok_r(NULL, "|", &tokTemp);
  cJoyXR = strtok_r(NULL, "|", &tokTemp);
  cJoyYR = strtok_r(NULL, "^", &tokTemp);

  joyXL = atoi(cJoyXL);
  joyYL = atoi(cJoyYL);
  joyXR = atoi(cJoyXR);
  joyYR = atoi(cJoyYR);
  But0 = atoi(cBut0);
  But1 = atoi(cBut1);
  But2 = atoi(cBut2);
  But3 = atoi(cBut3);
  But4 = atoi(cBut4);
  But5 = atoi(cBut5);
  But6 = atoi(cBut6);
  But7 = atoi(cBut7);
  But8 = atoi(cBut8);
  But9 = atoi(cBut9);
  But10 = atoi(cBut10);
  But11 = atoi(cBut11);

  if (joyXR < 1700) {
    Serial.println("reverse");
    dir = reverse;
    //going in reverse
    xSpeedModifier = map(joyXR, 0, 1700, 255, 0);
    rSpeed =  xSpeedModifier;
    lSpeed = rSpeed;
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor1Pin3, LOW);
    digitalWrite(motor1Pin4, HIGH);
  }
  else if (joyXR > 1950) {

    Serial.println("forward");
    dir = forward;
    //moving forward
    xSpeedModifier = map(joyXR, 1900, 4095, 0, 255);
    rSpeed = xSpeedModifier;
    lSpeed = rSpeed;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor1Pin3, HIGH);
    digitalWrite(motor1Pin4, LOW);
  }
  else {

    Serial.println("stopped");
    dir = stopped;
    rSpeed = 0;
    lSpeed = 0;
    //    digitalWrite(motor1Pin1, LOW);
    //    digitalWrite(motor1Pin2, HIGH);
    //    digitalWrite(motor1Pin3, HIGH);
    //    digitalWrite(motor1Pin4, LOW);

  }

  //Modify left and right speeds based on direction

  if (joyYR < 1600) {

    Serial.println("moving left");
    //moving left
    ySpeedModifier = map(joyYR, 1600, 0, 0, 255);
    lSpeed = rSpeed - (ySpeedModifier/3);
    rSpeed = rSpeed + ySpeedModifier;

    if (dir == stopped) {
      //left reverse right forward
      lSpeed = ySpeedModifier;
      rSpeed = ySpeedModifier;
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor1Pin3, HIGH);
      digitalWrite(motor1Pin4, LOW);
    }
  }
  else if (joyYR > 2200) {
    Serial.println("moving right");
    //moving right
    ySpeedModifier = map(joyYR, 4095, 2150, 255, 0);
    rSpeed =  lSpeed - (ySpeedModifier/3);
    lSpeed = lSpeed + ySpeedModifier;

    if (dir == stopped) {
      lSpeed = ySpeedModifier;
      rSpeed = ySpeedModifier;
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor1Pin3, LOW);
      digitalWrite(motor1Pin4, HIGH);
    }
  }
  else {

  }

  //check buttons
  if (!But0) {
    if (butSpeed < 255) {
      butSpeed++;
    }
  }
  if (!But1) {
    if (butSpeed > -255) {
      butSpeed--;
    }
  }

  lSpeed = lSpeed + butSpeed;
  rSpeed = rSpeed + butSpeed;

  if ( lSpeed > 255) {
    lSpeed = 255;
  }
  if ( rSpeed > 255) {
    rSpeed = 255;
  }
  if (lSpeed < 0) {
    lSpeed = 0;
  }
  if (rSpeed < 0) {
    rSpeed = 0;
  }


  //Serial.print("lSpeed: "); Serial.print(lSpeed); Serial.print("  rSpeed: "); Serial.println(rSpeed);

  ledcWrite(pwmChannel_1, lSpeed);
  ledcWrite(pwmChannel_2 , rSpeed);

}

