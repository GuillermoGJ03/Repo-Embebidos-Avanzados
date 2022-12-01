//===========CAN=========
#include <SPI.h>
#include "mcp_can.h"
#include <mcp2515_can.h>

const int spiCSPin = 5;
mcp2515_can CAN(spiCSPin);


bool a = true;
//==========ENDCAN=============
//==========PV=================
float myFloat;
uint8_t *array1;
unsigned char b[4]={0};

float valorAnterior = 0;
float valorActual;

int flag_send;

float pressure_sensor;
//==========END PV=================
//==========FIREBASE===========
#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Memo"
#define WIFI_PASSWORD "Arsenal987"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBH94lKsXSDNGF9j5_PGlwYXFaa6JyX6oA"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "a01654262@tec.mx"
#define USER_PASSWORD "Quintoes1#"

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "https://jhondeere-b2861-default-rtdb.firebaseio.com/"

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Variables to save database paths
String databasePath;
String tempPath;
String humPath;
String presPath;
String ledPath;

// BME280 sensor
//Adafruit_BME280 bme; // I2C
float temperature;
float humidity;
float pressure;
float contador =1.0;

// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 500;

//==========ENDFIREBASE===========


//=========FUNTIONS START========
// Initialize WiFi
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
}
// Write float values to the database
void sendFloat(String path, float value){
  if (Firebase.RTDB.setFloat(&fbdo, path.c_str(), value)){
    Serial.print("Writing value: ");
    Serial.print (value);
    Serial.print(" on the following path: ");
    Serial.println(path);
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
  }
  else {
    Serial.println("FAILED");
    //Serial.println("REASON: " + fbdo.errorReason());
  }
}



//FLOATUNION_t myFloat;
//read value
void readData(String path){
  String readIncoming ="";
  if (Firebase.RTDB.getString(&fbdo, path.c_str())){
    //Serial.println("PATH: " + fbdo.dataPath());
    //Serial.println("TYPE: " + fbdo.dataType());
    if(fbdo.dataType()=="string"){
      readIncoming = fbdo.stringData();
       Serial.println("DATA READ: " + readIncoming);
        myFloat = readIncoming.toFloat();

        //=====FLOAT TO 4BYTES========

        memcpy(b,&myFloat,4);
        /*
        for(int i=0; i<4; i++){
          Serial.println(b[i],BIN);
        }
        */

        array1 = (uint8_t*)(&myFloat);
        /*
            for(int i = 0; i<4; i++)
            {
                Serial.print(array1[i]);
                Serial.print("\t");
            }
            Serial.println();
         */
                
     }
    }
  else {
    Serial.println("FAILED READ");
    Serial.println("REASON: " + fbdo.errorReason());
  }
}




void send_can(){
  // put your main code here, to run repeatedly:

  
  unsigned char stmp[8];
  unsigned char stmp2[8] = {255, 128, 64, 32, 16, 8, 4, 2};
  
  stmp[0] = b[0];
  stmp[1] = b[1];
  stmp[2] = b[2];
  stmp[3] = b[3];
  stmp[4] = 0xFF;
  stmp[5] = 0xFF;
  stmp[6] = 0xFF;
  stmp[7] = 0xFF;
  
  
  Serial.println("==============================Can: send================================");


  


        //=====COMPARATION========
        //valorActual = myFloat;
        if (myFloat != valorAnterior){
          //Serial.println("####################### Son distintos #######################");
          flag_send = 1;
          valorAnterior = myFloat;
          CAN.sendMsgBuf(0x1F9, 0, 8, stmp); //
        }else if (myFloat == valorAnterior){
          //Serial.println("####################### Son iguales ####################### ");
          flag_send = 0;
        } else{
          Serial.println("Error comparacion");  
        }

        
  
}

void read_send_can(){
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned char vBuffer[4];
  delay(200);
  send_can();
  if(CAN_MSGAVAIL == CAN.checkReceive())
  {
    Serial.println("====================================================================================REsivo");
    CAN.readMsgBuf(&len, buf);

    unsigned long canId = CAN.getCanId();

    Serial.println("-----------------------------");
    Serial.print("Data from ID: 0x");
    Serial.println(canId, HEX);
    

    for(int i = 0; i<len; i++)
    {
        Serial.print(buf[i]);
        Serial.print("\t");
    }
    Serial.println();

      Serial.println("Conversion");
    
      vBuffer[0] = buf[0];
      vBuffer[1] = buf[1];
      vBuffer[2] = buf[2];
      vBuffer[3] = buf[3];
  
      pressure_sensor = *(float *)&vBuffer;
      Serial.println(pressure_sensor);
      

    if(canId == 65262){
      
    }

    
  }

}


//=========FUNCTION ENDS=========


void setup() {
  // put your setup code here, to run once:

//=======SETUP CAN=========
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
      Serial.println("CAN BUS Init Failed");
      delay(100);
  }
  Serial.println("CAN BUS  Init OK!");

//=======END SETUP CAN=======

//=======SETUP FIREBASE======
  initWiFi();

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);

  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  // Update database path
  databasePath = "/UsersData/" + uid;

  // Update database path for sensor readings
  tempPath = databasePath + "/temperature"; // --> UsersData/<user_uid>/temperature
  humPath = databasePath + "/humidity"; // --> UsersData/<user_uid>/humidity
  presPath = databasePath + "/pressure"; // --> UsersData/<user_uid>/pressure
  ledPath = databasePath + "/led"; // --> UsersData/<user_uid>/led

//=======END FIREBASE========


}

void loop() {
  // Send new readings to database
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    
    ///////COMENTAN ESTA SECCION SI DESEAN UTILIZAR EL BME////////////////
    if(contador>20){contador = 1.0;}
      
    contador = contador+1;
    temperature = contador*0.2;
    humidity = contador*0.3;
    pressure = pressure_sensor;


    // Send readings to database:
    sendFloat(tempPath, temperature);
    sendFloat(humPath, humidity);
    sendFloat(presPath, pressure);
    readData(ledPath);
    Serial.println(myFloat);
    read_send_can();
    delay(50);
  }

}
