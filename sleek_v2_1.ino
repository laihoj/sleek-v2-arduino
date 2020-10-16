#include <FreeRTOS_SAMD21.h>
#define  ERROR_LED_PIN  13 //Led Pin: Typical Arduino Board
#define ERROR_LED_LIGHTUP_STATE  HIGH // the state that makes the led light up on your board, either low or high


#include <FlashStorage.h>

#include <WiFiNINA.h>
int status = WL_IDLE_STATUS;
int IMU_status = 0;
WiFiServer server(80);

#define SECRET_SSID "sleek_device_1nf7noui2t"   //TODO: generate the identifier thing

#include <WiFiUdp.h>

WiFiUDP Udp;
unsigned int localPort = 9999;      // local port to listen on

#include <ArduinoJson.h>

StaticJsonDocument<200> doc;

#include <Arduino_LSM6DS3.h>

float acceleration_x, acceleration_y, acceleration_z, gyroscope_x, gyroscope_y, gyroscope_z;

#define SERIAL          SerialUSB //Sparkfun Samd21 Boards

typedef struct {
  boolean valid;      //if any credentials are provided
  boolean confirmed;  //if provided credentials are known to work
  char ssid[100];
  char password[100];
} Credentials;

typedef struct {
  boolean valid;      //if any credentials are provided
  char ip[100];
  int port;
} UDPTarget;

FlashStorage(my_flash_store, Credentials);

Credentials creds;

FlashStorage(udpTarget_flash_store, UDPTarget);
UDPTarget udpTarget;

//arbitrarily chosen values
#define PROGRAM_STATE_CLEANUP 4
#define PROGRAM_STATE_WIFI 5
#define PROGRAM_STATE_UDP 6
#define PROGRAM_STATE_NORMAL 7

int PROGRAM_STATE = 1;

void SET_STATE(int state) {
  PROGRAM_STATE = state;
}



TaskHandle_t Handle_ProgramFlowTask;
TaskHandle_t Handle_LEDPatternTask;

TaskHandle_t Handle_StartWiFiTask;
TaskHandle_t Handle_EndWiFiTask;
TaskHandle_t Handle_WifiModuleCheckTask;
TaskHandle_t Handle_WifiCredCheckTask;
TaskHandle_t Handle_APModeTask;
TaskHandle_t Handle_APClientTask;
TaskHandle_t Handle_ConnectWiFiTask;

TaskHandle_t Handle_monitorTask;

TaskHandle_t Handle_StartUDPTask;
TaskHandle_t Handle_EndUDPTask;
TaskHandle_t Handle_findLocalNodeTask;
TaskHandle_t Handle_UDPTargetCheckTask;

TaskHandle_t Handle_StartNormalOperationTask;
TaskHandle_t Handle_ReadSensorsTask;
TaskHandle_t Handle_TransmitReadingsTask;



SemaphoreHandle_t sem_WIFI_COMPLETE;
SemaphoreHandle_t sem_WIFI_CLEANUP;
//SemaphoreHandle_t sem_WIFI_COMPLETE;
SemaphoreHandle_t sem_UDP_COMPLETE;
SemaphoreHandle_t sem_NORMAL_COMPLETE;

SemaphoreHandle_t semWiFi, semUDP, semNORM;

SemaphoreHandle_t semA, semB, semC, semD, semF;

SemaphoreHandle_t UDP_semA, UDP_semB, UDP_semC;

SemaphoreHandle_t NORM_semA, NORM_semB;


void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );
}

void println(String text) {
  if (SERIAL) {
    SERIAL.println(text);
  }
}

void print(String text) {
  if (SERIAL) {
    SERIAL.print(text);
  }
}

void flush() {
  if (SERIAL) {
    SERIAL.flush();
  }
}

void write(char c) {
  if (SERIAL) {
    SERIAL.write(c);
  }
}

void println(int i) {
  String s = String(i);
  println(s);
}

void println() {
  print("\n");
}

void print(int i) {
  String s = String(i);
  print(s);
}


//*****************************************************************
// Create a thread that reads sensor for acceleration and gyroscope
// this task will delete itself
//*****************************************************************
static void readSensorsThread( void *pvParameters )
{
  println("Starting readSensorsThread");
  IMU_status = IMU.begin();
  if (!IMU_status) {
    println("Failed to initialize IMU!");
    while (true); //maybe consider to handle this in a task kind of way
  } else {
    println("IMU operational");
  }
  while (true) {
    xSemaphoreTake( NORM_semA, portMAX_DELAY );
    if (PROGRAM_STATE != PROGRAM_STATE_NORMAL) {
      xSemaphoreTake( NORM_semA, portMAX_DELAY );
      println("readSensorsThread Thread finished");
      vTaskDelete( NULL );
    }
    //simulate execution
    myDelayMs(100);
    readSensors();
    //    println("reading sensor");

    xSemaphoreGive( NORM_semB );
  }
}


void readSensors() {
  if (IMU_status) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(acceleration_x, acceleration_y, acceleration_z);
    }

    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gyroscope_x, gyroscope_y, gyroscope_z);
    }
  }
}

//*****************************************************************
// Create a thread that transmits readings
// this task will delete itself TODO when?
//*****************************************************************
static void transmitReadingsThread( void *pvParameters )
{
  Udp.begin(localPort);
  println("Starting transmitReadingsThread");
  while (true) {
    xSemaphoreTake( NORM_semB, portMAX_DELAY );
    if (PROGRAM_STATE != PROGRAM_STATE_NORMAL) {
      vSemaphoreDelete(NORM_semB);
      println("transmitReadingsThread finished");
      vTaskDelete( NULL );
    }
    //simulate execution
    //    myDelayMs(500);
    //    println("transmitting readings");

    transmitReadings();
    //    String payload = buildPayload();
    //    println(payload);

    xSemaphoreGive( NORM_semA );
  }
}

void transmitReadings() {
  Udp.beginPacket(udpTarget.ip, udpTarget.port);
  String msgbuffer = buildPayload();
  char chrbuffer[100];
  msgbuffer.toCharArray(chrbuffer, 100);
  Udp.write(chrbuffer);
  Udp.endPacket();
}

String buildPayload() {
  String payload = "{";
  payload += "\"ax\": \"";
  payload += acceleration_x;
  payload += "\",";
  payload += "\"ay\": \"";
  payload += acceleration_y;
  payload += "\",";
  payload += "\"az\": \"";
  payload += acceleration_z;
  payload += "\",";
  payload += "\"gx\": \"";
  payload += gyroscope_x;
  payload += "\",";
  payload += "\"gy\": \"";
  payload += gyroscope_y;
  payload += "\",";
  payload += "\"gz\": \"";
  payload += gyroscope_z;
  payload += "\"";
  payload += "}";
  return payload;
}



//*****************************************************************
// Create a thread that checks wifi credentials presence
// this task will delete itself after finding credentials
//*****************************************************************
static void checkUDPTargetThread( void *pvParameters )
{
  boolean succ = false;
  println("Checking UDP Target: Thread Started");
  while ( true ) {
    xSemaphoreTake( UDP_semA, portMAX_DELAY );
    if (PROGRAM_STATE != PROGRAM_STATE_UDP) {
      vSemaphoreDelete(UDP_semA);
      println("Checking UDP Target: Thread Deleted");
      vTaskDelete( NULL );
    }

    checkUDPTarget();
    //    myDelayMs(1000); // every 1 second
    //    if(!succ) {
    //      println("udp target bad");
    //      xSemaphoreGive( UDP_semB );
    //      succ = true;
    //    } else {
    //      println("udp target good");
    //      xSemaphoreGive( sem_UDP_COMPLETE );
    //    }
  }
}

void checkUDPTarget() {
  udpTarget = udpTarget_flash_store.read();
  myDelayMs(500); // every 0.5 second
  if (udpTarget.valid == false) {
    println("UDP TARGET NOT KNOWN");
    flush();
    xSemaphoreGive( UDP_semB );
  } else {
    println("Using Known UDP");
    xSemaphoreGive( sem_UDP_COMPLETE );
  }

}

//*****************************************************************
// Create a thread that
// this task will delete itself
//*****************************************************************
static void findLocalNodeThread( void *pvParameters )
{
  println("findLocalNodeThread: Started");
  while (  true  ) {
    xSemaphoreTake( UDP_semB, portMAX_DELAY );
    if (PROGRAM_STATE != PROGRAM_STATE_UDP) {
      vSemaphoreDelete(UDP_semB);
      println("Requesting Local Node Details: Deleted");
      vTaskDelete( NULL );
    }

    findLocalNode();
  }
}

char context_server[] = "sleek-v2-firebase.herokuapp.com";    // name address for Google (using DNS)
void findLocalNode() {

  WiFiClient client;
  if (client.connect(context_server, 80)) {
    println("connected to server");
    // Make a HTTP request:
    client.println("GET /api/node HTTP/1.1");
    client.println("Host: sleek-v2-firebase.herokuapp.com");
    client.println("Connection: close");
    client.println();
  }

  boolean looping = true;
  while (looping) {
    char prevChar = 'a';  //dummy char
    String currentLine = ""; //store response body
    boolean bodyFound = false;
    while (client.available()) {
      char c = client.read();
      if (bodyFound) {
        currentLine += c; //accumulate characters only when body upcoming
      }

      if (prevChar == '\n' && c == '\r') { //somehow this means body starting
        bodyFound = true;
      }
      prevChar = c;
    }
    currentLine.trim(); //clear whitespace
    if (bodyFound) {
      DeserializationError error = deserializeJson(doc, currentLine); //body is JSON, try to parse
      if (error) {
        print(F("deserializeJson() failed: "));
        println(error.c_str());
      } else {
        bodyFound = false;

        udpTarget.port = doc["port"];
        //          udpTarget.ip = doc["ip"];
        String ip = doc["ip"];
        ip.toCharArray(udpTarget.ip, 100);
        udpTarget.valid = true;
        udpTarget_flash_store.write(udpTarget);
        println(udpTarget.port);

        SERIAL_PORT_MONITOR.println();
        SERIAL_PORT_MONITOR.print("UDP IP: ");
        SERIAL_PORT_MONITOR.println(udpTarget.ip);
        SERIAL_PORT_MONITOR.print("UDP PORT: ");
        SERIAL_PORT_MONITOR.println(udpTarget.port);
        SERIAL_PORT_MONITOR.println("have been saved.");
        xSemaphoreGive( UDP_semA );

      }
    }

    // if the server's disconnected, stop the client:
    if (!client.connected()) {
      println();
      println("disconnecting from server.");
      client.stop();
      looping = false;
    }
  }
}

//*****************************************************************
// Create a thread that connects to wifi using credentials
// this task will delete itself TODO when?
//*****************************************************************
static void connectWifiThread( void *pvParameters )
{
  boolean succ = false;
  println("Connecting to WiFi: Started");
  while (  true  ) {
    xSemaphoreTake( semD, portMAX_DELAY );
    if (PROGRAM_STATE != PROGRAM_STATE_WIFI) {
      vSemaphoreDelete(semD);
      xSemaphoreGive(sem_WIFI_CLEANUP);
      println("Connecting to WiFi thread: Deleted");
      vTaskDelete( NULL );
    }
    //THREAD EXECUTION
    myDelayMs(1000);

    connectWifi();
  }
}

void connectWifi() {
  // attempt to connect to Wifi network:
  int MAX_ATTEMPTS = 3; //don't attempt more than 3 times
  int attempts = 0;
  while (status != WL_CONNECTED  && attempts < MAX_ATTEMPTS) {
    attempts++;
    print("Attempt ");
    println(attempts);
    print("Attempting to connect to SSID: ");
    println(creds.ssid);
    status = WiFi.begin(creds.ssid, creds.password);
    // wait 10 seconds for connection:
    myDelayMs(10000);
  }
  if (status == WL_CONNECTED) {
    //success
    println("Connected to wifi");
    printWiFiStatus();
    if (creds.confirmed == false) {
      creds.confirmed = true;
      my_flash_store.write(creds);
      println("Acknowledged first time connecting to wifi");
    }
    xSemaphoreGive( sem_WIFI_COMPLETE );
  } else {
    //fail
    print("Reason code: ");
    println(WiFi.reasonCode());
    creds.valid = false;
    my_flash_store.write(creds);
    println("Failed to connect to WiFi, try entering different credentials");
    xSemaphoreGive( semC );
  }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  print("SSID: ");
  println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  print("IP Address: ");
  println(ip);

  // print where to go in a browser:
  print("To see this page in action, open a browser to http://");
  println(ip);

}


//*****************************************************************
// Create a thread that opens access point network
// this task will delete itself after TODO when will this delete?
//*****************************************************************
static void openAccessPointThread( void *pvParameters )
{
  println("Open Access Point: Started");
  while (  true   ) {
    xSemaphoreTake( semC, portMAX_DELAY );
    if (PROGRAM_STATE != PROGRAM_STATE_WIFI) {
      vSemaphoreDelete(semC);
      xSemaphoreGive(sem_WIFI_CLEANUP);
      println("Open Access Point: Deleted");
      vTaskDelete( NULL );
    }

    myDelayMs(1000);
    openAccessPoint();
    println("AP good");

    xSemaphoreGive( semF );
  }
}

void openAccessPoint() {
  //  char ssid[] = SECRET_SSID;        // your network SSID (name)
  //  "sleek_device_1nf7noui2t"
  randomSeed(analogRead(0));
  int DEVICE_ID = random(2147483647);
  String ssid_string = "sleek_device_" + String(DEVICE_ID);
  char ssid[100];
  ssid_string.toCharArray(ssid, 100);
  // print the network name (SSID);
  print("Creating access point named: ");
  println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  //  status = WiFi.beginAP(ssid, pass);
  status = WiFi.beginAP(ssid);

  if (status != WL_AP_LISTENING) {
    println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  myDelayMs(10000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}


//*****************************************************************
// Create a thread that opens access point network
// this task will delete itself after TODO when will this delete?
//*****************************************************************
static void handleAccessPointClientsThread( void *pvParameters )
{
  println("AP Client handling: Started");
  while (  true ) {
    xSemaphoreTake( semF, portMAX_DELAY );
    if (PROGRAM_STATE != PROGRAM_STATE_WIFI) {
      vSemaphoreDelete(semF);
      xSemaphoreGive(sem_WIFI_CLEANUP);
      println("AP Cient handling: Deleted");
      vTaskDelete( NULL );
    }

    handleAccessPointClients();
    //    myDelayMs(1000);
    //
    //    println("AP clients good");
    //    xSemaphoreGive( semB );
  }
}

void handleAccessPointClients() {

  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      println("Device disconnected from AP");
    }
  }

  boolean gotCreds = false;
  //handle clients
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            //            client.println("Refresh: 5");
            client.println();

            // the content of the HTTP response follows the header:
            client.println("<!DOCTYPE HTML>");
            client.println("<html><head>");
            client.println("<link rel=\"stylesheet\" href=\"https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css\" integrity=\"sha384-BVYiiSIFeK1dGmJRAkycuHAHRg32OmUcww7on3RYdg4Va+PmSTsz/K68vbdEjh4u\" crossorigin=\"anonymous\">");
            client.println("</head><body>");
            client.println("<form method='GET' action='/'><br>");
            client.println("<input type='text' name='ssid' placeholder='ssid'><br>");
            client.println("<input type='text' name='pwd' placeholder='password'><br>");
            client.println("<input type='submit' value='Submit'></form>");
            client.println("</body></html>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        if (currentLine.endsWith("GET /?")) {
          String paramString = "";  //collect chars into parameter string
          char ch = 'a';            //char inited with some dummy char
          boolean readingParams = true;
          while (client.available() && readingParams) {             // if there's bytes to read from the client,
            ch = client.read();
            currentLine += ch;
            if (ch == '\n' || ch == ' ') { //end of parameter string found
              readingParams = false;
            } else {
              paramString += ch;
            }
          }

          //params found
          boolean extracting = true;


          while (extracting) {
            int i = paramString.indexOf('=');
            if (i < 0) {
              break;
            }
            String key = paramString.substring(0, i); //extract key //find first '='//read value of key
            paramString = paramString.substring(i + 1); //discard key including '='
            i = paramString.indexOf('&');       //find value //find first '&'
            String value = paramString.substring(0, i);
            paramString = paramString.substring(i + 1); //discard value including '&'

            //process each param according to logic
            if (key == "ssid" ) {
              println("SSID FOUND, VALUE = " + value);
              value.toCharArray(creds.ssid, 100);
              //
            } else if (key == "pwd") {
              println("PWD FOUND, VALUE = " + value);
              value.toCharArray(creds.password, 100);
            }

            if (paramString.length() <= 0) { //exit if all of string processed //maybe doesn't even work
              extracting = false;
            }
          }

          creds.valid = true;
          creds.confirmed = false;

          gotCreds = true;

          //          // ...and finally save everything into "my_flash_store"
          //          my_flash_store.write(creds);
          //
          //          SERIAL_PORT_MONITOR.println();
          //          SERIAL_PORT_MONITOR.print("Your ssid: ");
          //          SERIAL_PORT_MONITOR.println(creds.ssid);
          //          SERIAL_PORT_MONITOR.print("and your password: ");
          //          SERIAL_PORT_MONITOR.println(creds.password);
          //          SERIAL_PORT_MONITOR.println("have been saved. Thank you!");
          //
          ////          WiFi.end();
          //          xSemaphoreGive( semB);
          //          xSemaphoreTake( semF, portMAX_DELAY );


        }
      }
    }
    // close the connection:

    if (gotCreds) {
      // ...and finally save everything into "my_flash_store"
      my_flash_store.write(creds);

      SERIAL_PORT_MONITOR.println();
      SERIAL_PORT_MONITOR.print("Your ssid: ");
      SERIAL_PORT_MONITOR.println(creds.ssid);
      SERIAL_PORT_MONITOR.print("and your password: ");
      SERIAL_PORT_MONITOR.println(creds.password);
      SERIAL_PORT_MONITOR.println("have been saved. Thank you!");

      client.stop();
      println("client disconnected");
      
      WiFi.end();
      println("wifi ended");

      xSemaphoreGive( semB);
      xSemaphoreTake( semF, portMAX_DELAY );
    } else {
      client.stop();
      println("client disconnected");
    }
    
  }
  xSemaphoreGive( semF );
}



//*****************************************************************
// Create a thread that checks wifi module operation
// this task will delete itself after confirming module functionality
//*****************************************************************
static void checkWifiModuleThread( void *pvParameters )
{
  println("Checking WiFi Module: Started");
  while (  true ) {
    xSemaphoreTake( semA, portMAX_DELAY );
    if (PROGRAM_STATE != PROGRAM_STATE_WIFI) {
      vSemaphoreDelete(semA);
      xSemaphoreGive(sem_WIFI_CLEANUP);
      println("Checking WiFi Module: Deleted");
      vTaskDelete( NULL );
    }



    myDelayMs(1000);
    checkWifiModule();
    println("WiFi Module good");


    xSemaphoreGive( semB );
  }
}


void checkWifiModule() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  println("WiFi Module operational, continuing..");
  flush();

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    println("Please upgrade the firmware");
  }
}



//*****************************************************************
// Create a thread that checks wifi credentials presence
// this task will delete itself after finding credentials
//*****************************************************************
static void checkWifiCredentialsThread( void *pvParameters )
{
  boolean succ = false;
  println("Checking WiFi Credentials: Thread Started");
  while ( true ) {
    xSemaphoreTake( semB, portMAX_DELAY );
    if (PROGRAM_STATE != PROGRAM_STATE_WIFI) {
      vSemaphoreDelete(semB);
      xSemaphoreGive(sem_WIFI_CLEANUP);
      println("Checking WiFi Credentials: Thread Deleted");
      vTaskDelete( NULL );
    }


    myDelayMs(1000); // every 1 second
    checkWifiCredentials();
  }
}

void checkWifiCredentials() {
  creds = my_flash_store.read();
  if (creds.valid == false) {
    println("CREDS NOT FOUND");
    flush();
    xSemaphoreGive( semC );
  } else {
    xSemaphoreGive( semD );
  }
}


//*****************************************************************
// Create a thread that
// this task will delete itself
//*****************************************************************
static void startNormalOperationThread( void *pvParameters )
{
  println("Starting");
  xSemaphoreTake( UDP_semC, portMAX_DELAY );

  NORM_semA = xSemaphoreCreateBinary();
  NORM_semB = xSemaphoreCreateBinary();

  if (NORM_semA == NULL || NORM_semB == NULL ) {
    println("Not enough freertos heap to create semaphore");
  }

  xTaskCreate(readSensorsThread,     "Normal Operation",       256, NULL, tskIDLE_PRIORITY + 2, &Handle_ReadSensorsTask);
  xTaskCreate(transmitReadingsThread,     "Normal Operation",       256, NULL, tskIDLE_PRIORITY + 2, &Handle_TransmitReadingsTask);

  xSemaphoreGive( NORM_semA );

  println("Start Normal Operation Thread finished");
  vTaskDelete( NULL );
}

static void endNormalOperationThread( void *pvParameters )
{
  println("Ending normal operation");

  xSemaphoreGive( NORM_semA );
  xSemaphoreGive( NORM_semB );

  println("Normal Operation ended ");
  vTaskDelete( NULL );
}




//*****************************************************************
// Create a thread that connects to wifi using credentials
// this task will delete itself TODO when?
//*****************************************************************
static void startWiFiThread( void *pvParameters )
{
  println("Starting");
  semA = xSemaphoreCreateBinary();
  semB = xSemaphoreCreateBinary();
  semC = xSemaphoreCreateBinary();
  semD = xSemaphoreCreateBinary();
  semF = xSemaphoreCreateBinary();

  if (semA == NULL || semB == NULL || semC == NULL || semD == NULL || semF == NULL ) {
    println("Not enough freertos heap to create semaphore");
  }


  xTaskCreate(checkWifiModuleThread,     "Check wifi module",       128, NULL, tskIDLE_PRIORITY + 2, &Handle_WifiModuleCheckTask);
  xTaskCreate(checkWifiCredentialsThread, "Check wifi creds",        128, NULL, tskIDLE_PRIORITY + 2, &Handle_WifiCredCheckTask);
  xTaskCreate(openAccessPointThread,     "open access point",       128, NULL, tskIDLE_PRIORITY + 1, &Handle_APModeTask);
  xTaskCreate(handleAccessPointClientsThread,     "open access point",       256, NULL, tskIDLE_PRIORITY + 1, &Handle_APClientTask);
  xTaskCreate(connectWifiThread,         "connect to wifi",         256, NULL, tskIDLE_PRIORITY + 1, &Handle_ConnectWiFiTask);  //needs 256 of stack

  xSemaphoreGive( semA );

  println("Start WiFi Thread finished");
  vTaskDelete( NULL );
}

static void endWiFiThread( void *pvParameters )
{
  println("Ending WiFi threads");

  xSemaphoreGive( semA );
  xSemaphoreGive( semB );
  xSemaphoreGive( semC );
  xSemaphoreGive( semD );
  xSemaphoreGive( semF );

  println("WiFi thread ended");
  vTaskDelete( NULL );
}




//*****************************************************************
// Create a thread that connects to wifi using credentials
// this task will delete itself TODO when?
//*****************************************************************
static void startUDPThread( void *pvParameters )
{
  println("Starting UDP Thread");
  UDP_semA = xSemaphoreCreateBinary();
  UDP_semB = xSemaphoreCreateBinary();
  UDP_semC = xSemaphoreCreateBinary();

  if (UDP_semA == NULL || UDP_semB == NULL || UDP_semC == NULL  ) {
    println("Not enough freertos heap to create semaphore");
  }


  xTaskCreate(findLocalNodeThread,     "find local node Thread",       256, NULL, tskIDLE_PRIORITY + 5, &Handle_findLocalNodeTask);
  xTaskCreate(checkUDPTargetThread,     "find local node Thread",       256, NULL, tskIDLE_PRIORITY + 5, &Handle_UDPTargetCheckTask);

  xSemaphoreGive( UDP_semA );

  println("Start UDP Thread finished");
  vTaskDelete( NULL );
}

static void endUDPThread( void *pvParameters )
{
  println("Ending UDP Thread");

  xSemaphoreGive( UDP_semA );
  xSemaphoreGive( UDP_semB );
  xSemaphoreGive( UDP_semC );

  println("UDP thread ended");
  vTaskDelete( NULL );
}




//*****************************************************************
// Create a thread that handles program flow
// this task will delete itself TODO when?
//*****************************************************************

/*
   Handles program flow in while loop.
   Control should loop around only in case something goes wrong in normal operation

   start LED control -> setWifi
   setWifi -> setUPD
   setUDP -> normal operation
   normal operation -> setWifi
*/
static void programFlowThread( void *pvParameters )
{
  println("Starting Program Execution");
  //  myDelayMs(500);


  while (true) {
    myDelayMs(500);

    //'OS' LED control thread
    xTaskCreate(
      LEDPatternThread
      , "Start LED Thread"
      , 512
      , NULL
      , tskIDLE_PRIORITY + 11
      , &Handle_LEDPatternTask);



    /* start  */ SET_STATE(PROGRAM_STATE_WIFI);
    sem_WIFI_COMPLETE = xSemaphoreCreateBinary();
    if ( sem_WIFI_COMPLETE == NULL ) {
      println("Not enough freertos heap to create semaphore");
    }
    /* create */ xTaskCreate(
      startWiFiThread
      , "Start Thread"
      , 256
      , NULL
      , tskIDLE_PRIORITY + 10
      , &Handle_StartWiFiTask);
    /* begin  */
    xSemaphoreTake( sem_WIFI_COMPLETE, portMAX_DELAY ); //wait for completion
    /* end    */ SET_STATE(PROGRAM_STATE_CLEANUP);
    sem_WIFI_CLEANUP = xSemaphoreCreateCounting(5, 0);
    if ( sem_WIFI_COMPLETE == NULL ) {
      println("Not enough freertos heap to create wifi cleanup counting semaphore fda");
    }
    xTaskCreate(endWiFiThread, "Start Thread", 256, NULL, tskIDLE_PRIORITY + 10, &Handle_EndWiFiTask);
    int wifitasks = 5;
    while (wifitasks > 0) {
      xSemaphoreTake( sem_WIFI_CLEANUP, portMAX_DELAY ); //wait for completion
      wifitasks --;
    }
    println("a lsfn ");
    vSemaphoreDelete(sem_WIFI_COMPLETE);
    println("foiua");
    vSemaphoreDelete(sem_WIFI_CLEANUP);

    println("WiFi threads successfully cleaned up");


    /* start  */ SET_STATE(PROGRAM_STATE_UDP);
    sem_UDP_COMPLETE = xSemaphoreCreateBinary();
    if ( sem_UDP_COMPLETE  == NULL ) {
      println("Not enough freertos heap to create semaphore");
    }
    /* create */ xTaskCreate(
      startUDPThread
      , "Start Thread"
      , 256
      , NULL
      , tskIDLE_PRIORITY + 10
      , &Handle_StartUDPTask);
    /* begin  */
    xSemaphoreTake( sem_UDP_COMPLETE, portMAX_DELAY ); //wait for completion
    /* end    */ SET_STATE(PROGRAM_STATE_CLEANUP);
    xTaskCreate(endUDPThread, "Start Thread", 256, NULL, tskIDLE_PRIORITY + 10, &Handle_EndUDPTask);



    /* start  */ SET_STATE(PROGRAM_STATE_NORMAL);
    sem_NORMAL_COMPLETE = xSemaphoreCreateBinary();
    if ( sem_NORMAL_COMPLETE  == NULL ) {
      println("Not enough freertos heap to create semaphore");
    }
    /* create */ xTaskCreate(
      startNormalOperationThread
      , "Normal Operation"
      , 256
      , NULL
      , tskIDLE_PRIORITY + 2
      , &Handle_StartNormalOperationTask);
    /* begin  */
    xSemaphoreTake( sem_NORMAL_COMPLETE, portMAX_DELAY ); //wait for completion
    /* end    */ SET_STATE(PROGRAM_STATE_CLEANUP);
    xTaskCreate(endNormalOperationThread, "Start Thread", 256, NULL, tskIDLE_PRIORITY + 10, &Handle_EndUDPTask);

  }
}



//*****************************************************************
// Create a thread that controls LED blinking patterns
// this task will not delete, part of the 'OS'
//*****************************************************************


static void LEDPatternThread( void *pvParameters )
{
  println("Starting LED pattern thread");
  while (true) {
    switch (PROGRAM_STATE) {
      case PROGRAM_STATE_CLEANUP:
        LEDPattern_HIGH();
        break;
      case PROGRAM_STATE_WIFI:
        LEDPattern_fastDoubleBlink();
        break;
      case PROGRAM_STATE_UDP:
        LEDPattern_fastBlink();
        break;
      case PROGRAM_STATE_NORMAL:
        LEDPattern_slowBlink();
        break;
    }
  }
}


void LEDPattern_fastBlink() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  myDelayMs(200);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  myDelayMs(200);
}

void LEDPattern_slowDoubleBlink() {
  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  myDelayMs(500);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  myDelayMs(500);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  myDelayMs(500);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  myDelayMs(500);
  myDelayMs(5000);
}

void LEDPattern_slowBlink() {
  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  myDelayMs(200);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  myDelayMs(5000);
}

void LEDPattern_fastDoubleBlink() {
  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  myDelayMs(400);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  myDelayMs(400);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  myDelayMs(400);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  myDelayMs(400);
  myDelayMs(1000);
}

void LEDPattern_HIGH() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

void LEDPattern_LOW() {
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
}






//*****************************************************************
// Task will periodically print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
// Run time stats are generated from all task timing collected since startup
// No easy way yet to clear the run time stats yet
//*****************************************************************
static char ptrTaskList[400]; //temporary string buffer for task stats

void taskMonitor(void *pvParameters)
{
  int x;
  int measurement;

  println("Task Monitor: Started");

  // run this task afew times before exiting forever
  while (1)
  {
    myDelayMs(10000); // print every 10 seconds

    flush();
    println("");
    println("****************************************************");
    print("Free Heap: ");
    print(xPortGetFreeHeapSize());
    println(" bytes");

    print("Min Heap: ");
    print(xPortGetMinimumEverFreeHeapSize());
    println(" bytes");
    flush();

    println("****************************************************");
    println("Task            ABS             %Util");
    println("****************************************************");

    vTaskGetRunTimeStats(ptrTaskList); //save stats to char array
    println(ptrTaskList); //prints out already formatted stats
    flush();

    println("****************************************************");
    println("Task            State   Prio    Stack   Num     Core" );
    println("****************************************************");

    vTaskList(ptrTaskList); //save stats to char array
    println(ptrTaskList); //prints out already formatted stats
    flush();

    println("****************************************************");
    println("[Stacks Free Bytes Remaining] ");





    measurement = uxTaskGetStackHighWaterMark( Handle_ProgramFlowTask );
    print("Handle_ProgramFlowTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_LEDPatternTask );
    print("Handle_LEDPatternTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_StartWiFiTask );
    print("Handle_StartWiFiTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_EndWiFiTask );
    print("Handle_EndWiFiTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_WifiModuleCheckTask );
    print("Handle_WifiModuleCheckTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_WifiCredCheckTask );
    print("Handle_WifiCredCheckTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_APModeTask );
    print("Handle_APModeTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_APClientTask );
    print("Handle_APClientTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_ConnectWiFiTask );
    print("Handle_ConnectWiFiTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
    print("Handle_monitorTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_StartUDPTask );
    print("Handle_StartUDPTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_EndUDPTask );
    print("Handle_EndUDPTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_findLocalNodeTask );
    print("Handle_findLocalNodeTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_UDPTargetCheckTask );
    print("Handle_UDPTargetCheckTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_StartNormalOperationTask );
    print("Handle_StartNormalOperationTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_ReadSensorsTask );
    print("Handle_ReadSensorsTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_TransmitReadingsTask );
    print("Handle_TransmitReadingsTask: ");
    println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
    print("Monitor Stack: ");
    println(measurement);

    println("****************************************************");
    flush();

  }

  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  println("Task Monitor: Deleting");
  vTaskDelete( NULL );

}


//*****************************************************************








void setup() {

  SERIAL.begin(115200);

  delay(1000); // prevents usb driver crash on startup, do not omit this
  //  while (!SERIAL) ;  // Wait for serial terminal to open port before starting program

  println("");
  println("******************************");
  println("        Program start         ");
  println("******************************");
  flush();

  pinMode(LED_BUILTIN, OUTPUT);

  // Set the led the rtos will blink when we have a fatal rtos error
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. d
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object. Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack! Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

  // sets the serial port to print errors to when the rtos crashes
  // if this is not set, serial information is not printed by default
  vSetErrorSerial(&SERIAL);

  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks


  //  //TODO: figure out why taskmonitor causes crashes. probably too much activity at serial buffer ??
  //  xTaskCreate(
  //    taskMonitor
  //    , "Task Monitor"
  //    , 512
  //    , NULL
  //    , tskIDLE_PRIORITY + 1
  //    , &Handle_monitorTask);


  xTaskCreate(
    programFlowThread
    , "Program Flow Thread"
    , 128
    , NULL
    , tskIDLE_PRIORITY + 10
    , &Handle_ProgramFlowTask);





  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  // should never get here
  while (1)
  {
    println("Scheduler Failed! \n");
    flush();
    delay(1000);
  }
}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop() {}
