//Global Statements
#include <SoftwareSerial.h>      //we have to include the SoftwareSerial library, or else we can't use it.    
#include <avr/pgmspace.h>   
#include <OneWire.h>    // library for team 4 temp sensor


// include the SD library:
#include <SPI.h>
#include <SD.h>

// EC
#define EC_rx 4 // D4
#define EC_tx 5 // D5
SoftwareSerial EC_serial(EC_rx, EC_tx);

// PH
#define pH_rx 8 // D8                     
#define pH_tx 7 // D7 
SoftwareSerial pH_serial(pH_rx, pH_tx); 

//pins
//int tempPin = A1; 
int turbidityPin = A2;
int turbidityControlPin = 3; // D3

int DS18S20_Pin = A0; //DS18S20 Signal pin on analog pin 6
OneWire ds(DS18S20_Pin);  // on digital pin 1

long lastTime = 0;
  
/////////////////////////////////variables for Bluetooth and SD Card///////////////////////////////

int deviceID = 1;
int sampleID = 0;
long prevSampleTime;
float timeSinceLast = 0;
long currentTime;
int firstReading = 1;
int counter = 0;
int validNo[22] = {84, 185, 300, 416, 82, 183, 299, 413, 518, 619, 737, 838, 906, 1003, 1119, 1216, 83, 199, 296, 412, 529, 644}; //lookup table for checking for errors in commands
File collection;
String collectionName = "test.txt";


// String building
// Used for sending values
String temp;
String overhead;
String t;     
String Temperature;
String E_C;  
String PH;
String Turbidity;

void setup() {
    temp.reserve(32);
     
    //initialise bluetooth communication
    Serial.begin(115200); //115200 for bluetooth
  
    //initialising SD card with clock on CS on pin 10
    if (SD.begin(10)) {
      if (SD.exists(collectionName)) {
        //removing the previous test.txt
        SD.remove(collectionName);
      }
      
       // makes a new test file and then closes the handler  
      collection = SD.open(collectionName, FILE_WRITE); //making a new test.txt file
      if (collection) {
        collection.close();
      }
    }

//      // This code can be used to create multiple files meaning we dont delete files when the device tunrs on
//      int i = 0;
//      while(SD.exists("test"+i+".txt"){
//        i++;
//        }
//      collectionName = "test"+i+".txt";
//    // makes a new test file that can be writen to and then closes the handler  
//      collection = SD.open(collectionName, FILE_WRITE); //making a new test.txt file
//      if (collection) {
//        collection.close();
//      }

     
      
    //set input pins for sensors
    //pinMode(tempPin, INPUT); was used for old temp sensor
    pinMode(turbidityControlPin, OUTPUT);
    pinMode(turbidityPin, INPUT);
    
    // This is setting up serial for the conductivity breakout board and sensor
    EC_serial.begin(9600);
    EC_serial.print("L,1\r"); // LEDs off, EEPROM will remember afterwards
    EC_serial.print("C,0\r"); // Single sample mode

    // This is setting up serial for the pH breakout board and sensor
    pH_serial.begin(38400);
    pH_serial.print("E\r"); // Put pH sensor into Standby
    pH_serial.print("L1\r"); // LEDs off, EEPROM will remember afterwards
    
    // This is used to give you time from turning it on to putting it in the water
    //delay(5*60*1000); // 5 minute start-up delay

    
    

} 

/////////////////////////// Loop /////////////////////////

void loop() {
  
  String sample = "";
  double temperature = 0;
  double turbidity = 0;
  String pH = "";
  String EC = ""; 

  // Used to figure out if another sample needs to be taken
  long currentStateTime = millis();
  float timeSince = ((currentStateTime - lastTime)/1000)/60;
  
  
  /////////////////////////////// Reading State ////////////////////////////////
  if(timeSince > 1){    
    temperature = getTemperatureReal();
    turbidity = getTurbidityReal();
    pH = getPHReal(temperature);
    EC = getConductivityReal();
  
    getSample(temperature, turbidity, pH, EC); // Take a sample
    
    lastTime = currentStateTime;
  }
  
  /////////////////////////////// Bluetooth Transmition /////////////////////////
  else{
    MainComs(temperature,turbidity,pH,EC);
    }
   
} //end of loop

/////////////////////////////////Bluetooth transmission///////////////////
void MainComs(double temperature, double turbidity, String pH, String EC){
   //retrieving command from phone
    if (Serial.available() > 0) {
      //reading from serial
      readSerial();
  
      //flag for checking invalid commands
      int containsFlag = contains(counter);
  
      //checking if command is in the look up table
      if (containsFlag == 0) {
        counter = 0;
      }
  
      //Retreiveing stored data cmd = RetrieveData
      else if (counter == 1216) {
        //opening test.txt to read data
        collection = SD.open(collectionName);
  
        //calculating time
        currentTime = millis();
        timeSinceLast = ((float)currentTime - (float)prevSampleTime) / (float)(60000);
        prevSampleTime = currentTime; //set the new previous sample time

        Serial.println(F("[databegin] {"));
        Serial.println(F("\"status\": \"complete\",\n"));
        Serial.println(F("\"samples\": ["));
  
        //reading and sending data to bluetooth
        while (collection.available()) {
          Serial.write(collection.read());
          delay(2);
        }
        collection.close();

        Serial.print("\n],\n\"TimeSinceLast\":" + String(timeSinceLast) + "\n}[dataend]\n");
        //resetting the counter
        counter = 0;
      }
  
      //Sending a test sample cmd = Test
      else if (counter == 416) {  
        temperature = getTemperatureReal();
        turbidity = getTurbidityReal();
        pH = getPHReal(temperature);
        EC = getConductivityReal();
     
        getSample(temperature, turbidity, pH, EC);
   
        Serial.println("[databegin]");

        Serial.print(overhead);
        Serial.print(t);
        Serial.print(Temperature);
        Serial.print(Turbidity);
        Serial.print(PH);
        Serial.print(E_C);

        Serial.println("\n[dataend]");
        
        //reset counter
        counter = 0;
      }
  
      //Sending Status cmd = Status
      else if (counter == 644) {
        Serial.println(F("[databegin]"));
        Serial.println(F("{\"status\":\"ready\"}")); //just send ready status (for testing)
        Serial.println(F("[dataend]"));
        counter = 0;
      }
    }
  }    
  

///////////////////////////////// Temperature ////////////////////////////
double getTemperatureReal() {
  double total = 0; // running total
  for(int x=0; x<10;x++){
    total = total + (double)(getTemp()); // getting 10 measurements
  }
  return (total/10); // returns the average
}

////////////////////////////////temp helper method////////////////////////


float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
}

///////////////////////////////// Turbidity ////////////////////////////
double getTurbidityReal(){
  double rawTurbidity = 0;
  double fitTurbidity = 0;
  double total = 0;  // the readings from the analog input
  
  digitalWrite(turbidityControlPin, HIGH); // turning on turbidity sensor
  delay(100); // takes a little while to turn on

  // getting 10 measurements
  for (int x = 0; x < 10; x++){
    rawTurbidity = analogRead(turbidityPin);
    delay(2);
    
    // adding a fitting curve
    fitTurbidity = (1/500000)*(rawTurbidity * rawTurbidity) - 0.0022*(rawTurbidity) + 3.7996; //curve fitting
    
    total = total + fitTurbidity; 
  }
  digitalWrite(turbidityControlPin, LOW); // turning of sensor
  return (total / 10.00); // returning average of 10 measurements and curve fitting.
}

/////////////////////////////////// Conductivity ////////////////////////////////

String getConductivityReal(){
  String EC = "";                          //char pointer used in string parsing 
  char EC_data[48];                  //we make a 48 byte character array to hold incoming data from the EC.                             
  byte received_from_EC_sensor = 0;       //we need to know how many characters have been received.
  byte string_received = 0;            //used to identify when we have received a string from the EC circuit.

  // waking up the chip
  EC_serial.print("W\r"); // wakes up the device
  delay(5000);
  EC_serial.listen();
  EC_serial.print("R\r"); // telling it to take a singular reading
  delay(5000);
  
  if(EC_serial.available() > 0){        //if we see that the Conductivity Circuit has sent a character.
     received_from_EC_sensor=EC_serial.readBytesUntil(13,EC_data,48); //we read the data sent from EC Circuit until we see a <CR>. We also count how many character have been received.  
     EC_data[received_from_EC_sensor] = 0;  //we add a 0 to the spot in the array just after the last character we received. This will stop us from transmitting incorrect data that may have been left in the buffer. 
     
     if((EC_data[0] >= 48) && (EC_data[0] <= 57)){   //if ec_data[0] is a digit and not a letter
        EC = strtok(EC_data, ",");  
     }   
  }
  EC_serial.print("SLEEP\r"); //put device to sleep
  return EC;
}

  /////////////////////////////// PH ////////////////////////////////
String getPHReal(double temperature){
  char pH_data[48];                  //we make a 48 byte character array to hold incoming data from the pH.                                
  byte received_from_pH_sensor=0;       //we need to know how many characters have been received.          

  pH_serial.listen();
  pH_serial.print("R\r"); // telling the device to take a reading
  delay(1000);
  
  if(pH_serial.available() > 0){
    received_from_pH_sensor=pH_serial.readBytesUntil(13,pH_data,48); //we read the data sent from ph Circuit until we see a <CR>. We also count how many character have been received.  
    pH_data[received_from_pH_sensor]=0; 
  }
  
  pH_serial.print("E\r");
  return (String)pH_data;
}

  
  /////////////////////////////// Helper Methods ////////////////////////////////

//Reading the serial to increment the counter
void readSerial() {
  while (Serial.available() > 0) {
    counter += (int)Serial.read();
  }
}

//checking if the current counter is in the look up table of possible counter states
int contains(int no) {
  for (int i = 0; i < 22; i++) {
    if (validNo[i] == no) {
      return 1;
    }
  }
  return 0;
}

void getSample(double temperature, double turbidity, String pH, String EC) {
  
  temp = F("");
  overhead = F("");
  t = F("");
  Temperature = F("");
  E_C = F("");
  PH = F("");
  Turbidity = F("");
  
  //opening test.txt which contains the backlog of data
  collection = SD.open(collectionName, FILE_WRITE);

  //formatting
  if (firstReading == 1) {
    firstReading = 0;
  }
  else {
    collection.print(",\n");
  }
  
  //status
  temp += F("{\"status\":\"complete\"");
  overhead += temp;

  collection.print("{\"DeviceID\":\"");
  collection.print(String(deviceID));
  
  temp = F(", \"DeviceID\":\"");
  temp += String(deviceID);
  overhead += temp;

  //sampleID
  temp = F("\", \"SampleID\":\"");
  temp += String(sampleID);
  sampleID++; //increment sample id
  collection.print(temp);
  overhead += temp;

  //timesincelast
  temp = F("\", \"TimeSinceLast\":\"");
    
  if (timeSinceLast == 0) { //first reading
    currentTime = millis();
    timeSinceLast = ((float)currentTime - (float)prevSampleTime) / (float)(60000);
    temp += "0";
    prevSampleTime = currentTime; //set the new previous sample time
    collection.print(temp);
    t += temp;
  }
  else {
    currentTime = millis();
    timeSinceLast = ((float)currentTime - (float)prevSampleTime) / (float)(60000);
    temp += (String)timeSinceLast;
    prevSampleTime = currentTime; //set the new previous sample time

    collection.print(temp);
    t += temp;
  }

  //temperature
  char buf[20];
  dtostrf(temperature, 2, 2, buf);
  temp = F("\", \"Temperature\":\"");
  temp += (String)buf;
  collection.print(temp);
  Temperature += temp;

  //buf[0] = char(0);

//turbidity
  dtostrf(turbidity, 2, 2, buf);
  temp = F("\", \"Turbidity\":\"");
  temp += (String)buf;
  collection.print(temp);
  Turbidity += temp;

  //pH
  temp = F("\", \"pH\":\"");
  temp += pH;
  collection.print(temp);
  PH += temp;

  //conductivity

  temp = F("\", \"Conductivity\":\"");
  temp += EC;
  temp += "\"}";
  collection.print(temp); 
  E_C += temp;
  collection.close();
}


