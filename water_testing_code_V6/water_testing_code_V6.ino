//Global Statements
#include <SoftwareSerial.h>      //we have to include the SoftwareSerial library, or else we can't use it.    
#include <avr/pgmspace.h>
#include <OneWire.h>    // library for team 4 temp sensor
#include <avr/sleep.h> // needed for power save (idle mode)
#include <String.h>

// include the SD library:
#include <SPI.h>
#include <SD.h>

// EC
#define EC_rx 4 // D4
#define EC_tx 5 // D5
SoftwareSerial EC_serial(EC_rx, EC_tx);

// PH
#define pH_rx 7 // D7                     
#define pH_tx 8 // D8 
SoftwareSerial pH_serial(pH_rx, pH_tx);

//pins

int turbidityPin = A5; // this is A2 for for the damaged bluno nano, A5 for undamaged
int turbidityControlPin = 3; // D3 - control pin stays the same

int DS18S20_Pin = A4; //DS18S20 Signal pin on analog pin 0 for old bluno, A7 for undamaged
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
int validNo[34] = {84, 185, 300, 416, 82, 183, 299, 413, 518, 619, 737, 838, 906, 1003, 1119, 1216, 83, 199, 296, 412, 529, 644, 68, 169, 267, 384, 487, 188, 298, 414, 100, 200, 297, 510}; //lookup table for checking for errors in commands
File collection;

String collectionName = "test.txt";
String sample_timer_file = "config.txt";
long sample_timer = 300;


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
  //temp.reserve(32);

  //initialise bluetooth communication
  Serial.begin(115200); //115200 for bluetooth

  //initialising SD card with clock on CS on pin 10
  if (SD.begin(10)) {
//    if (SD.exists(collectionName)) {
//      //removing the previous test.txt
//      SD.remove(collectionName);
//    }

    // makes a new test file and then closes the handler
    collection = SD.open(collectionName, FILE_WRITE); //making a new test.txt file
    if (collection) {
      collection.close();
    }
  }

//        // This code can be used to create multiple files meaning we dont delete files when the device tunrs on
//        int i = 0;
//        while(SD.exists("test"+i+".txt"){
//          i++;
//          }
//        collectionName = "test"i".txt";
//      // makes a new test file that can be writen to and then closes the handler
//        collection = SD.open(collectionName, FILE_WRITE); //making a new test.txt file
//        if (collection) {
//          collection.close();
//        }



  //set input pins for sensors
  //pinMode(tempPin, INPUT); was used for old temp sensor
  pinMode(turbidityControlPin, OUTPUT);
  pinMode(turbidityPin, INPUT);

  // This is setting up serial for the conductivity breakout board and sensor
  EC_serial.begin(9600);
  delay(1000);
  EC_serial.print("Response,0\r"); // switch of recived command confirmation
  delay(1000);
  EC_serial.print("L,1\r"); // LEDs off, EEPROM will remember afterwards
  delay(1000);
  EC_serial.print("C,0\r"); // Single sample mode

  // This is setting up serial for the pH breakout board and sensor
  pH_serial.begin(38400);
  pH_serial.print("E\r"); // Put pH sensor into Standby
  pH_serial.print("L0\r"); // LEDs off, EEPROM will remember afterwards

  // This is used to give you time from turning it on to putting it in the water
  //delay(5*60*1000); // 5 minute start-up delay

  getTemp(); //run this to remove errors from first time measurements with the temp sensors

  read_sample_timer();  //run this to load timer

  // now for power saving stuff

  PRR = PRR | B10000000;  //disable two wire interface
  //power_adc_disable();

  // changes clock speed to slow
  //clk_speed(256);
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
  //float timeSince = ((currentStateTime - lastTime) / 1000) / 60;
  timeSinceLast = ((currentStateTime - lastTime) / 1000); //in seconds


  /////////////////////////////// Reading State ////////////////////////////////
  if (timeSinceLast > sample_timer) {
    //start enabling devices
    //ADC_off(0); // enable ADC now so it can stabalise before using it

    //now take samples
    temperature = getTemperatureReal();
    turbidity = getTurbidityReal();
    pH = getPHReal(temperature);
    EC = getConductivityReal();

    getSample(temperature, turbidity, pH, EC, timeSinceLast); // Take a sample

    lastTime = currentStateTime;
  }

  /////////////////////////////// Bluetooth Transmition /////////////////////////
  else {
    MainComs(temperature, turbidity, pH, EC);
  }

} //end of loop

/////////////////////////////////Bluetooth transmission///////////////////
void MainComs(double temperature, double turbidity, String pH, String EC) {
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
      //currentTime = millis();
      //timeSince = ((float)currentTime - (float)prevSampleTime) / (float)(1000);
      //prevSampleTime = currentTime; //set the new previous sample time

      Serial.println(F("[databegin] {"));
      Serial.println(F("\"status\": \"complete\",\n"));
      Serial.println(F("\"samples\": ["));

      //reading and sending data to bluetooth
      while (collection.available()) {
        Serial.write(collection.read());
        delay(2);
      }
      collection.close();

      //Serial.print("\n],\n\"TimeSinceLast\":" + String(timeSince) + "\n}[dataend]\n");
      //resetting the counter
      counter = 0;
    }

    //Sending a test sample cmd = Test
    else if (counter == 416) {
      long currentStateTime = millis();
      float timeSince = ((currentStateTime - lastTime) / 1000); //in seconds
      
      temperature = getTemperatureReal();
      turbidity = getTurbidityReal();
      pH = getPHReal(temperature);
      EC = getConductivityReal();

      getSample(temperature, turbidity, pH, EC, timeSince);

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

    //if Debug command is recieved, enter sensor debug mode for direct access to sensor commands from bluetooth
    else if (counter == 487) {
      Serial.println(F("Debug_on"));
      counter = 0;
      pH_EC_debug_mode();
    }
    // method to change sampling time on the fly
    else if (counter == 414) {
      change_sample_timer();
    }
    
    // method to delete file on device if needed
    else if (counter == 510) {
      if (SD.exists(collectionName)) {
        //removing the previous test.txt
        SD.remove(collectionName);
      }
      collection = SD.open(collectionName, FILE_WRITE); //making a new test.txt file
      Serial.print(F("Data Deleted"));
      
    }
    
  }
}


///////////////////////////////// Temperature ////////////////////////////
double getTemperatureReal() {
  double total = 0; // running total
  for (int x = 0; x < 10; x++) {
    total = total + (double)(getTemp()); // getting 10 measurements
  }
  return (total / 10); // returns the average
}

////////////////////////////////temp helper method////////////////////////


float getTemp() {
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
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

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
double getTurbidityReal() {

  double rawTurbidity = 0;
  double fitTurbidity = 0;
  double total = 0;  // the readings from the analog input

  digitalWrite(turbidityControlPin, HIGH); // turning on turbidity sensor
  delay(100); // takes a little while to turn on

  // getting 10 measurements
  for (int x = 0; x < 10; x++) {
    rawTurbidity = analogRead(turbidityPin);
    delay(2);

    // adding a fitting curve
    fitTurbidity = (1 / 500000) * (rawTurbidity * rawTurbidity) - 0.0022 * (rawTurbidity) + 3.7996; //curve fitting

    //total = total + fitTurbidity;
    total = total + rawTurbidity;
    
  }
  digitalWrite(turbidityControlPin, LOW); // turning of sensor

  //turn ADC off since it is no longer needed
  //ADC_off(1);
  return (total / 10.00); // returning average of 10 measurements and curve fitting.
}

/////////////////////////////////// Conductivity ////////////////////////////////

String getConductivityReal() {
  String EC = "";                          //char pointer used in string parsing
  char EC_data[48];                  //we make a 48 byte character array to hold incoming data from the EC.
  byte received_from_EC_sensor = 0;       //we need to know how many characters have been received.
  byte string_received = 0;            //used to identify when we have received a string from the EC circuit.

  // waking up the chip
  EC_serial.listen();

  EC_serial.print("Slepy\r"); // wakes up the device and sends incorrect command (2 char wake it up, more than 2 registers as a command and gets device to return an error, bringing corupt symbols with it
  delay(2000);
  while (EC_serial.available() > 0) {       // catch any corrupt symbols from the chip when waking up from sleep
    EC_serial.read();

  }

  EC_serial.print("R\r"); // tell chip to take a sample
  delay(3000);

  if (EC_serial.available() > 0) {                                       //if we see that the EC Circuit has sent a character.
    received_from_EC_sensor = EC_serial.readBytesUntil(13, EC_data, 48); //we read the data sent from EC Circuit until we see a <CR>. We also count how many character have been received.
    EC_data[received_from_EC_sensor] = 0;                                //we add a 0 to the spot in the array just after the last character we received. This will stop us from transmitting incorrect data that may have been left in the buffer.

    if ((EC_data[0] >= 48) && (EC_data[0] <= 57)) {                      //if ec_data[0] is a digit and not a letter
      EC = strtok(EC_data, ",");
    }
  }
  EC_serial.print("SLEEP\r"); //put device to sleep

  if (String("") == EC) {
    EC = "-1    ";
  }

  return EC;
}

/////////////////////////////// PH ////////////////////////////////
String getPHReal(double temperature) {
  char pH_data[48];                  //we make a 48 byte character array to hold incoming data from the pH.
  byte received_from_pH_sensor = 0;     //we need to know how many characters have been received.

  pH_serial.listen();
  pH_serial.print("R\r"); // telling the device to take a reading
  delay(1000);

  if (pH_serial.available() > 0) {
    received_from_pH_sensor = pH_serial.readBytesUntil(13, pH_data, 48); //we read the data sent from ph Circuit until we see a <CR>. We also count how many character have been received.
    pH_data[received_from_pH_sensor] = 0;
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
  for (int i = 0; i < 34; i++) {
    if (validNo[i] == no) {
      return 1;
    }
  }
  return 0;
}

void getSample(double temperature, double turbidity, String pH, String EC, float timeSince) {

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

  collection.print("{\"DID\":\"");    //DID -> Device ID
  collection.print(String(deviceID));

  temp = F(", \"DID\":\"");           //DID -> Device ID
  temp += String(deviceID);
  overhead += temp;

  //sampleID
  temp = F("\", \"SID\":\"");       // SID -> sample ID
  temp += String(sampleID);
  sampleID++; //increment sample id
  collection.print(temp);
  overhead += temp;

  //timesincelast
  temp = F("\", \"TSL\":\"");       // TSL -> time since last

  if (timeSince == 0) { //first reading
    //currentTime = millis();
    //timeSinceLast = ((float)currentTime - (float)prevSampleTime) / (float)(60000);
    temp += "0";
    //prevSampleTime = currentTime; //set the new previous sample time
    collection.print(temp);
    t += temp;
  }
  else {
    //currentTime = millis();
    //timeSinceLast = ((float)currentTime - (float)prevSampleTime) / (float)(60000);
    temp += (String)timeSince;
    //prevSampleTime = currentTime; //set the new previous sample time

    collection.print(temp);
    t += temp;
  }

  //temperature
  char buf[20];
  dtostrf(temperature, 2, 2, buf);
  temp = F("\", \"Tmp\":\"");         //Tmp -> Tempurature
  temp += (String)buf;
  collection.print(temp);
  Temperature += temp;

  //buf[0] = char(0);

  //turbidity
  dtostrf(turbidity, 2, 2, buf);
  temp = F("\", \"Trb\":\"");       //Trb -> turbidity
  temp += (String)buf;
  collection.print(temp);
  Turbidity += temp;

  //pH
  temp = F("\", \"pH\":\"");
  temp += pH;
  collection.print(temp);
  PH += temp;

  //conductivity

  temp = F("\", \"EC\":\"");    //EC -> Electrical conductivity
  temp += EC;
  temp += "\"}";
  collection.print(temp);
  E_C += temp;
  collection.close();
}

//////////////////////////////////////////// change clock speed /////////////////////////////////////////////////
//void clk_speed(int prescaler) {
//  // CLKPR is 8 bit long
//
//
//  CLKPR =  B10000000;    // Tell the AtMega we want to change the system clock
//
//  CLKPR = CLKPR | B00001000;   // must be an integer value e.g. 1,2,4,8,16,32,64,128,256 //
//}
/////////////////////////////////////////// Turning off some stuff /////////////////////////////////////////////
//void ADC_off(int A) {
//  if (A == 1) {
//    // Disable the ADC by setting the ADEN bit (bit 7)  of the
//    // ADCSRA register to zero.
//    ADCSRA = ADCSRA | B10000000;
//
//    // Disable the analog comparator by setting the ACD bit
//    // (bit 7) of the ACSR register to one.
//    ACSR = ACSR | B10000000;
//
//    PRR = PRR | B00000001; // puts the ADC into power reduction mode after dissabling it
//  }
//  else {
//
//    PRR = PRR & B11111110; // takes the ADC out of power reduction mode before re-enabling it
//    // enable the ADC by setting the ADEN bit (bit 7)  of the
//    // ADCSRA register to one.
//    ADCSRA = ADCSRA & B01111111;
//
//    // ensable the analog comparator by setting the ACD bit
//    // (bit 7) of the ACSR register to zero.
//    ACSR = ACSR & B01111111;
//
//
//  }
//}

/////////////////////////////////////////// IDLE power down method /////////////////////////////////////////////
//void idle(int A) { // A = 1 turn on, A =0, turn off
//  if (A == 1) {
//    // power reduction register (PRR)
//    // Bit 7 - PRTWI: Power Reduction TWI (Two wire interface)
//    // Bit 6 - PRTIM2: Power Reduction Timer/Counter2
//    // Bit 5 - PRTIM0: Power Reduction Timer/Counter0
//    // Bit 4 - Res: Reserved bit
//    // Bit 3 - PRTIM1: Power Reduction Timer/Counter1
//    // Bit 2 - PRSPI: Power Reduction Serial Peripheral Interface
//    // Bit 1 - PRUSART0: Power Reduction USART0
//    // Bit 0 - PRADC: Power Reduction ADC
//    // '1' will turn it off while '0' will turn it back on
//    // 76543210
//    PRR = PRR | 0b00100000; // disables time timmer before entering SLEEP_MODE_IDLE otherwise it will wake up every mill second.
//    set_sleep_mode (SLEEP_MODE_IDLE); // puts arduino in idle
//    sleep_enable();
//    // Put the device to sleep:
//    sleep_mode();
//    ADCSRA = 0;// disable ADC
//    PRR = PRR | 0b11101101; // turns of all but USART0
//  }
//  else {
//    sleep_disable();
//    ADCSRA = 1; // turn on ADC
//    PRR = 0b11110111; // turns on all things we tunred off
//
//  }
//
//}




/////////////////////////////////////////// pH and EC debug modes ///////////////////////////////////////////////


void pH_EC_debug_mode() {

  String inputstring = "";                              //a string to hold incoming data from the PC
  String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
  boolean input_string_complete = false;                //have we received all the data from the PC
  boolean run_debug_mode = true;                        //variable used to break from loop if no longer need to be in debug mode
  SoftwareSerial *sensor_serial = &EC_serial;           // pointer to a SoftwareSerial object allows this method to be used for both EC and pH chips

  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);                           //set aside some bytes for receiving data from Atlas Scientific product


  //Serial.println(F("{\"status\":\"Debug_Starting\"}"));   //Notify entering debug mode
  Serial.println(F("EC_mode"));          //Notify default mode is conductivity chip
  sensor_serial->listen();                                //start listining to default software serial port

  while (run_debug_mode) {                                        //here we go...


    if (Serial.available() > 0) {                     //if we have recieved a character from the bluetooth
      inputstring = Serial.readStringUntil(13);       //read the string until we see a <CR>
      input_string_complete = true;                   //set the flag to true

    }

    if (input_string_complete == true and inputstring == F("exit")) { //check if an exit command was sent
      inputstring = "";
      run_debug_mode = false;               //turn off debug mode
      input_string_complete = false;        // clear input string flag to prevent leftover data being sent to the active software serial port.
    }
    else if (input_string_complete == true and inputstring == F("pH_mode")) {   // switch to pH sensor if "pH_mode" is recieved
      inputstring = "";                     //clear input incase leftover commands are present
      input_string_complete = false;        //clear flag incase leftover commands are present
      sensor_serial = &pH_serial;           //Swap softwareSerial pointer to the pH sensor chip
      Serial.println(F("{\"status\":\"pH_mode\"}"));  //notify of the serial change
      sensor_serial->listen();                        //start listining to the new softwareSerial connection
    }
    else if (input_string_complete == true and inputstring == F("EC_mode")) {
      inputstring = "";                     //clear input incase leftover commands are present
      input_string_complete = false;        //clear flag incase leftover commands are present
      sensor_serial = &EC_serial;           //Swap softwareSerial pointer to the conductivity sensor chip
      Serial.println(F("{\"status\":\"EC_mode\"}")); //notify of the serial change
      sensor_serial->listen();              //start listining to the new softwareSerial connection
    }


    if (input_string_complete) {                        //if a string from the PC has been received in its entirety
      sensor_serial->print(inputstring);                //send that string to the active softwareSerial connection
      sensor_serial->print('\r');                       //add a <CR> to the end of the string
      inputstring = "";                                 //clear the string
      input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the bluetooth

    }

    while (sensor_serial->available() > 0) {              //while we see that the pH or conductivity Sensor (whomever is active at time) has sent a character
      sensorstring = sensor_serial->readStringUntil(13);  // Read until an endline <CR> is recieved
      Serial.println(sensorstring);                     //print the raw characters/data back to the bluetooth
      sensorstring = "";                                //clear the string holding the recieved data from the sensors (might be unnecessary)
    }
  }
  Serial.println(F("Debug_off")); //notify that debug mode is being exited

}




void change_sample_timer() {
  String inputstring;
  boolean input_string_complete = false;

  Serial.println(F("Sready"));

  while (input_string_complete == false) {
    if (Serial.available() > 0) {                     //if we have recieved a character from the bluetooth
      inputstring = Serial.readStringUntil(13);       //read the string until we see a <CR>
      input_string_complete = true;                   //set the flag to true

    }
  }

  //while its bad, the input will not be checked for if it is a string/integer
  SD.remove(sample_timer_file);
  collection = SD.open(sample_timer_file, FILE_WRITE);
  //"config.txt";
  collection.println(inputstring);
  collection.close();
  sample_timer = inputstring.toInt();
  Serial.println(F("Sdone"));
  
}


void read_sample_timer() {
  String inputnumber;

  if (SD.exists(sample_timer_file)) {
    collection = SD.open(sample_timer_file);
    //"config.txt";
    inputnumber = collection.readStringUntil(13);  
    collection.close();
    inputnumber.trim();
    sample_timer = inputnumber.toInt();
  }
  else {

    collection = SD.open(sample_timer_file, FILE_WRITE);
    collection.println("300"); //default is 300 seconds
    collection.close();  
  }
  
}























