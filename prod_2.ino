#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <SoftWire.h>
#include <string.h>
#include <ctype.h>
#include <Stream.h>
#include <stdio.h>
#include <stdlib.h>


//initialize RTC
#define MCP7940N_ADDRESS 0x6F
#define MFP_PIN PA1
SoftWire rtcWire(PB8, PB9, SOFT_STANDARD);

// RTC Time variables
uint8_t rtcSeconds, rtcMinutes, rtcHours, rtcDay, rtcDate, rtcMonth, rtcYear;
char rtcDateTimeStr[64];
bool rtcInitialized = false;

char rtcLogDate[10];  
char rtcLogTime[10];  

///for GPS
char byteGPS;
char linea[300] = "";
char comandoGNRMC[7] = "$GNRMC";
char comandoGNGGA[7] = "$GNGGA";
char comandoGNGLL[7] = "$GNGLL";

int cont=0;
int bien1=0;
int bien2=0;
int bien3=0;
int conta=0;
int indices[13];
int r;

int gnrmc_fix_flag;
int gngga_fix_flag;
int gngll_fix_flag;

bool exit_gnrmc,exit_gngga,exit_gngll;

char logi[12];
char lati[12];
char latdir[3];
char longdir[3];
char fix[3];

char temp_var[12];
int gps_flag;

volatile int lat_pointer=0;
volatile int latdir_pointer=0;
volatile int long_pointer=0;
volatile int longdir_pointer=0;
volatile int fix_pointer=0;

bool gps_exit_flag=0;

char gps_data_string[30];
double latitude_data=0.0;
double longitude_data=0.0;
int any_fix_valid=0;


//TIMER VARIABLES
volatile uint32_t rising_time = 0;
volatile uint32_t falling_time = 0;
volatile uint32_t last_rising_time = 0;
volatile uint32_t pulse_width = 0;
volatile uint32_t period = 0;
volatile bool measurement_ready = false;


const int pwmPin = PB1;



int i=1;


//logging details


#define ENTRY_SIZE 64      
#define BUFFER_SIZE 300    
#define DATA_FILENAME "dlogs2.dat"
#define INDEX_FILENAME "DHEADER.dat"
#define HEADER_FILENAME "VHEADER.dat"


#define SD_CS   PB12    // CS
#define SD_SCK  PB13    // SCK
#define SD_MISO PB14    // MISO
#define SD_MOSI PB15    // MOSI


SPIClass SPI_2(2);  


typedef struct
 {
uint32_t magicData;          
uint32_t currentPosition;     
uint32_t overflowFlag;       
uint32_t totalEntry;   
uint32_t lastEntrytime;  
}BufferIndex;

BufferIndex deviceHeader;
bool bufferInitialized=0;
char *substrings[20];
int total_parsing=0;
char parsed_data[45];

//char array_for_vehicle_header[490];


 char databuff[ENTRY_SIZE];
//STRUCTURE FOR VEHCLE DETAILS
typedef struct {
    char VEHICLE_OWNERS_NAME[30];
    char VEHICLE_OWNER_ID[15];
    char OWNER_PHONE_NO[15];
    char VEHICLE_REG_NUMBER[20];
    char CHASIS_NO[30];
    char VEHICLE_MAKE_TYPE[50];
    char CERTIFICATE_NUMBER[20];
    char LIMITER_TYPE[20];
    char LIMITER_SERIAL[15];
    char DATE_OF_FITTING[15];
    char FITTING_AGENTS_NAME[50];
    char FITING_AGENT_ID[50];
    char NAME_LOCATION_OF_STATION[50];
    char EMAIL_ADDRESS[50];
    char AGENT_PHONE_NUMBER[15];
    char BUSINESS_REG_NO[50];
} header;

 header myVehicleHeader;




//________________________________________________________________________________saveVehicleHeader_START__________________________________________________

bool saveVehicleHeader() {
    File headerFiles;
    
    // Initialize SPI
    SPI.setModule(2);

    if (SD.exists(HEADER_FILENAME)) {
SD.remove(HEADER_FILENAME);
}
    
   
    headerFiles = SD.open(HEADER_FILENAME, FILE_WRITE);
    if (!headerFiles) {
        Serial.println("failed to open the file");
    
        return false;
    }
    else {
        Serial.println("opened the file");
    }
    
    
    Serial.print("Saving header to: ");
    Serial.println(HEADER_FILENAME);
    
    headerFiles.println(myVehicleHeader.VEHICLE_OWNERS_NAME);
    headerFiles.println(myVehicleHeader.VEHICLE_OWNER_ID);
    headerFiles.println(myVehicleHeader.OWNER_PHONE_NO);
    headerFiles.println(myVehicleHeader.VEHICLE_REG_NUMBER);
    headerFiles.println(myVehicleHeader.CHASIS_NO);
    headerFiles.println(myVehicleHeader.VEHICLE_MAKE_TYPE);
    headerFiles.println(myVehicleHeader.CERTIFICATE_NUMBER);
    headerFiles.println(myVehicleHeader.LIMITER_TYPE);
    headerFiles.println(myVehicleHeader.LIMITER_SERIAL);
    headerFiles.println(myVehicleHeader.DATE_OF_FITTING);
    headerFiles.println(myVehicleHeader.FITTING_AGENTS_NAME);
    headerFiles.println(myVehicleHeader.FITING_AGENT_ID);
    headerFiles.println(myVehicleHeader.NAME_LOCATION_OF_STATION);
    headerFiles.println(myVehicleHeader.EMAIL_ADDRESS);
    headerFiles.println(myVehicleHeader.AGENT_PHONE_NUMBER);
    headerFiles.println(myVehicleHeader.BUSINESS_REG_NO);
    
    // Close the file
    headerFiles.flush();
    headerFiles.close();
    
    
    Serial.println("Vehicle header saved successfully");
    
    // Return success
    return true;
}

//________________________________________________________________________________saveVehicleHeader_STOP__________________________________________________


//________________________________________________________________________________get_each_data_START__________________________________________________
void get_each_data(char* string_2_parse,int type)
{ 
int length;
int temp=0;
int flag=0;
length=strlen(string_2_parse);
  
for(int i=0;i<length;i++)
{
  if(type==5)
  {
    if(flag==1)
    {
    parsed_data[temp]=string_2_parse[i];
    temp++;
    }
    if(string_2_parse[i]=='-')
    {
    flag=1;
    temp=0;
    }
  }
  else if(type==9)
  {
    parsed_data[temp]=string_2_parse[i];
    temp++;    
  }
}
parsed_data[temp]='\0';
}



//________________________________________________________________________________get_each_data_STOP__________________________________________________


//________________________________________________________________________________newline_string_parsing_START__________________________________________________
void newline_string_parsing(char* string_2_parse)
{
int temp_data;
int looping=0;
for (looping = 0; looping < 5; looping++)
{
substrings[looping] = 0;
}
char* newline_sep_string = (char*)strtok(string_2_parse,"\n");
    
looping = 0;
total_parsing=0;
while (newline_sep_string != 0 && looping < 50)
{
total_parsing=looping;
if(strstr((char *)newline_sep_string,"END")!=NULL)
{
substrings[looping++] = newline_sep_string;
total_parsing=looping;
break;
}
substrings[looping++] = newline_sep_string;
newline_sep_string = strtok(0,"\n");
}
for(looping=0;looping<5;looping++)
{
get_each_data(substrings[looping],5);
temp_data = atoi(parsed_data);
switch(looping)
{
case 0:
deviceHeader.magicData=temp_data;
break;
case 1:
deviceHeader.currentPosition=temp_data;
break;
case 2:
deviceHeader.overflowFlag=temp_data;
break;
case 3:
deviceHeader.totalEntry=temp_data;
        
break;
case 4:
deviceHeader.lastEntrytime=temp_data;
break;
}
}
}

void parse_vecihle_details(char* string_2_parse)
{
int temp_data;
int looping=0;
for (looping = 0; looping < 17; looping++)
{
substrings[looping] = 0;
}
char* newline_sep_string = (char*)strtok(string_2_parse,"\n");
    
looping = 0;
total_parsing=0;
while (newline_sep_string != 0 && looping < 50)
{
total_parsing=looping;
if(strstr((char *)newline_sep_string,"END")!=NULL)
{
substrings[looping++] = newline_sep_string;
total_parsing=looping;
break;
}
substrings[looping++] = newline_sep_string;
newline_sep_string = strtok(0,"\n");
}
for(looping=0;looping<16;looping++)
{
get_each_data(substrings[looping],9);
Serial.print("Head=");
Serial.println(parsed_data);
//temp_data = atoi(parsed_data);
switch(looping)
{
case 0:
sprintf(parsed_data,myVehicleHeader.VEHICLE_OWNERS_NAME);
break;
case 1:
sprintf(parsed_data,myVehicleHeader.VEHICLE_OWNER_ID);
break;
case 2:
sprintf(parsed_data,myVehicleHeader.OWNER_PHONE_NO);
break;
case 3:
sprintf(parsed_data,myVehicleHeader.VEHICLE_REG_NUMBER);
        
break;
case 4:
sprintf(parsed_data,myVehicleHeader.CHASIS_NO);
break;
case 5:
sprintf(parsed_data,myVehicleHeader.VEHICLE_MAKE_TYPE);
break;
case 6:
sprintf(parsed_data,myVehicleHeader.CERTIFICATE_NUMBER);
break;
case 7:
sprintf(parsed_data,myVehicleHeader.LIMITER_TYPE);
break;
case 8:
sprintf(parsed_data,myVehicleHeader.LIMITER_SERIAL);
        
break;
case 9:
sprintf(parsed_data,myVehicleHeader.DATE_OF_FITTING);
break;
case 10:
sprintf(parsed_data,myVehicleHeader.FITTING_AGENTS_NAME);
break;
case 11:
sprintf(parsed_data,myVehicleHeader.FITING_AGENT_ID);
break;
case 12:
sprintf(parsed_data,myVehicleHeader.NAME_LOCATION_OF_STATION);
break;
case 13:
sprintf(parsed_data,myVehicleHeader.EMAIL_ADDRESS);
break;
case 14:
sprintf(parsed_data,myVehicleHeader.AGENT_PHONE_NUMBER);
break;
case 15:
sprintf(parsed_data,myVehicleHeader.BUSINESS_REG_NO);
break;
}
}
}

//________________________________________________________________________________newline_string_parsing_STOP__________________________________________________


//________________________________________________________________________________writeStruct_START__________________________________________________

bool writeStruct(const char* filename, const BufferIndex& data)
{
char buffer_var[55];
SPI.setModule(2);
  
 
if (SD.exists(filename)) {
SD.remove(filename);
}
  
File dataFile = SD.open(filename, FILE_WRITE);
if (dataFile) 
  {
      sprintf(buffer_var,"magicData-%u\n",data.magicData);
      dataFile.print(buffer_var);
      sprintf(buffer_var,"currentPosition-%u\n",data.currentPosition);
      dataFile.print(buffer_var);
      sprintf(buffer_var,"overflowFlag-%u\n",data.overflowFlag);
      dataFile.print(buffer_var);
      sprintf(buffer_var,"totalEntry-%u\n",data.totalEntry);
      dataFile.print(buffer_var);
      sprintf(buffer_var,"lastEntrytime-%u\n",data.lastEntrytime);
      dataFile.print(buffer_var);      
      dataFile.flush();
      dataFile.close();
      return true;
  }
  else
  {
    Serial.println("Error Write");
    return false;
  }
}


//________________________________________________________________________________writeStruct_STOP__________________________________________________


//________________________________________________________________________________readStruct_START__________________________________________________

bool readStruct(const char* filename, BufferIndex& data)
{ 
  SPI.setModule(2);
  File dataFile = SD.open(filename);
  if (dataFile) 
  {
    long fileSize = dataFile.size();
    char* buffer_var = new char[fileSize + 1];
    dataFile.read(buffer_var, fileSize);
    buffer_var[fileSize] = '\0';
    //Serial.write(buffer_var);
    dataFile.close();
    newline_string_parsing(buffer_var);
    return true;
  } 
  else
  {
    Serial.println("Error Read");
    return false;
  }
}


//________________________________________________________________________________readStruct_STOP__________________________________________________



//________________________________________________________________________________saveIndex_START__________________________________________________
bool saveIndex()
{
  if(writeStruct(INDEX_FILENAME,deviceHeader))
  {
    Serial.println("updated index");
    return true;
  }
  else
  {
    Serial.println("failed to save index");
    return false;
  }
}

//________________________________________________________________________________saveIndex_STOP__________________________________________________



//________________________________________________________________________________loadIndex_START__________________________________________________
bool loadIndex()
{
  if (readStruct(INDEX_FILENAME, deviceHeader)) 
  {
    if(deviceHeader.magicData==0xa1b2c3)
    {
      Serial.print("M: ");
      Serial.println(deviceHeader.magicData,HEX);
      Serial.print("C Pos: ");
      Serial.println(deviceHeader.currentPosition);
      Serial.print("OF: ");
      Serial.println(deviceHeader.overflowFlag);
      Serial.print("TEntry: ");
      Serial.println(deviceHeader.totalEntry);
      Serial.print("OF: ");
      Serial.println(deviceHeader.lastEntrytime);
      return true;
    }
   else
   {
     return false;
   }
  }
  else
  {
    return false;
  }
}

//________________________________________________________________________________loadIndex_STOP__________________________________________________

//________________________________________________________________________________deleteIndex_START__________________________________________________

bool deleteIndex()
{
  if (!SD.remove(INDEX_FILENAME)) {
    Serial.println("Failed to remove file.");
    return false;
  } else {
    Serial.println("File removed successfully!");
    return true;
  }
}

//________________________________________________________________________________deleteIndex_STOP__________________________________________________



//________________________________________________________________________________initializeBuffer_START__________________________________________________

bool initializeBuffer()
{
  File dataFile;
  deviceHeader.magicData=0; 
  deviceHeader.currentPosition=0;  
  deviceHeader.overflowFlag=0;   
  deviceHeader.totalEntry=0; 
  deviceHeader.lastEntrytime=0; 

  if(loadIndex())
  {
    if(deviceHeader.currentPosition>=BUFFER_SIZE)
    {
      Serial.println("Invalid currentPos");
      deviceHeader.currentPosition=0;
    }
  }
  else
  {
    deviceHeader.magicData=0xa1b2c3; 
    deviceHeader.currentPosition=0;  
    deviceHeader.overflowFlag=0;   
    deviceHeader.totalEntry=0; 
    deviceHeader.lastEntrytime=0; 
    deleteIndex();
    saveIndex();
  }
  

  SPI.setModule(2);
  
  if (!SD.exists(DATA_FILENAME)) {
    Serial.println("Creating data file with proper size...");
    dataFile = SD.open(DATA_FILENAME, FILE_WRITE);
    if (dataFile) {
      
      char zeroBuffer[ENTRY_SIZE];
      memset(zeroBuffer, 0, ENTRY_SIZE);
      
      for (int i = 0; i < BUFFER_SIZE; i++) {
        dataFile.write((uint8_t*)zeroBuffer, ENTRY_SIZE);
      }
      
      dataFile.flush();
      dataFile.close();
      Serial.println("Data file created with proper size");
    } else {
      Serial.println("Failed to create data file!");
      return false;
    }
  } else {
  
    dataFile = SD.open(DATA_FILENAME, FILE_READ);
    if (dataFile) {
      uint32_t fileSize = dataFile.size();
      dataFile.close();
      
      Serial.print("Verified file size: ");
      Serial.print(fileSize);
      Serial.print(" bytes (expected ");
      Serial.print(BUFFER_SIZE * ENTRY_SIZE);
      Serial.println(" bytes)");
      
    
      if (fileSize != BUFFER_SIZE * ENTRY_SIZE) {
        Serial.println("File size incorrect, recreating...");
        SD.remove(DATA_FILENAME);
        
        dataFile = SD.open(DATA_FILENAME, FILE_WRITE);
        if (dataFile) {
          char zeroBuffer[ENTRY_SIZE];
          memset(zeroBuffer, 0, ENTRY_SIZE);
          
          for (int i = 0; i < BUFFER_SIZE; i++) {
            dataFile.write((uint8_t*)zeroBuffer, ENTRY_SIZE);
          }
          
          dataFile.flush();
          dataFile.close();
          Serial.println("Data file recreated with proper size");
        } else {
          Serial.println("Failed to recreate data file!");
          return false;
        }
      }
    } else {
      Serial.println("Failed to open data file for size verification!");
      return false;
    }
  }
  
  bufferInitialized = 1;
  return true;  
}


// bool addBufferEntry(const char* entry, uint32_t timestamp) {
//   if (!bufferInitialized) {
//     Serial.println("Buffer not initialized - cannot add entry");
//     return false;
//   }

//   File dataFile;
//   SPI.setModule(2);
  
//   for (int retry = 0; retry < 3; retry++) {

//     dataFile = SD.open(DATA_FILENAME, FILE_WRITE);
//     if (dataFile) break;
//     delay(50); 


//     Serial.print("Retry opening data file: ");
//     Serial.println(retry + 1);
//   }
  
//   if (!dataFile) {
//     Serial.println("Failed to open data file after multiple attempts");
//     return false;
//   }
  
//   uint32_t writePos = deviceHeader.currentPosition * ENTRY_SIZE;
  
 
//   if (!dataFile.seek(writePos)) {
//     Serial.print("Failed to seek to position: ");
//     Serial.println(writePos);
//     dataFile.close();
//     return false;
//   }
  
//   Serial.print("Writing at position: ");
//   Serial.print(deviceHeader.currentPosition);
//   Serial.print(" (byte offset: ");
//   Serial.print(writePos);
//   Serial.println(")");
  
 
//   char entryBuffer[ENTRY_SIZE];
//   memset(entryBuffer, 0, ENTRY_SIZE); 
  
 
//   size_t entryLen = strlen(entry);

//   if (entryLen > ENTRY_SIZE - 1) {

//     entryLen = ENTRY_SIZE - 1; 
//   }

//   memcpy(entryBuffer, entry, entryLen);
//   entryBuffer[entryLen] = '\0'; 
  
//   Serial.print("Entry: ");
//   Serial.println(entryBuffer);
  
  
//   size_t bytesWritten = dataFile.write((uint8_t*)entryBuffer, ENTRY_SIZE);

  
//   dataFile.flush();
//   dataFile.close();
  
 
//   uint32_t nextPosition = (deviceHeader.currentPosition + 1) % BUFFER_SIZE;
  
 
//   if (nextPosition == 0) {
//     deviceHeader.overflowFlag = 1;
//     Serial.println("*** Buffer overflow detected - will start overwriting oldest entries ***");
//   }
  
//   deviceHeader.currentPosition = nextPosition;
  
//   // Update metadata
//   deviceHeader.totalEntry++;
//   deviceHeader.lastEntrytime = timestamp;

//   // Save updated index
//   bool indexSaveSuccess = false;
//   for (int retry = 0; retry < 3; retry++) {

//     if (deleteIndex() && saveIndex()) {
//       indexSaveSuccess = true;
//       break;
//     }

//     delay(50); 
//     Serial.print("Retry saving index: ");
//     Serial.println(retry + 1);
//   }
  
//   if (!indexSaveSuccess) {
//     Serial.println("Warning: Failed to update index after multiple attempts");
    
//   }
  
//   return true;
// }

bool addBufferEntry(const char* entry, uint32_t timestamp) 
{
  File dataFile;
  uint32_t temporary_position=0;
  uint32_t max_size_page=0;
  int flag=0;
  if (!bufferInitialized)
  {
    return false;
  }

  SPI.setModule(2);


  dataFile = SD.open(DATA_FILENAME, O_RDWR );
  if (!dataFile)
  {
    return false;
  }
    // Calculate position and seek
  uint32_t writePos = deviceHeader.currentPosition * ENTRY_SIZE;
  dataFile.seek(writePos);

    Serial.print("POS");
  Serial.println(writePos);

  char entryBuffer[ENTRY_SIZE+2];
  memset(entryBuffer, ' ', ENTRY_SIZE);
  strncpy(entryBuffer, entry, ENTRY_SIZE - 1);
  Serial.println(entryBuffer);
  entryBuffer[64]='\0';

  size_t bytesWritten = dataFile.print(entryBuffer);
  //dataFile.flush();
  dataFile.close();


  // Increment position, with wrap-around when needed
  temporary_position = (deviceHeader.currentPosition + 1);
  if(temporary_position>=BUFFER_SIZE)
  {
    deviceHeader.overflowFlag = 1;
    deviceHeader.currentPosition =0;
    flag=9;
  }
  else
  {
    //   max_size_page = 300*64;
    // if(writePos>=max_size_page)
    // {
    //   //deviceHeader.currentPosition=0;
    // }  
    // else
    // {
    //   deviceHeader.currentPosition = temporary_position;
    // } 
    deviceHeader.currentPosition = temporary_position; 
     flag=9;
  }
  if(flag==9)
  {
    deviceHeader.totalEntry++;
    deviceHeader.lastEntrytime=timestamp;
    flag=0;
    deleteIndex();
    saveIndex();
  }
  return true;
}


bool get_data_logs() {
  int counter_4_newline=0;
  File dataFile;
  SPI.setModule(2);
  
  dataFile = SD.open(DATA_FILENAME, FILE_READ);
  if (!dataFile) {
    Serial.println("Failed to open data file for reading");
    return false;
  }
  
  Serial.println("Reading buffer entries:");
  
  if (deviceHeader.overflowFlag == 1) {
    
    Serial.println("Buffer has overflowed - reading in proper order (oldest to newest):");
    
    
    Serial.println("Oldest entries (from current position to end):");
    for (uint32_t i = deviceHeader.currentPosition; i < BUFFER_SIZE; i++) {
      counter_4_newline++;
      uint32_t readPos = i * ENTRY_SIZE;
      dataFile.seek(readPos);
      
      char entryBuffer[ENTRY_SIZE + 1];
      dataFile.read((uint8_t*)entryBuffer, ENTRY_SIZE);
      entryBuffer[ENTRY_SIZE] = '\0';
      
      Serial.print("Entry ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(entryBuffer);
      Serial2.println(entryBuffer);

    }
    
   // entryBuffer[ENTRY_SIZE + 1]={0};

  } 
        Serial.println("Newest entries (from beginning to current position):");
        counter_4_newline=0;
    for (uint32_t i = 0; i < deviceHeader.currentPosition; i++) {
      counter_4_newline++;
      uint32_t readPos = i * ENTRY_SIZE;
      dataFile.seek(readPos);
      
      char entryBuffer[ENTRY_SIZE + 1];
      dataFile.read((uint8_t*)entryBuffer, ENTRY_SIZE);
      entryBuffer[ENTRY_SIZE] = '\0';
      
      Serial.print("Entry ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(entryBuffer);
      Serial2.println(entryBuffer);

    }

   Serial2.print("\n\n");
  
  dataFile.close();
  return true;
}





//
bool get_each_header() {
  char data_read;
  int counter_4_newline=0;
  File vehicle_header;
  SPI.setModule(2);
  
  vehicle_header = SD.open(HEADER_FILENAME, FILE_READ);
  if (!vehicle_header) {
    Serial.println("Failed to open vheader file for reading");
    return false;
  }
  else
  {
    long fileSize = vehicle_header.size();
    char* buffer_var = new char[fileSize + 1];
    vehicle_header.read(buffer_var, fileSize);
    buffer_var[fileSize] = '\0';
    Serial.print(buffer_var);
    parse_vecihle_details(buffer_var);
  } 
  
  
  vehicle_header.close();
  return true;
}

bool get_header_logs() {
  char data_read;
  int counter_4_newline=0;
  File vehicle_header;
  SPI.setModule(2);
  
  vehicle_header = SD.open(HEADER_FILENAME, FILE_READ);
  if (!vehicle_header) {
    Serial.println("Failed to open vheader file for reading");
    return false;
  }

  while(vehicle_header.available())
  {
    data_read = vehicle_header.read();
    Serial.print(data_read);
    Serial2.print(data_read);
    delay(1);

  }
  Serial2.print("\n");
  


  
  
  vehicle_header.close();
  return true;
}




//##########################################################################



/*


bool readVehicleHeader() {
    File headerFileRead; // Use a distinct name for the File object

    // Initialize SPI (if not already done elsewhere appropriate for your setup)
    SPI.setModule(2);

    headerFileRead = SD.open(HEADER_FILENAME, FILE_READ);
    if (!headerFileRead) {
        Serial.print("Failed to open for reading: ");
        Serial.println(HEADER_FILENAME);
        return false;
    } else {
        Serial.print("Successfully opened for reading: ");
        Serial.println(HEADER_FILENAME);
    }

    // Check if the file has content before trying to read
    if (headerFileRead.available()) {
        // Read each line in the same order it was written
        // Using .trim() to remove any potential '\r' or leading/trailing whitespace
        myVehicleHeader.VEHICLE_OWNERS_NAME = headerFileRead.readStringUntil("\n");
        //myVehicleHeader.VEHICLE_OWNERS_NAME.trim();

        myVehicleHeader.VEHICLE_OWNER_ID = headerFileRead.readStringUntil('\n');
        
        

        myVehicleHeader.OWNER_PHONE_NO = headerFileRead.readStringUntil('\n');
       

        myVehicleHeader.VEHICLE_REG_NUMBER = headerFileRead.readStringUntil('\n');
 

        myVehicleHeader.CHASIS_NO = headerFileRead.readStringUntil('\n');
        

        myVehicleHeader.VEHICLE_MAKE_TYPE = headerFileRead.readStringUntil('\n');
       

        myVehicleHeader.CERTIFICATE_NUMBER = headerFileRead.readStringUntil('\n');
       

        myVehicleHeader.LIMITER_TYPE = headerFileRead.readStringUntil('\n');
       

        myVehicleHeader.LIMITER_SERIAL = headerFileRead.readStringUntil('\n');
       

        myVehicleHeader.DATE_OF_FITTING = headerFileRead.readStringUntil('\n');
      

        myVehicleHeader.FITTING_AGENTS_NAME = headerFileRead.readStringUntil('\n');
        

        myVehicleHeader.FITING_AGENT_ID = headerFileRead.readStringUntil('\n');
       

        myVehicleHeader.NAME_LOCATION_OF_STATION = headerFileRead.readStringUntil('\n');
 

        myVehicleHeader.EMAIL_ADDRESS = headerFileRead.readStringUntil('\n');
       

        myVehicleHeader.AGENT_PHONE_NUMBER = headerFileRead.readStringUntil('\n');
      

        myVehicleHeader.BUSINESS_REG_NO = headerFileRead.readStringUntil('\n');
       

    } else {
        Serial.print("File is empty or unreadable after opening: ");
        Serial.println(HEADER_FILENAME);
        headerFileRead.close();
        return false;
    }

    // Close the file
    headerFileRead.close();

    Serial.println("Vehicle header read successfully.");

    // Optional: You can add a section here to print the loaded values
    // for verification if needed, similar to the previous example.

    return true;
}

*/
//##########################################################################





















void read_file(const char* filename) {
  File dataFile = SD.open(filename);
  if (dataFile) {
    Serial.print("Reading file: ");
    Serial.println(filename);
    Serial.print("File size: ");
    Serial.print(dataFile.size());
    Serial.println(" bytes");
    
    int bytesRead = 0;
    while (dataFile.available()) {
      Serial.write(dataFile.read());
      bytesRead++;
      
      
      if (bytesRead % 64 == 0) {
        Serial.println();
      }
    }
    
    dataFile.close();
    Serial.println("\nDone reading file");
  } else {
    Serial.print("Failed to open file: ");
    Serial.println(filename);
  }
}

//________________________________________________________________________________###########################__________________________________________________________


// --------------------- RTC Functions ---------------------
bool mcp7940n_write_register(uint8_t reg, uint8_t value) {
  rtcWire.beginTransmission(MCP7940N_ADDRESS);
  rtcWire.write(reg);
  rtcWire.write(value);
  return rtcWire.endTransmission() == 0;
}

bool mcp7940n_read_register(uint8_t reg, uint8_t *value) {
  rtcWire.beginTransmission(MCP7940N_ADDRESS);
  rtcWire.write(reg);
  if (rtcWire.endTransmission(false) != 0) return false;
  if (rtcWire.requestFrom(MCP7940N_ADDRESS, 1) != 1) return false;
  *value = rtcWire.read();
  return true;
}

bool mcp7940n_read_time() {
  uint8_t value;
  if (!mcp7940n_read_register(0x00, &value)) return false;
  rtcSeconds = (value & 0x0F) + ((value & 0x70) >> 4) * 10;

  if (!mcp7940n_read_register(0x01, &value)) return false;
  rtcMinutes = (value & 0x0F) + ((value & 0x70) >> 4) * 10;

  if (!mcp7940n_read_register(0x02, &value)) return false;
  rtcHours = (value & 0x0F) + ((value & 0x30) >> 4) * 10;

  if (!mcp7940n_read_register(0x03, &value)) return false;
  rtcDay = value & 0x07;

  if (!mcp7940n_read_register(0x04, &value)) return false;
  rtcDate = (value & 0x0F) + ((value & 0x30) >> 4) * 10;

  if (!mcp7940n_read_register(0x05, &value)) return false;
  rtcMonth = (value & 0x0F) + ((value & 0x10) >> 4) * 10;

  if (!mcp7940n_read_register(0x06, &value)) return false;
  rtcYear = (value & 0x0F) + ((value & 0xF0) >> 4) * 10;

  return true;
}

bool mcp7940n_set_time(uint8_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t min, uint8_t sec) {
  uint8_t sec_bcd = ((sec / 10) << 4) | (sec % 10);
  uint8_t min_bcd = ((min / 10) << 4) | (min % 10);
  uint8_t hour_bcd = ((h / 10) << 4) | (h % 10);
  uint8_t date_bcd = ((d / 10) << 4) | (d % 10);
  uint8_t month_bcd = ((m / 10) << 4) | (m % 10);
  uint8_t year_bcd = ((y / 10) << 4) | (y % 10);
  uint8_t wkday = 1; // Monday (placeholder - would be calculated in full implementation)

  sec_bcd |= 0x80; // Start oscillator

  return
    mcp7940n_write_register(0x00, sec_bcd) &&
    mcp7940n_write_register(0x01, min_bcd) &&
    mcp7940n_write_register(0x02, hour_bcd) &&
    mcp7940n_write_register(0x03, wkday | 0x08) &&
    mcp7940n_write_register(0x04, date_bcd) &&
    mcp7940n_write_register(0x05, month_bcd) &&
    mcp7940n_write_register(0x06, year_bcd);
}

// Initialize the RTC module
bool mcp7940n_init() {
  rtcWire.begin();
  uint8_t dummy;
  return mcp7940n_read_register(0x03, &dummy);
}

// --------------------- Helper Functions ---------------------
// Calculate timestamp from RTC values
uint32_t calculateRtcTimestamp() {
  // Simple timestamp calculation using RTC values
  return ((2000+rtcYear-1970)*365 + rtcMonth*30 + rtcDate) * 86400 + 
         rtcHours*3600 + rtcMinutes*60 + rtcSeconds;
}

double convertToDecimalDegrees(char* location_data, char* hemisphere)
{
  double nmeaCoord = atof(location_data);
  int degrees = (int)(nmeaCoord / 100);
  double minutes = nmeaCoord - (degrees * 100);
  double decimalDegrees = degrees + (minutes / 60.0);

  // Apply sign based on hemisphere
  if (hemisphere[0] == 'S' || hemisphere[0] == 'W')
  {
    decimalDegrees *= -1.0;
  }

  return decimalDegrees;
}

void gps_read()
{
  byteGPS=Serial3.read();  

  if (byteGPS == -1) 
  {           
    // See if the port is empty yet
    delay(10); 
  } 
  else 
  {
     linea[conta]=byteGPS;        // If there is serial port data, it is put in the buffer
     Serial.print(byteGPS); 
     conta++;                      
     if (byteGPS==13)
     {

       cont=0;
       bien1=0;
       bien2=0;
       bien3=0;
       for (int i=1;i<7;i++)
       {
         // Verifies if the received command starts with $GPR
         if (linea[i]==comandoGNRMC[i-1])
         {
           bien1++;
           
         }
         if(gnrmc_fix_flag!=9)
         {
          if (linea[i]==comandoGNGGA[i-1])
          {
            bien2++;
            
          }
          if (linea[i]==comandoGNGLL[i-1])
          {
            bien3++;
            
          }
         }
       }
       if((bien1==6)||(bien2==6)||(bien3==6))
       {               // If yes, continue and process the dat
       if(bien1==6)
       {
        exit_gnrmc=1;
        lat_pointer     =2;
        latdir_pointer  =3;
        long_pointer    =4;
        longdir_pointer =5;
        fix_pointer     =1;
       }
       else if(bien2==6)
       {
        exit_gngga=1;
        lat_pointer     =1;
        latdir_pointer  =2;
        long_pointer    =3;
        longdir_pointer =4;
        fix_pointer     =5;
       }
       else if(bien3==6)
       {
        exit_gngll=1;
        lat_pointer     =0;
        latdir_pointer  =1;
        long_pointer    =2;
        longdir_pointer =3;
        fix_pointer     =5;
       }
       for (int i=0;i<300;i++){
       if (linea[i]==','){    // check for the position of the  "," separator
       indices[cont]=i;
       cont++;
       }
       if (linea[i]=='*'){    // ... and the "*"
       indices[12]=i;
       cont++;
       }
       }
       
      for (int i=0;i<7;i++)
      {
        r=0;
        for (int j=indices[i];j<(indices[i+1]-1);j++)
        {
        gps_flag=0;
        temp_var[r]=linea[j+1];
        r=r+1;
        }
        temp_var[r]='\0';
        

        if(i==lat_pointer)
        {
          sprintf(lati,"%s",temp_var);
          Serial.print("lat="); 
          Serial.print(i); Serial.print(","); 
          Serial.println(temp_var); 
        }
        else if(i==latdir_pointer)
        {
          sprintf(latdir,"%s",temp_var);
          Serial.print("latdir="); Serial.print(i); Serial.print(","); 
          Serial.println(temp_var); 
        }
        else if(i==long_pointer)
        {
          sprintf(logi,"%s",temp_var);
          Serial.print("long="); Serial.print(i); Serial.print(","); 
          Serial.println(temp_var); 
        }
        else if(i==longdir_pointer)
        {
          sprintf(longdir,"%s",temp_var);
          Serial.print("longdir="); Serial.print(i); Serial.print(","); 
          Serial.println(temp_var); 
        }
        else if(i==fix_pointer)
        {
          sprintf(fix,"%s",temp_var);
          Serial.print("fix="); Serial.print(i); Serial.print(","); 
          Serial.println(temp_var); 
          if(bien1==6)
          {
            if(strcmp(fix,"A")==0)
            {
              gnrmc_fix_flag=9;
            }
            else
            {
              gnrmc_fix_flag=0;
            }
          }
          else if(bien2==6)
          {
            if(strcmp(fix,"1")==0)
            {
              gngga_fix_flag=9;
            }
          }
          else if(bien3==6)
          {
            if(strcmp(fix,"A")==0)
            {
              gngll_fix_flag=9;
            }
          }
          if((gngga_fix_flag==9)||(gngll_fix_flag==9)||(gnrmc_fix_flag==9))
          {
            Serial.println("GOT FIX");
            any_fix_valid=1;
          }
          else
          {
            sprintf(lati,"SL");
            sprintf(logi,"SL");
          }
        }      
      }
      if(any_fix_valid==1)
      {
        latitude_data = convertToDecimalDegrees((char*)lati, latdir);
        longitude_data = convertToDecimalDegrees((char*)logi, longdir);
        sprintf(gps_data_string,"%f,%f",latitude_data,longitude_data);
      }
      else
      {
        sprintf(gps_data_string,"SL,SL");
      }
      }
      conta=0;  
      for (int i=0;i<300;i++)
      {
      linea[i]=' ';             
      }  
      if((exit_gngga==1)&&(exit_gngll==1)&&(exit_gnrmc==1))  
      {
        gps_exit_flag=0;
      }
    }
  }
}


//___________________________________________________________________speed_ditection_timer________________________________________________________________
void pwmInterrupt() {
  // Get current time
  uint32_t now = micros();
  
  // Check whether it's a rising or falling edge
  if (digitalRead(pwmPin) == HIGH) {
    // Rising edge
    rising_time = now;
    
    // Calculate period if we have a previous rising edge
    if (last_rising_time > 0) {
      period = rising_time - last_rising_time;
    }
    
    // Store for next calculation
    last_rising_time = rising_time;
  } 
  else {
    // Falling edge
    falling_time = now;
    
    // Calculate pulse width if we have a rising edge time
    if (rising_time > 0) {
      pulse_width = falling_time - rising_time;
      
     
      
      if (period > 0) {
        measurement_ready = true;
      }
    }
  }
}





//________________________________________________________________________________###########################__________________________________________________________

// --------------------- Setup ---------------------
void setup() 
{
  
  Serial.begin(115200);
  delay(100); 
 
  Serial2.begin(9600);
 
  Serial3.begin(115200);
  delay(100);

  Serial.println("\nEnhanced GPS Logger with RTC and Circular Buffer");
  Serial.println("Ready.");

 
  Serial.println("Initializing RTC (primary time source)...");
  rtcWire.begin();
  delay(100);  
  
  if (mcp7940n_init()) {
    Serial.println("RTC initialized successfully");
    rtcInitialized = true;

  //  mcp7940n_set_time(25, 5, 19, 15, 17, 0);
    
    // Read current time from RTC
    if (mcp7940n_read_time()) {
      sprintf(rtcDateTimeStr, "20%02d/%02d/%02d %02d:%02d:%02d",
              rtcYear, rtcMonth, rtcDate, rtcHours, rtcMinutes, rtcSeconds);
      Serial.print("RTC time: ");
      Serial.println(rtcDateTimeStr);
      
      // Format RTC time for logging
      sprintf(rtcLogDate, "%02d%02d%02d", rtcDate, rtcMonth, rtcYear);
      sprintf(rtcLogTime, "%02d%02d%02d", rtcHours, rtcMinutes, rtcSeconds);
    } else {
      Serial.println("Failed to read time from RTC");
      // Set default RTC time if it wasn't set before
      //if (mcp7940n_set_time(25, 5, 14, 12, 12, 0)) {
      //  Serial.println("Set default RTC time: 2025-05-14 12:00:00");
     // }
      
      // Read the time we just set
      if (mcp7940n_read_time()) {
        // Format RTC time for logging
        sprintf(rtcLogDate, "%02d%02d%02d", rtcDate, rtcMonth, rtcYear);
        sprintf(rtcLogTime, "%02d%02d%02d", rtcHours, rtcMinutes, rtcSeconds);
      }
    }
  } else {
    Serial.println("Failed RTC - CRITICAL ERROR!");
  }

  // Configure SPI pins for SD card
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH); // Deselect SD card

  // Initialize SPI2 with conservative settings
  SPI.setModule(2);
  SPI_2.begin();
  SPI_2.setClockDivider(SPI_CLOCK_DIV128); 
  SPI_2.setDataMode(SPI_MODE0);
  SPI_2.setBitOrder(MSBFIRST);
  
  digitalWrite(SD_CS, HIGH);
  delay(100);

  // Send dummy clock cycles with CS high
  for(int i = 0; i < 10; i++) {
    SPI_2.transfer(0xFF);
    Serial.print(".");
  }
  Serial.println(" Done");

  delay(100);

  Serial.println("\nInitializing SD card on SPI2...");
  
  // Select SPI2 for SD card operations
  SPI.setModule(2);
  
 
  bool sdInitialized = false;
  for (int retry = 0; retry < 3; retry++) 
  {
    if (SD.begin(SD_CS)) 
    {
      Serial.println("SD card initialization successful!");
      
      // Test file operations
      File dataFile = SD.open("test.txt", FILE_WRITE);
      if (dataFile) {
        Serial.println("\nWriting to test.txt...");
        dataFile.println("Testing SD card with level shifter");
        dataFile.println("Module is working properly!");
        dataFile.close();
        Serial.println("Write successful!");
        
        // Verify we can read the file back
        dataFile = SD.open("test.txt", FILE_READ);
        if (dataFile) {
          Serial.println("Reading from test.txt:");
          while (dataFile.available()) {
            Serial.write(dataFile.read());
          }
          dataFile.close();
          Serial.println("\nRead test successful!");
          sdInitialized = true;
          //break;
        } else {
          Serial.println("Failed to open test.txt for reading!");
          continue;
        }
      } else {
        Serial.println("Failed to open test.txt for writing!");
        continue;
      }
    }
    Serial.print("SD initialization attempt ");
    Serial.print(retry + 1);
    Serial.println(" failed. Retrying...");
    delay(500);
  }

  if (!sdInitialized) {
    Serial.println("Failed to initialize SD card after multiple attempts!");
    while(1); 
  }

  // Initialize circular buffer
  Serial.println("\nInitializing circular buffer...");
  if (initializeBuffer()) {
    Serial.println("Circular buffer initialized successfully");
  } else {
    Serial.println("Failed to initialize circular buffer!");
    while(1); 
  }

 
  Serial.println("\nWriting test entries to buffer...");
 
  int numTestEntries = 450; 
  
 

  // Display buffer stats
  Serial.println("\nBuffer statistics:");
  Serial.print("Current position: ");
  Serial.println(deviceHeader.currentPosition);
  Serial.print("Overflow flag: ");
  Serial.println(deviceHeader.overflowFlag);
  Serial.print("Total entries written: ");
  Serial.println(deviceHeader.totalEntry);
  
  // Read all entries from the buffer
  Serial.println("\nReading all entries from buffer:");
  //get_data_logs();
  gps_exit_flag=1;


























 
  
  
  strcpy(myVehicleHeader.VEHICLE_OWNERS_NAME, "ATHUL");
  strcpy(myVehicleHeader.VEHICLE_OWNER_ID, "A112#");
  strcpy(myVehicleHeader.OWNER_PHONE_NO, "9012345678");
  strcpy(myVehicleHeader.VEHICLE_REG_NUMBER, "KAA123B");
  strcpy(myVehicleHeader.CHASIS_NO, "KA123456789");
  strcpy(myVehicleHeader.VEHICLE_MAKE_TYPE, "BRIO");
  strcpy(myVehicleHeader.CERTIFICATE_NUMBER, "TH1012");
  strcpy(myVehicleHeader.LIMITER_TYPE, "PEDDLE");
  strcpy(myVehicleHeader.LIMITER_SERIAL, "987654");
  strcpy(myVehicleHeader.DATE_OF_FITTING, "20230501");
  strcpy(myVehicleHeader.FITTING_AGENTS_NAME, "THINTURE");
  strcpy(myVehicleHeader.FITING_AGENT_ID, "123");
  strcpy(myVehicleHeader.NAME_LOCATION_OF_STATION, "VIDYARANYAPURA");
  strcpy(myVehicleHeader.EMAIL_ADDRESS, "athul@thinture.com");
  strcpy(myVehicleHeader.AGENT_PHONE_NUMBER, "9876543215");
  strcpy(myVehicleHeader.BUSINESS_REG_NO, "abcd1234567");
  
 
  bool result = saveVehicleHeader();

  if(result)
  Serial.println("sucess_header");
  else
   Serial.println("failed_heder");;





delay(20);
Serial.println("Interrupt-based PWM Measurement on PB1");
  

  pinMode(pwmPin, INPUT_PULLDOWN);
  
  
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);  
  
  
  attachInterrupt(digitalPinToInterrupt(pwmPin), pwmInterrupt, CHANGE);
  
  Serial.println("Setup complete - waiting for signal...");

}

int previous_seconds=0;

void loop()
{
  
char speed_data[8];
  /*
  if (mcp7940n_read_time()) {
    // Format RTC time for display
    sprintf(rtcLogDate, "%02d%02d%02d", rtcDate, rtcMonth, rtcYear);
    sprintf(rtcLogTime, "%02d%02d%02d", rtcHours, rtcMinutes, rtcSeconds);

    Serial.print("\n\nCurrent time: ");
    Serial.print(rtcLogDate);
    Serial.print(" ");
    Serial.println(rtcLogTime);
    //delay(1000);
  }
  */


    if (mcp7940n_read_time()) {
      // Format RTC time for logging
      sprintf(rtcLogDate, "%02d-%02d-%02d", rtcDate, rtcMonth, rtcYear);
      sprintf(rtcLogTime, "%02d:%02d:%02d", rtcHours, rtcMinutes, rtcSeconds);
      


    Serial.print("\n\nCurrent time: ");
    Serial.print(rtcLogDate);
    Serial.print(" ");
    Serial.println(rtcLogTime);


int i=1;
     

//         if (rtcSeconds % 5 == 0) {

// Serial.println("seconds#############################################################################################################");
//           Serial.println(rtcSeconds);

//            // Prepare entry data
//       sprintf(databuff, "test=%d Date:%s Time:%s", i, rtcLogDate, rtcLogTime);
      
//       // Log the entry
//       uint32_t timestamp = calculateRtcTimestamp();
//       bool success = addBufferEntry(databuff, timestamp);
      
//       Serial.print("Entry ");
//       Serial.print(i);
//       Serial.print(": ");
//       Serial.println(success ? "Success" : "Failed");
      
//         }
//       //delay(100);
//     }
  






  gps_exit_flag=1;
  exit_gnrmc=exit_gngga=exit_gngll=0;
  while(gps_exit_flag==1)
  {
    if(Serial3.available()>0)
    {
      gps_read();
      //delayMicroseconds(10);
    }
  }
 
  
 // delay(100);




String speed_v;

   if (measurement_ready) {
   
    digitalWrite(PC13, !digitalRead(PC13));
    
   
    float freq = 1000000.0 / period;  
    float duty = (pulse_width * 100.0) / period;

    speed_v = String(freq, 2);
    //sprintf(speed_data,"%f",freq);
    
    // Print results
    Serial.print("Freq=");
    Serial.print(speed_v);
    Serial.print("Hz - Duty=");
    Serial.print(duty, 1);
    Serial.println("%");
    
    // Debug info
    Serial.print("Period=");
    Serial.print(period);
    Serial.print("us, Pulse=");
    Serial.print(pulse_width);
    Serial.println("us");
   }
    
    // Reset flag
    measurement_ready = false;

           if ((rtcSeconds % 5 == 0) &&(previous_seconds!=rtcSeconds)){//prvi not equal to current second
           previous_seconds=rtcSeconds;

Serial.println("seconds#############################################################################################################");
          Serial.println(rtcSeconds);
 Serial.println("");
           // Prepare entry data
      sprintf(databuff, "%s,%s,%s,S%s", rtcLogDate, rtcLogTime,gps_data_string,speed_v.c_str());
      Serial.println("");
      // Log the entry
      uint32_t timestamp = calculateRtcTimestamp();
      bool success = addBufferEntry(databuff, timestamp);
      
      Serial.print("Entry ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(success ? "Success" : "Failed");
      
        }
      //delay(100);
    delay(500);
    if(Serial.available()>0)
    {
      char x = Serial.read();
      Serial.println(x);
      Serial.println("******************");
      if(x=='G')
      {
        x='0';
        //get_each_header();
        get_header_logs();
        get_data_logs();
       //readVehicleHeader();
      }
    }
  }
}
