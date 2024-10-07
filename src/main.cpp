/*
This program will interact with the LTC6812 dev. board using SPI. It will print
out the cell voltage values to the serial monitor.

See README file for links to libraries, etc.
*/

/*
===============================================================================================
                                    Includes
===============================================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <LTC6812.h>
#include <LTC681x.h>

/*
===============================================================================================
                                    Definitions
===============================================================================================
*/
//Input and output pins on ESP32
/// Vspi pins 
#define MOSI   23  //GPIO23  MOSI
#define MISO   19  //GPIO19  MISO
#define SCLK   18  //GPIO18
#define CS1     5   //GPIO5
//SPIClass * vspi = NULL;

// CAN Pins
#define TXD 1 //CAN_MISO
#define RXD 3 //CAN_MOSI

#define DATALOG_ENABLED 1

//FreeRTOS tasks
#define TWAI_READ_REFRESH_RATE 1                  // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define TWAI_WRITE_REFRESH_RATE 8                 // measured in ticks (RTOS ticks interrupt at 1 kHz)

#define VOLTAGE_READ_REFRESH_RATE 9               // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define TEMPERATURE_READ_REFRESH_RATE 9           // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define SERIAL_WRITE_REFRESH_RATE 10              // measured in ticks (RTOS ticks interrupt at 1 kHz)


#define TASK_STACK_SIZE 20000 // in bytes

/*
===============================================================================================
                                  Global Variables
===============================================================================================
*/

/**
 *  the dataframe that describes the entire state of the battery
 */

/*
===============================================================================================
                                    Configuration
===============================================================================================
*/
//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)
/*************************************************************************
 Set configuration register. Refer to the data sheet
**************************************************************************/
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false, false, true, true, true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool GPIOBITS_B[4] = {false, false, false, false}; //!< GPIO Pin Control // Gpio 6,7,8,9

uint16_t UV = UV_THRESHOLD; //!< Under voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD; //!< Over voltage Comparison Voltage

bool DCCBITS_A[12] = {false, false, false, false, false, false, false, false, false, false, false, false};
//!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7] = {false, false, false, false}; //!< Discharge cell switch //Dcc 0,13,14,15
bool DCTOBITS[4] = {true, false, true, false}; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min

/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = true; //!< Enable Discharge Timer Monitor
bool PSBits[2] = {false, false}; //!< Digital Redundancy Path Selection//ps-0,1

/*
===============================================================================================
                                  Global Variables
===============================================================================================
*/

const uint8_t total_ic = 2; //number of ic's in daisy chain
uint16_t conv_time = 0; //Set to default value
cell_asic BMS_IC[total_ic]; //cell_asic ic_pt; //structure defined in LTC681x.h --> where most data is stored

// Mutex
SemaphoreHandle_t xMutex = NULL;

// Hardware Timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// RTOS Task Handles
TaskHandle_t xHandleVoltageRead = NULL;
TaskHandle_t xHandleTempRead = NULL;

TaskHandle_t xHandleSerialWrite = NULL;

TaskHandle_t xHandleTWAIRead = NULL;
TaskHandle_t xHandleTWAIWrtie = NULL;

/*
===============================================================================================
                                    Function Declarations
===============================================================================================
*/

[[noreturn]] void voltageReadTask(void* pvParameters);
[[noreturn]] void temperatureReadTask(void* pvParameters);

[[noreturn]] void serialWriteTask(void* pvParameters);

[[noreturn]] void TWAIReadTask(void* pvParameters);
[[noreturn]] void TWAIWriteTask(void* pvParameters);

//helpers


//TODO: convert
void read_voltage(void);
void print_cells(uint8_t);
void print_wrconfig(void);
void serial_print_hex(uint8_t);

/*
===============================================================================================
                                            Setup
===============================================================================================
*/

void setup() {
  Serial.begin(9600);
  Serial.println("        ");

  /*** Spi Initializations ***/
  //vspi = new SPIClass(VSPI);
  //SPI.begin(SCLK,MISO,MOSI,CS1); //SCLK,MISO,MOSI,CS1     set spi pins for ESP32
  SPI.begin();
  pinMode(CS1, OUTPUT);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  /*** LTC6812 Initializations ***/
  //initialize configuration registers
  LTC6812_init_cfg(total_ic, BMS_IC);
  LTC6812_init_cfgb(total_ic, BMS_IC);
  //set registers for each IC
  LTC6812_set_cfgr(1, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
  LTC6812_set_cfgrb(1, BMS_IC, FDRF, DTMEN, PSBits, GPIOBITS_B, DCCBITS_B);

  LTC6812_reset_crc_count(total_ic, BMS_IC);
  LTC6812_init_reg_limits(total_ic, BMS_IC);

  //RTOS initialization
}

/*
===============================================================================================
                                    Main Loop
===============================================================================================
*/

void loop() {
  read_voltage();

  // Set and reset the gpio pins(to drive output on gpio pins)
  /***********************************************************************
   Please ensure you have set the GPIO bits according to your requirement
    in the configuration register.( check the global variable GPIOBITS_A )
  ************************************************************************/
  wakeup_sleep(total_ic);
  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) {
    LTC6812_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
    LTC6812_set_cfgrb(current_ic, BMS_IC, FDRF, DTMEN, PSBits, GPIOBITS_B, DCCBITS_B);
  }
  wakeup_idle(total_ic);
  LTC6812_wrcfg(total_ic, BMS_IC);
  LTC6812_wrcfgb(total_ic, BMS_IC);
  print_wrconfig();

  delay(1000);
}

/*
===============================================================================================
                                FreeRTOS Task Functions
===============================================================================================
*/

[[noreturn]] void voltageReadTask(void* pvParameters) {
  for (;;) {
    //Check for mutex availability
    if (xSemaphoreTake(xMutex, 10) == pdTRUE) {
      //wake up ic
      wakeup_sleep(total_ic);

      //start ADC voltage conversion
      LTC6812_adcv(MD_7KHZ_3KHZ,DCP_DISABLED,CELL_CH_ALL); //normal operation, discharge disabled, all cell channels
      //wait for ADC conversion
      conv_time = LTC6812_pollAdc();
      Serial.print("Conversion Time: ");
      Serial.println(conv_time);

      //reads cell voltage
      uint8_t pec_error = LTC6812_rdcv(REG_ALL, total_ic, BMS_IC);
      //read registers, number of ICs, pointer to structure where data will be stored
      if (pec_error == -1) Serial.println("Read Error"); //check for error

      //LINDUINO function to print cell data to serial monitor
      print_cells(DATALOG_ENABLED);
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(VOLTAGE_READ_REFRESH_RATE);
  }
}

[[noreturn]] void temperatureReadTask(void* pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xMutex, 10) == pdTRUE) {
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(TEMPERATURE_READ_REFRESH_RATE);
  }
}

[[noreturn]] void TWAIReadTask(void* pvParameters) {
  for (;;) {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, 10) == pdTRUE) {
      // release mutex
      xSemaphoreGive(xMutex);
    }
    // limit task refresh rate (every other tick [?])
    vTaskDelay(TWAI_READ_REFRESH_RATE);
  }
}

[[noreturn]] void TWAIWriteTask(void* pvParameters) {
  for (;;) {
    //Check for mutex availability
    if (xSemaphoreTake(xMutex, 10) == pdTRUE) {
      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit task refresh rate
    vTaskDelay(TWAI_WRITE_REFRESH_RATE);
  }
}

/*
===============================================================================================
                                    Helper Functions
===============================================================================================
*/

/***Basic function to read cell voltages***/
void read_voltage() {
  //wake up ic
  wakeup_sleep(total_ic);

  //start ADC voltage conversion
  LTC6812_adcv(MD_7KHZ_3KHZ,DCP_DISABLED,CELL_CH_ALL); //normal operation, discharge disabled, all cell channels
  //wait for ADC conversion
  conv_time = LTC6812_pollAdc();
  Serial.print("Conversion Time: ");
  Serial.println(conv_time);

  //reads cell voltage
  uint8_t pec_error = LTC6812_rdcv(REG_ALL, total_ic, BMS_IC);
  //read registers, number of ICs, pointer to structure where data will be stored
  if (pec_error == -1) Serial.println("Read Error"); //check for error

  //LINDUINO function to print cell data to serial monitor
  print_cells(DATALOG_ENABLED);
}


/*!************************************************************
  \brief Prints cell voltage to the serial port
   @return void
 *************************************************************/
void print_cells(uint8_t datalog_en) {
  for (int current_ic = 0; current_ic < total_ic; current_ic++) {
    if (datalog_en == 0) {
      Serial.print(" IC ");
      Serial.print(current_ic + 1,DEC);
      Serial.print(", ");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        Serial.print(" C");
        Serial.print(i + 1,DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.println();
    }
    else {
      Serial.print(" Cells, ");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}


/*!******************************************************************************
 \brief Prints the Configuration Register A data that is going to be written to
 the LTC6812 to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfig(void) {
  int cfg_pec;
  Serial.println(F("Written Configuration A Register: "));
  for (int current_ic = 0; current_ic < total_ic; current_ic++) {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic + 1,DEC);
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6, &BMS_IC[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println("\n");
  }
}


/*!************************************************************
 \brief Function to print in HEX form
 @return void
*************************************************************/
void serial_print_hex(uint8_t data) {
  if (data < 16) {
    Serial.print("0");
    Serial.print((byte)data,HEX);
  }
  else
    Serial.print((byte)data,HEX);
}
