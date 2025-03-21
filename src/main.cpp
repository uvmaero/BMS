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
#include <LTC6812.h>
#include <LTC681x.h>
#include <SPI.h>
#include <Wire.h>
#include <iomanip>
#include <soc/rtc.h>
#include <sstream>
#include <vector>

#include "TemperatureMessages.h"
#include "data_types.h"
#include "driver/twai.h"
#include "rtc.h"
#include "tables.h"

/*
===============================================================================================
                                    Definitions
===============================================================================================
*/
// Input and output pins on ESP32
// Vspi pins for pack 0
#define MOSI0 23 // GPIO23  MOSI
#define MISO0 19 // GPIO19  MISO
#define SCLK0 18 // GPIO18
#define CS10 5 // GPIO5

// Vspi pins for pack 1
#define MOSI1 13 // GPIO13
#define MISO1 12 // GPIO12
#define SCLK1 14 // GPIO14
#define CS11 15 // GPIO15

// SPIClass * vspi = NULL;

// 0 for rhs, 1 for lhs
#define SIDE 0

// CAN Pins
#define TXD 1 // CAN_MISO
#define RXD 3 // CAN_MOSI

#define DATALOG_ENABLED 1

/*
// FreeRTOS tasks
#define TWAI_READ_REFRESH_RATE                                                 \
    1 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define TWAI_WRITE_REFRESH_RATE                                                \
    8 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define TEMPERATURE_READ_REFRESH_RATE                                          \
    9 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define PACK_READ_REFRESH_RATE                                                 \
    9 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define SERIAL_WRITE_REFRESH_RATE                                              \
    500 // measured in ticks (RTOS ticks interrupt at 1 kHz)


#define TASK_STACK_SIZE 20000 // in bytes
*/

#define SERIAL_DEBUG Serial

#define I2C_CONN Wire

// TWAI
#define TWAI_RX_PIN 42
#define TWAI_TX_PIN 41

/*
===============================================================================================
                                    Configuration
===============================================================================================
*/
// Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000;
//!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000;
//!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)
/*************************************************************************
 Set configuration register. Refer to the data sheet
**************************************************************************/
bool REFON = true;
// Reference Powered Up Bit (Remain powered until watchdog timeout)
bool ADCOPT = false; // ADC Mode option bit 0 = (27kHz, 7kHz, 422 Hz, or 26 Hz)
bool GPIOBITS_A[5] = {false, false, true, true, true};
// GPIO Pin Control Pins 1,2,3,4,5
bool GPIOBITS_B[4] = {false, false, false, false};
// GPIO Pin Control Pins 6,7,8,9

uint16_t UV = UV_THRESHOLD; // Under voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD; // Over voltage Comparison Voltage

bool DCCBITS_A[12] = {false, false, false, false, false, false,
                      false, false, false, false, false, false};
// Discharge cell switch 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7] = {false, false, false, false};
//!< Discharge cell switch  0,13,14,15
bool DCTOBITS[4] = {true, false, true, false};
//!< Discharge time value  0,1,2,3  // Programed for 4 min

/*Ensure that Dcto bits are set according to the required discharge time. Refer
 * to the data sheet */
bool FDRF = false; // Force Digital Redundancy Failure Bit
bool DTMEN = true; // Enable Discharge Timer Monitor
bool PSBits[2] = {false, false}; //!< Digital Redundancy Path Selection//ps-0,1


/*
===============================================================================================
                                  Global Variables
===============================================================================================
*/

struct cell_status {
    uint8_t side{};

    struct CellData {
        const uint8_t total_ic = TOTAL_IC; // number of ic's in daisy chain
    } cellData;

    // Voltages
    struct VoltageStatus {
        cell_asic BMS_IC[TOTAL_IC];
        uint64_t voltageStamp = 0;
    } voltageStatus;

    // Temperature
    struct TemperatureStatus {
        std::vector<float> cell[TOTAL_IC]{};
    } temperatureStatus;
};

cell_status cellStatus0;
cell_status cellStatus1;
cell_status *activeCell;

std::vector<cell_temp> activeTemp{};

const uint8_t total_ic = 1;
cell_asic BMS_IC[total_ic];

// This controls which battery pack is being read through SPI
// A value of -1 means none is active
int8_t activeSPI = -1;
/*
// This controls whether the ADC conversion is considered "finished"
cell_read_status adcStatus = NOTSTARTED;
// This controls whether Temp reading is done
cell_read_status tempStatus = NOTSTARTED;

bool voltageDataAvailable = false;

// Mutex
SemaphoreHandle_t xMutex = nullptr;

// Hardware Timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// RTOS Task Handles
TaskHandle_t xHandleVoltageRead = nullptr;
TaskHandle_t xHandleTempRead = nullptr;

TaskHandle_t xHandleSerialWrite = nullptr;

TaskHandle_t xHandleTWAIRead = nullptr;
TaskHandle_t xHandleTWAIWrite = nullptr;
*/

// TWAI
static const twai_general_config_t can_general_config =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,
                                (gpio_num_t)TWAI_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t can_timing_config =
    TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t can_filter_config =
    TWAI_FILTER_CONFIG_ACCEPT_ALL();

/*
===============================================================================================
                                    Function Declarations
===============================================================================================
*/
/*
[[noreturn]] void packReadTask(void *pvParameters);

[[noreturn]] void serialWriteTask(void *pvParameters);

[[noreturn]] void TWAIReadTask(void *pvParameters);
[[noreturn]] void TWAIWriteTask(void *pvParameters);

// helpers
*/

// These functions will read the cells
void readVoltage();
void readTemperature();
// These will read the twai
void TWAIWrite();
void TWAIRead();

// Write for debugging
void serialWrite();

// String TaskStateToString(eTaskState state);
String msToMSms(uint64_t ms);

// function to convert the base voltage values into readable temperatures
void convertTemps();

// function to initialize or swap which battery SPI is connected to
void switchSPI();

void print_cells(uint8_t);
void print_wrconfig();
void serial_print_hex(uint8_t);

/*
===============================================================================================
                                            Setup
===============================================================================================
*/

void setup() {
    // --------------------- initialize serial connection ------------------- //
    SERIAL_DEBUG.begin(9600);
    SERIAL_DEBUG.printf("\n\n|--- STARTING SETUP ---|\n\n");
    // ----------------------------------------------------------------------

    cellStatus0.side = 0;
    cellStatus1.side = 1;

    SPI.begin();
    pinMode(CS10, OUTPUT);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    /*** Spi Initialization ***/
    // switchSPI();
    SERIAL_DEBUG.printf("SPI initialized \n");
    SERIAL_DEBUG.printf("test\n");

    /*** LTC6812 Initializations ***/
    // initialize configuration registers
    LTC6812_init_cfg(total_ic, BMS_IC);
    LTC6812_init_cfgb(total_ic, BMS_IC);
    SERIAL_DEBUG.printf("Init Done\n");
    // set registers for each IC
    // set registers for each IC
    LTC6812_set_cfgr(1, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS,
                     UV, OV);
    LTC6812_set_cfgrb(1, BMS_IC, FDRF, DTMEN, PSBits, GPIOBITS_B, DCCBITS_B);

    LTC6812_reset_crc_count(total_ic, BMS_IC);
    LTC6812_init_reg_limits(total_ic, BMS_IC);

    SERIAL_DEBUG.printf("configuration completed\n");

    bool twaiActive = false;
    // install TWAI driver
    /*
    if (twai_driver_install(&can_general_config, &can_timing_config,
                            &can_filter_config) == ESP_OK) {
        SERIAL_DEBUG.printf("TWAI DRIVER INSTALL [ SUCCESS ]\n");

        // start TWAI bus
        if (twai_start() == ESP_OK) {
            SERIAL_DEBUG.printf("TWAI INIT [ SUCCESS ]\n");
            twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);

            twaiActive = true;
        }
        else {
            SERIAL_DEBUG.printf("TWAI INIT [ FAILED ]\n");
        }
    }
    else {
        SERIAL_DEBUG.printf("TWAI DRIVER INSTALL [ FAILED ]\n");
    }
*/
    SERIAL_DEBUG.printf("\n\n|--- END SETUP ---|\n\n");
    // ----------------------------------------------------------------------------------------
}

/*
===============================================================================================
                                    Main Loop
===============================================================================================
*/

void loop() {
    wakeup_sleep(total_ic);
    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) {
        LTC6812_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A,
                         DCCBITS_A, DCTOBITS, UV, OV);
        LTC6812_set_cfgrb(current_ic, BMS_IC, FDRF, DTMEN, PSBits, GPIOBITS_B,
                          DCCBITS_B);
    }
    wakeup_idle(total_ic);
    LTC6812_wrcfg(total_ic, BMS_IC);
    LTC6812_wrcfgb(total_ic, BMS_IC);

    readVoltage();
    readTemperature();
    serialWrite();
    // switchSPI();
}

/*
===============================================================================================
                                  Task Functions
===============================================================================================
*/

void readVoltage() {
    SERIAL_DEBUG.printf("Reading Voltage\n");

    wakeup_sleep(total_ic);

    // start ADC voltage conversion
    // normal operation, discharge disabled, all cell channels
    LTC6812_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
    LTC6812_pollAdc();

    const uint8_t pec_error = LTC6812_rdcv(REG_ALL, total_ic, BMS_IC);
    if (pec_error != 0) {
        SERIAL_DEBUG.printf("VOLTAGE READ ERROR; Code: %d\n", pec_error);
    }
    SERIAL_DEBUG.printf("Finished Reading Voltage\n");
}

void readTemperature() {
    // ------------------ Temperature Read ------------------
    SERIAL_DEBUG.printf("Reading Temp");

    //--------------------- Begin Read ---------------------
    cell_asic temperatures[TOTAL_IC];

    for (int i = 0; i < 7; i++) {
        switch (i) {
        case 0:
            LTC6812_wrcomm(total_ic, &Mux0S1);
            break;
        case 1:
            LTC6812_wrcomm(total_ic, &Mux0S2);
            break;
        case 2:
            LTC6812_wrcomm(total_ic, &Mux0S3);
            break;
        case 3:
            LTC6812_wrcomm(total_ic, &Mux0S4);
            break;
        case 4:
            LTC6812_wrcomm(total_ic, &Mux0S5);
            break;
        case 5:
            LTC6812_wrcomm(total_ic, &Mux0S6);
            break;
        case 6:
            LTC6812_wrcomm(total_ic, &Mux0S7);
            break;
        default:
            break;
        }

        LTC6812_stcomm(4);

        LTC6812_adax(MD_7KHZ_3KHZ, GPIO_NUM_8);
        LTC6812_pollAdc();
        const uint8_t pec_error =
            LTC6812_rdaux(GPIOTEMP1, total_ic, temperatures);
        if (pec_error != 0) {
            SERIAL_DEBUG.printf("TEMPERATURE READ ERROR; Code: %d\n",
                                pec_error);
        }

        // The rdaux function will start with the first IC and put
        // the reading into aux.a_codes [0], then count up from
        // there here we are looping through the number of IC's and
        // pushing back the end of the line the cells
        SERIAL_DEBUG.printf("\n Please Don't Be this \n");
        for (int j = 0; j < TOTAL_IC; j++) {
            activeCell->temperatureStatus.cell[j].push_back(
                temperatures->aux.a_codes[j]);
        }
        SERIAL_DEBUG.printf("\n It Wasn't this\n");
    }
    for (int i = 0; i < 6; i++) {
        switch (i) {
        case 0:
            LTC6812_wrcomm(total_ic, &Mux1S1);
            break;
        case 1:
            LTC6812_wrcomm(total_ic, &Mux1S2);
            break;
        case 2:
            LTC6812_wrcomm(total_ic, &Mux1S3);
            break;
        case 3:
            LTC6812_wrcomm(total_ic, &Mux1S4);
            break;
        case 4:
            LTC6812_wrcomm(total_ic, &Mux1S5);
            break;
        case 5:
            LTC6812_wrcomm(total_ic, &Mux1S6);
            break;
        default:
            break;
        }
        LTC6812_stcomm(4);


        LTC6812_adax(MD_7KHZ_3KHZ, GPIO_NUM_9);
        LTC6812_pollAdc();
        const uint8_t pec_error =
            LTC6812_rdaux(GPIOTEMP1, total_ic, temperatures);
        if (pec_error != 0) {
            SERIAL_DEBUG.printf("TEMPERATURE READ ERROR; Code: %d\n",
                                pec_error);
        }


        // The rdaux function will start with the first IC and put
        // the reading into aux.a_codes [0], then count up from
        // there here we are looping through the number of IC's and
        // pushing back the end of the line the cells
        SERIAL_DEBUG.printf("\n Please Don't Be this \n");
        for (int j = 0; j < TOTAL_IC; j++) {
            activeCell->temperatureStatus.cell[j].push_back(
                temperatures->aux.a_codes[j]);
            // This might fault depending on what the read back data
            // is It *SHOULD* just read in a 0 on the voltage, but
            // nothing is guaranteed
        }
        SERIAL_DEBUG.printf("\n It Wasn't this\n");
    }
    SERIAL_DEBUG.printf("Finished Reading Temp\n");
    // Convert the temperature voltages into degrees C
    convertTemps();
}

void serialWrite() {
    SERIAL_DEBUG.printf("starting write\n");
    // Build the data frame string
    String dataFrame = "";

    for (int current_ic = 0; current_ic < total_ic; current_ic++) {
        Serial.print(" IC ");
        Serial.print(current_ic + 1, DEC);
        Serial.print(", ");
        for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
            Serial.print(" C");
            Serial.print(i + 1, DEC);
            Serial.print(":");
            Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
            Serial.print(",");
        }
        Serial.println();
    }

    /*
        // Create top separator line
        dataFrame.concat("+----------+-----------+-----------+\n");
        // Get the timestamp
        String timestamp = msToMSms(activeCell->voltageStatus.voltageStamp);

        // Build header row with timestamp in second and third
        // columns
        char headerLine[64];
        snprintf(headerLine, sizeof(headerLine), "|          | %9s | %9s |\n",
                 timestamp.c_str(), "12:34.456");
        dataFrame.concat(headerLine);

        // Add separator line
        dataFrame.concat("+----------+-----------+-----------+\n");
        // Add column headers
        dataFrame.concat("| cell #   | voltage   | temp      |\n");
        // Add separator line
        dataFrame.concat("+----------+-----------+-----------+\n");


        SERIAL_DEBUG.print(dataFrame);

        // TODO: Make this less bad

        int j = 0;
        // Iterate over each IC
        for (int current_ic = 0; current_ic < total_ic; current_ic++) {
            int cell_channels = BMS_IC[0].ic_reg.cell_channels;


            // Iterate over each cell channel
            for (int i = 0; i < cell_channels; i++) {
                // Calculate the global cell number
                int cell_number = current_ic * cell_channels + i + 1;

                // Get the cell voltage and convert it to volts
                float voltage = BMS_IC[current_ic].cells.c_codes[i] * 0.0001;

                float temperature = activeTemp[j].temperature;
                char line[64];
                snprintf(line, sizeof(line), "| %8d | %9.4f | %9.4f |\n",
                         cell_number, voltage, temperature);

                // Add the formatted line to the data frame
                dataFrame.concat(line);
                j++;
            }
            j++;
        }

        // Add bottom separator line
        dataFrame.concat("+----------+-----------+-----------+\n");

        // Print the data frame to the serial port
        SERIAL_DEBUG.println(dataFrame);
        */
}

void TWAIRead() {
    // TODO
}

void TWAIWrite() {
    // TODO
}


/*
===============================================================================================
                                    Helper Functions
===============================================================================================
*/

String TaskStateToString(const eTaskState state) {
    // init
    String stateStr;

    // get state
    switch (state) {
    case eReady:
        stateStr = "RUNNING";
        break;

    case eBlocked:
        stateStr = "BLOCKED";
        break;

    case eSuspended:
        stateStr = "SUSPENDED";
        break;

    case eDeleted:
        stateStr = "DELETED";
        break;

    default:
        stateStr = "ERROR";
        break;
    }

    return stateStr;
}

String msToMSms(uint64_t ms) {
    const uint minutes = ms / 60000;
    ms = ms % 60000;

    const uint seconds = ms / 1000;
    ms = ms % 1000;

    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%02lu:%02lu.%03llu", minutes, seconds,
             ms);

    return {buffer};
}

// This function initializes or switches the SPI connection between the
// two battery packs
void switchSPI() {
    // Ending an inactive connection has no effect
    SERIAL_DEBUG.printf("Finished thing \n");

    if (activeSPI == 0 || activeSPI == 1) {
        SPI.end();
    }
    // If inactive or on pack 1 switch to pack 0
    if (activeSPI == -1 || activeSPI == 1) {
        SPI.begin(SCLK0, MISO0, MOSI0, CS10);
        pinMode(CS10, OUTPUT);
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        SPI.endTransaction();
        SERIAL_DEBUG.printf("Switched SPI to 1\n");
    }
    else if (activeSPI == 0) {
        SPI.begin(SCLK1, MISO1, MOSI1, CS11);
        pinMode(CS11, OUTPUT);
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        SPI.endTransaction();
        SERIAL_DEBUG.printf("Switched SPI to 2\n");
    }
    else {
        // If something went wrong with active spi, set to disabled,
        // flag then recall this function
        SERIAL_DEBUG.printf("ERROR: Invalid activeSPI detected: %d, recovering",
                            activeSPI);
        activeSPI = -1;
        switchSPI();
    }
}

void convertTemps() {
    std::vector<cell_temp> temperatures;

    for (uint8_t i = 0; i < 25; i++) {
        cell_temp t = {i,
                       voltage_to_temperature(
                           activeCell->temperatureStatus.cell[i % 2][i])};
        // We use emplace back to represent that we are making a new object and
        // this temporary one will be deleted
        temperatures.emplace_back(t);
    }

    activeTemp = temperatures;
}


/*!******************************************************************************
 \brief Prints the Configuration Register A data that is going to be
 written to the LTC6812 to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfig(void) {
    int cfg_pec;
    SERIAL_DEBUG.println(F("Written Configuration A Register: "));
    for (int current_ic = 0; current_ic < activeCell->cellData.total_ic;
         current_ic++) {
        SERIAL_DEBUG.print(F("CFGA IC "));
        SERIAL_DEBUG.print(current_ic + 1, DEC);
        for (int i = 0; i < 6; i++) {
            SERIAL_DEBUG.print(F(", 0x"));
            serial_print_hex(
                activeCell->voltageStatus.BMS_IC[current_ic].config.tx_data[i]);
        }
        SERIAL_DEBUG.print(F(", Calculated PEC: 0x"));
        cfg_pec = pec15_calc(
            6, &activeCell->voltageStatus.BMS_IC[current_ic].config.tx_data[0]);
        serial_print_hex((uint8_t)(cfg_pec >> 8));
        SERIAL_DEBUG.print(F(", 0x"));
        serial_print_hex((uint8_t)(cfg_pec));
        SERIAL_DEBUG.println("\n");
    }
}


/*!************************************************************
 \brief Function to print in HEX form
 @return void
*************************************************************/
void serial_print_hex(const uint8_t data) {
    if (data < 16) {
        SERIAL_DEBUG.print("0");
        SERIAL_DEBUG.print((byte)data, HEX);
    }
    else
        SERIAL_DEBUG.print((byte)data, HEX);
}
