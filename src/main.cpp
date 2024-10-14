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
#include <iomanip>
#include <soc/rtc.h>
#include <sstream>
#include "data_types.h"
#include "rtc.h"

/*
===============================================================================================
                                    Definitions
===============================================================================================
*/
// Input and output pins on ESP32
/// Vspi pins
#define MOSI 23 // GPIO23  MOSI
#define MISO 19 // GPIO19  MISO
#define SCLK 18 // GPIO18
#define CS1 5 // GPIO5
// SPIClass * vspi = NULL;

// CAN Pins
#define TXD 1 // CAN_MISO
#define RXD 3 // CAN_MOSI

#define DATALOG_ENABLED 1

// FreeRTOS tasks
#define TWAI_READ_REFRESH_RATE 1 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define TWAI_WRITE_REFRESH_RATE 8 // measured in ticks (RTOS ticks interrupt at 1 kHz)

#define VOLTAGE_READ_REFRESH_RATE 9 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define TEMPERATURE_READ_REFRESH_RATE 9 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define SERIAL_WRITE_REFRESH_RATE 500 // measured in ticks (RTOS ticks interrupt at 1 kHz)


#define TASK_STACK_SIZE 20000 // in bytes

#define SERIAL_DEBUG Serial

/*
===============================================================================================
                                    Configuration
===============================================================================================
*/
// Under Voltage and Over Voltage Thresholds
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

struct cell_status {
    struct CellData {
        uint8_t total_ic = 0; // number of ic's in daisy chain
    } cellData;

    // Voltages
    struct VoltageStatus {
        cell_asic BMS_IC[2]{};
        uint64_t voltageStamp = 0;
    } voltageStatus;

    // Temperature
    // TODO
} cellStatus;

// This controls whether the ADC conversion is considered "finished"
adc_conv_status adcStatus = NOTSTARTED;
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

/*
===============================================================================================
                                    Function Declarations
===============================================================================================
*/

[[noreturn]] void voltageReadTask(void *pvParameters);
[[noreturn]] void temperatureReadTask(void *pvParameters);

[[noreturn]] void serialWriteTask(void *pvParameters);

[[noreturn]] void TWAIReadTask(void *pvParameters);
[[noreturn]] void TWAIWriteTask(void *pvParameters);

// helpers

String TaskStateToString(eTaskState state);
String msToMSms(uint64_t ms);

// TODO: convert
void print_cells(uint8_t);
void print_wrconfig();
void serial_print_hex(uint8_t);

/*
===============================================================================================
                                            Setup
===============================================================================================
*/

void setup() {
    // ----------------------- initialize serial connection --------------------- //
    SERIAL_DEBUG.begin(9600);
    SERIAL_DEBUG.printf("\n\n|--- STARTING SETUP ---|\n\n");
    // -------------------------------------------------------------------------- //

    /*** Spi Initializations ***/
    // vspi = new SPIClass(VSPI);
    // SPI.begin(SCLK,MISO,MOSI,CS1); //SCLK,MISO,MOSI,CS1     set spi pins for ESP32
    SPI.begin();
    pinMode(CS1, OUTPUT);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    /*** LTC6812 Initializations ***/
    // initialize configuration registers
    LTC6812_init_cfg(cellStatus.cellData.total_ic, cellStatus.voltageStatus.BMS_IC);
    LTC6812_init_cfgb(cellStatus.cellData.total_ic, cellStatus.voltageStatus.BMS_IC);
    // set registers for each IC
    LTC6812_set_cfgr(1, cellStatus.voltageStatus.BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
    LTC6812_set_cfgrb(1, cellStatus.voltageStatus.BMS_IC, FDRF, DTMEN, PSBits, GPIOBITS_B, DCCBITS_B);

    LTC6812_reset_crc_count(cellStatus.cellData.total_ic, cellStatus.voltageStatus.BMS_IC);
    LTC6812_init_reg_limits(cellStatus.cellData.total_ic, cellStatus.voltageStatus.BMS_IC);

    // ------------------------------- Scheduler & Task Status --------------------------------- //
    // init mutex
    xMutex = xSemaphoreCreateMutex();

    if (xMutex != nullptr) {
        // Cell read tasks
        xTaskCreatePinnedToCore(voltageReadTask, "Voltage-Read", TASK_STACK_SIZE, nullptr, tskIDLE_PRIORITY,
                                &xHandleVoltageRead, 0);
        xTaskCreatePinnedToCore(temperatureReadTask, "Temperature-Read", TASK_STACK_SIZE, nullptr, tskIDLE_PRIORITY,
                                &xHandleTempRead, 0);

        // TWAI tasks
        xTaskCreatePinnedToCore(TWAIReadTask, "TWAI-Read", TASK_STACK_SIZE, nullptr, 1, &xHandleTWAIRead, 0);
        xTaskCreatePinnedToCore(TWAIWriteTask, "TWAI-Write", TASK_STACK_SIZE, nullptr, 1, &xHandleTWAIWrite, 1);
        // Debug task
        xTaskCreatePinnedToCore(serialWriteTask, "SERIAL_DEBUG-Write", TASK_STACK_SIZE, nullptr, tskIDLE_PRIORITY,
                                &xHandleSerialWrite, 1);
    }
    else {
        SERIAL_DEBUG.printf("FAILED TO INIT MUTEX!\nHALTING OPERATIONS!");
        // ReSharper disable once CppDFAEndlessLoop
        while (true)
            ;
    }

    SERIAL_DEBUG.printf("\nTask Status:\n");
    // Read
    if (xHandleVoltageRead != nullptr)
        SERIAL_DEBUG.printf("VOLTAGE READ TASK STATUS: %s \n",
                            TaskStateToString(eTaskGetState(xHandleVoltageRead)).c_str());
    else
        SERIAL_DEBUG.printf("VOLTAGE READ TASK STATUS: DISABLED!\n");

    if (xHandleTempRead != nullptr)
        SERIAL_DEBUG.printf("TEMPERATURE READ TASK STATUS: %s \n",
                            TaskStateToString(eTaskGetState(xHandleTempRead)).c_str());
    else
        SERIAL_DEBUG.printf("TEMPERATURE READ TASK STATUS: DISABLED!\n");

    // Serial
    if (xHandleSerialWrite != nullptr)
        SERIAL_DEBUG.printf("SERIAL Write TASK STATUS: %s \n",
                            TaskStateToString(eTaskGetState(xHandleSerialWrite)).c_str());
    else
        SERIAL_DEBUG.printf("SERIAL Write TASK STATUS: DISABLED!\n");

    // TWAI
    if (xHandleTWAIRead != nullptr)
        SERIAL_DEBUG.printf("TWAI READ TASK STATUS: %s \n", TaskStateToString(eTaskGetState(xHandleTWAIRead)).c_str());
    else
        SERIAL_DEBUG.printf("TWAI READ TASK STATUS: DISABLED!\n");

    if (xHandleTWAIWrite != nullptr)
        SERIAL_DEBUG.printf("TWAI WRITE TASK STATUS: %s \n",
                            TaskStateToString(eTaskGetState(xHandleTWAIWrite)).c_str());
    else
        SERIAL_DEBUG.printf("TWAI WRITE TASK STATUS: DISABLED!\n");

    // scheduler status
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        SERIAL_DEBUG.printf("\nScheduler Status: RUNNING\n");

        // clock frequency
        rtc_cpu_freq_config_t clock_config;
        rtc_clk_cpu_freq_get_config(&clock_config);
        SERIAL_DEBUG.printf("CPU Frequency: %dMHz\n", clock_config.freq_mhz);
    }
    else {
        SERIAL_DEBUG.printf("\nScheduler STATUS: FAILED\nHALTING OPERATIONS");
        // ReSharper disable once CppDFAEndlessLoop
        while (true)
            ;
    }
    SERIAL_DEBUG.printf("\n\n|--- END SETUP ---|\n\n");
    // ---------------------------------------------------------------------------------------- //
}

/*
===============================================================================================
                                    Main Loop
===============================================================================================
*/

void loop() {
    // everything is managed by RTOS, so nothing really happens here!
    vTaskDelay(1); // prevent watchdog from getting upset

    // Set and reset the gpio pins(to drive output on gpio pins)
    /***********************************************************************
     Please ensure you have set the GPIO bits according to your requirement
      in the configuration register.( check the global variable GPIOBITS_A )
    ************************************************************************/
    /*
    wakeup_sleep(cellStatus.cellData.total_ic);
    for (uint8_t current_ic = 0; current_ic < cellStatus.cellData.total_ic; current_ic++) {
      LTC6812_set_cfgr(current_ic, cellStatus.voltageStatus.BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV,
    OV); LTC6812_set_cfgrb(current_ic, cellStatus.voltageStatus.BMS_IC, FDRF, DTMEN, PSBits, GPIOBITS_B, DCCBITS_B);
    }
    wakeup_idle(cellStatus.cellData.total_ic);
    LTC6812_wrcfg(cellStatus.cellData.total_ic, cellStatus.voltageStatus.BMS_IC);
    LTC6812_wrcfgb(cellStatus.cellData.total_ic, cellStatus.voltageStatus.BMS_IC);
    print_wrconfig();

    delay(1000);
    */
}

/*
===============================================================================================
                                FreeRTOS Task Functions
===============================================================================================
*/

[[noreturn]] void voltageReadTask(void *pvParameters) {
    uint8_t prevErr = 0;
    for (;;) {
        // Check for mutex availability
        if (xSemaphoreTake(xMutex, 10) == pdTRUE) {
            // If we have data yet to be converted, we don't want to over-write it
            if (adcStatus == NOTSTARTED) {
                // wake up ic
                wakeup_sleep(cellStatus.cellData.total_ic);

                // start ADC voltage conversion
                // normal operation, discharge disabled, all cell channels
                LTC6812_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);

                // record the timestamp when the data was snapshotted and convert to milliseconds
                cellStatus.voltageStatus.voltageStamp = esp_rtc_get_time_us() / 1000;
                // just in case, to avoid duplicate data, ensure that data collection does to try to operate again
                voltageDataAvailable = false;

                // We set the conversion to started
                adcStatus = INPROGRESS;
            }

            if (adcStatus == INPROGRESS) {
                // Check to see if the conversion is done
                const byte result = LTC681x_pladc();
                // If byte is 0xFF then the conversion is not done
                result == 0xFF ? adcStatus = INPROGRESS : adcStatus = COMPLETED;
            }

            if (adcStatus == COMPLETED) {
                const uint8_t pec_error =
                    LTC6812_rdcv(REG_ALL, cellStatus.cellData.total_ic, cellStatus.voltageStatus.BMS_IC);
                if (pec_error != 0) {
                    if (prevErr != pec_error)
                        SERIAL_DEBUG.printf("VOLTAGE READ ERROR; Code: %d\n", pec_error);
                    prevErr = pec_error;
                }
                // We have read the data, conversion is done, redo
                adcStatus = NOTSTARTED;
                // We can read the data, and it won't be undefined
                voltageDataAvailable = true;
            }
            // release mutex
            xSemaphoreGive(xMutex);
        }
        // limit task refresh rate
        vTaskDelay(VOLTAGE_READ_REFRESH_RATE);
    }
}

[[noreturn]] void temperatureReadTask(void *pvParameters) {
    for (;;) {
        // Check for mutex availability
        if (xSemaphoreTake(xMutex, 10) == pdTRUE) {
            // release mutex
            xSemaphoreGive(xMutex);
        }
        // limit task refresh rate
        vTaskDelay(TEMPERATURE_READ_REFRESH_RATE);
    }
}

[[noreturn]] void serialWriteTask(void *pvParameters) {
    for (;;) {
        // Check for mutex availability
        if (xSemaphoreTake(xMutex, 10) == pdTRUE) {
            // Don't try and read data that may be out-of date or undefined
            if (voltageDataAvailable) {
                // Build the data frame string
                String dataFrame = "-----------------------------\n";
                // Add timestamp
                dataFrame.concat("| " + msToMSms(cellStatus.voltageStatus.voltageStamp) + " |\n");
                dataFrame.concat("-----------------------------\n");
                // Add header line
                dataFrame.concat("| cell # | voltage | tempera |\n");

                // Iterate over each IC
                for (int current_ic = 0; current_ic < cellStatus.cellData.total_ic; current_ic++) {
                    int cell_channels = cellStatus.voltageStatus.BMS_IC[0].ic_reg.cell_channels;

                    // Iterate over each cell channel
                    for (int i = 0; i < cell_channels; i++) {
                        // Calculate the global cell number
                        int cell_number = current_ic * cell_channels + i + 1;

                        // Get the cell voltage and convert it to volts
                        float voltage = cellStatus.voltageStatus.BMS_IC[current_ic].cells.c_codes[i] * 0.0001;

                        // Since temperature data is not available yet, we'll use "N/A"
                        // Format the line with fixed-width columns
                        char line[50];
                        snprintf(line, sizeof(line), "| %6d | %7.4f | %7s |\n", cell_number, voltage, "N/A");

                        // Add the formatted line to the data frame
                        dataFrame.concat(line);
                    }
                }
                SERIAL_DEBUG.println(dataFrame);
                voltageDataAvailable = false;
            }
            // release mutex
            xSemaphoreGive(xMutex);
        }
        // limit task refresh rate
        vTaskDelay(SERIAL_WRITE_REFRESH_RATE);
    }
}

[[noreturn]] void TWAIReadTask(void *pvParameters) {
    for (;;) {
        // check for mutex availability
        if (xSemaphoreTake(xMutex, 10) == pdTRUE) {
            // release mutex
            xSemaphoreGive(xMutex);
        }
        // limit task refresh rate
        vTaskDelay(TWAI_READ_REFRESH_RATE);
    }
}

[[noreturn]] void TWAIWriteTask(void *pvParameters) {
    for (;;) {
        // Check for mutex availability
        if (xSemaphoreTake(xMutex, 10) == pdTRUE) {
            // release mutex
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
    std::ostringstream Msms;

    const uint minutes = ms / 60000;
    ms = ms % 60000;

    const uint seconds = ms / 1000;
    ms = ms % 1000;

    Msms << std::setw(2) << minutes << ':' << std::setw(2) << seconds << '.' << std::setw(3) << ms;

    return String(Msms.str().c_str());
}

// TODO: Convert
/*!************************************************************
  \brief Prints cell voltage to the serial port
   @return void
 *************************************************************/
void print_cells(uint8_t datalog_en) {
    for (int current_ic = 0; current_ic < cellStatus.cellData.total_ic; current_ic++) {
        if (datalog_en == 0) {
            SERIAL_DEBUG.print(" IC ");
            SERIAL_DEBUG.print(current_ic + 1, DEC);
            SERIAL_DEBUG.print(", ");
            for (int i = 0; i < cellStatus.voltageStatus.BMS_IC[0].ic_reg.cell_channels; i++) {
                SERIAL_DEBUG.print(" C");
                SERIAL_DEBUG.print(i + 1, DEC);
                SERIAL_DEBUG.print(":");
                SERIAL_DEBUG.print(cellStatus.voltageStatus.BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
                SERIAL_DEBUG.print(",");
            }
            SERIAL_DEBUG.println();
        }
        else {
            SERIAL_DEBUG.print(" Cells, ");
            for (int i = 0; i < cellStatus.voltageStatus.BMS_IC[0].ic_reg.cell_channels; i++) {
                SERIAL_DEBUG.print(cellStatus.voltageStatus.BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
                SERIAL_DEBUG.print(",");
            }
        }
    }
    SERIAL_DEBUG.println("\n");
}


/*!******************************************************************************
 \brief Prints the Configuration Register A data that is going to be written to
 the LTC6812 to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfig(void) {
    int cfg_pec;
    SERIAL_DEBUG.println(F("Written Configuration A Register: "));
    for (int current_ic = 0; current_ic < cellStatus.cellData.total_ic; current_ic++) {
        SERIAL_DEBUG.print(F("CFGA IC "));
        SERIAL_DEBUG.print(current_ic + 1, DEC);
        for (int i = 0; i < 6; i++) {
            SERIAL_DEBUG.print(F(", 0x"));
            serial_print_hex(cellStatus.voltageStatus.BMS_IC[current_ic].config.tx_data[i]);
        }
        SERIAL_DEBUG.print(F(", Calculated PEC: 0x"));
        cfg_pec = pec15_calc(6, &cellStatus.voltageStatus.BMS_IC[current_ic].config.tx_data[0]);
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
void serial_print_hex(uint8_t data) {
    if (data < 16) {
        SERIAL_DEBUG.print("0");
        SERIAL_DEBUG.print((byte)data, HEX);
    }
    else
        SERIAL_DEBUG.print((byte)data, HEX);
}
