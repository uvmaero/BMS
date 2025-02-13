//
// Created by tobedetered on 1/22/25.
//

#ifndef TEMPERATUREMESSAGES_H
#define TEMPERATUREMESSAGES_H

#include <Arduino.h>
#include <LTC6812.h>
#include <LTC681x.h>
#include "data_types.h"

// Pack Read Definitions
#define ACK 0b00000000
#define START 0b0110
#define STOP 0b0001
#define BLANK 0b0000
#define NOTHING 0b00000000

// Mux 0 Full and final half
#define A0 0b10011010
#define A01 0b10100000

// Mux stacks first half
#define AX0 0b00001001

// Mux 1 Full and final half
#define A1 0b10011110
#define A11 0b1110

#define GPIOTEMP1 8
#define GPIOTEMP2 9

// CELL Defs
#define S10 0b00000000
#define S11 0b00010000
#define S20 0b00000000
#define S21 0b00100000
#define S30 0b00000000
#define S31 0b01000000
#define S40 0b00000000
#define S41 0b10000000
#define S50 0b00000001
#define S51 0b00000000
#define S60 0b00000010
#define S61 0b00000000
#define S70 0b00000100
#define S71 0b00000000
// TODO


cell_asic Mux0S1[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S10),
                         static_cast<uint8_t>(S11 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S10),
                         static_cast<uint8_t>(S11 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif

};
cell_asic Mux0S2[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S20),
                         static_cast<uint8_t>(S21 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S20),
                         static_cast<uint8_t>(S21 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif


};
cell_asic Mux0S3[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S30),
                         static_cast<uint8_t>(S31 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S30),
                         static_cast<uint8_t>(S31 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif

};
cell_asic Mux0S4[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S40),
                         static_cast<uint8_t>(S41 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S40),
                         static_cast<uint8_t>(S41 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};
cell_asic Mux0S5[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S50),
                         static_cast<uint8_t>(S51 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S50),
                         static_cast<uint8_t>(S51 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};
cell_asic Mux0S6[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S60),
                         static_cast<uint8_t>(S61 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S60),
                         static_cast<uint8_t>(S61 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};
cell_asic Mux0S7[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A01 | ACK),
                         static_cast<uint8_t>(BLANK | S70),
                         static_cast<uint8_t>(S71 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S60),
                         static_cast<uint8_t>(S61 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};


cell_asic Mux1S1[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S10),
                         static_cast<uint8_t>(S11 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S10),
                         static_cast<uint8_t>(S11 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};
cell_asic Mux1S2[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S20),
                         static_cast<uint8_t>(S21 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S20),
                         static_cast<uint8_t>(S21 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};

cell_asic Mux1S3[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S30),
                         static_cast<uint8_t>(S31 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S30),
                         static_cast<uint8_t>(S31 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};

cell_asic Mux1S4[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S40),
                         static_cast<uint8_t>(S41 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S40),
                         static_cast<uint8_t>(S41 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};
cell_asic Mux1S5[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S50),
                         static_cast<uint8_t>(S51 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S50),
                         static_cast<uint8_t>(S51 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};

cell_asic Mux1S6[TOTAL_IC] = {
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S60),
                         static_cast<uint8_t>(S61 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}},
#if TOTAL_IC > 1
    {.com = {.rx_data = {static_cast<uint8_t>(START | AX0),
                         static_cast<uint8_t>(A11 | ACK),
                         static_cast<uint8_t>(BLANK | S60),
                         static_cast<uint8_t>(S61 | ACK),
                         static_cast<uint8_t>(STOP | NOTHING)}}}
#endif
};


#endif // TEMPERATUREMESSAGES_H
