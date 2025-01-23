//
// Created by tobedetered on 1/22/25.
//

#ifndef TEMPERATUREMESSAGES_H
#define TEMPERATUREMESSAGES_H

#include <Arduino.h>
#include <LTC6812.h>
#include <LTC681x.h>

// Pack Read Definitions
#define ACK 0x00000000
#define START 0x0110
#define STOP 0x0001
#define BLANK 0x0000
#define NOTHING 0x00000000

// Mux 0 Full and final half
#define A0 0x10011010
#define A01 0x10100000

// Mux stacks first half
#define AX0 0x00001001

// Mux 1 Full and final half
#define A1 0x10011110
#define A11 0x1110

#define GPIOTEMP1 8
#define GPIOTEMP2 9

// CELL Defs
#define S10 0x00000000
#define S11 0x00010000
#define S20 0x00000000
#define S21 0x00100000
#define S30 0x00000000
#define S31 0x01000000
#define S40 0x00000000
#define S41 0x10000000
#define S50 0x00000001
#define S51 0x00000000
#define S60 0x00000010
#define S61 0x00000000
#define S70 0x00000100
#define S71 0x00000000
// TODO


cell_asic *Mux0S1 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A01 | ACK),
                        static_cast<uint8_t>(BLANK | S10), static_cast<uint8_t>(S11 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux0S2 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A01 | ACK),
                        static_cast<uint8_t>(BLANK | S20), static_cast<uint8_t>(S21 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux0S3 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A01 | ACK),
                        static_cast<uint8_t>(BLANK | S30), static_cast<uint8_t>(S31 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux0S4 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A01 | ACK),
                        static_cast<uint8_t>(BLANK | S40), static_cast<uint8_t>(S41 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux0S5 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A01 | ACK),
                        static_cast<uint8_t>(BLANK | S50), static_cast<uint8_t>(S51 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux0S6 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A01 | ACK),
                        static_cast<uint8_t>(BLANK | S60), static_cast<uint8_t>(S61 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};


cell_asic *Mux1S1 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A11 | ACK),
                        static_cast<uint8_t>(BLANK | S10), static_cast<uint8_t>(S11 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux1S2 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A11 | ACK),
                        static_cast<uint8_t>(BLANK | S20), static_cast<uint8_t>(S21 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux1S3 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A11 | ACK),
                        static_cast<uint8_t>(BLANK | S30), static_cast<uint8_t>(S31 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux1S4 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A11 | ACK),
                        static_cast<uint8_t>(BLANK | S40), static_cast<uint8_t>(S41 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux1S5 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A11 | ACK),
                        static_cast<uint8_t>(BLANK | S50), static_cast<uint8_t>(S51 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux1S6 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A11 | ACK),
                        static_cast<uint8_t>(BLANK | S60), static_cast<uint8_t>(S61 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};
cell_asic *Mux1S7 = {
    .com = {.rx_data = {static_cast<uint8_t>(START | AX0), static_cast<uint8_t>(A11 | ACK),
                        static_cast<uint8_t>(BLANK | S60), static_cast<uint8_t>(S61 | ACK),
                        static_cast<uint8_t>(STOP | NOTHING)}}};


#endif // TEMPERATUREMESSAGES_H
