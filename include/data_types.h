//
// Created by tobedetered on 10/7/24.
//

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <LTC681x.h>
#include <cstdint>

#define TOTAL_IC 1

enum cell_read_status { NOTSTARTED, COMPLETED, INPROGRESS };

struct cell_temp {
    uint8_t cellNum;
    int8_t temperature;
};


#endif // DATA_TYPES_H
