//
// Created by tobedetered on 1/30/25.
//

#ifndef TABLES_H
#define TABLES_H

// This function will take in a voltage value and output the temperature value
// based on the temperature probe's datasheet
inline int8_t voltage_to_temperature(const float voltage) {
    if (voltage >= 2.44) {
        return -40;
    }
    if (voltage >= 2.42) {
        return -35;
    }
    if (voltage >= 2.40) {
        return -30;
    }
    if (voltage >= 2.38) {
        return -25;
    }
    if (voltage >= 2.35) {
        return -20;
    }
    if (voltage >= 2.32) {
        return -15;
    }
    if (voltage >= 2.27) {
        return -10;
    }
    if (voltage >= 2.23) {
        return -5;
    }
    if (voltage >= 2.17) {
        return 0;
    }
    if (voltage >= 2.11) {
        return 5;
    }
    if (voltage >= 2.05) {
        return 10;
    }
    if (voltage >= 1.99) {
        return 15;
    }
    if (voltage >= 1.92) {
        return 20;
    }
    if (voltage >= 1.86) {
        return 25;
    }
    if (voltage >= 1.80) {
        return 30;
    }
    if (voltage >= 1.74) {
        return 35;
    }
    if (voltage >= 1.68) {
        return 40;
    }
    if (voltage >= 1.63) {
        return 45;
    }
    if (voltage >= 1.59) {
        return 50;
    }
    if (voltage >= 1.55) {
        return 55;
    }
    if (voltage >= 1.51) {
        return 60;
    }
    if (voltage >= 1.48) {
        return 65;
    }
    if (voltage >= 1.45) {
        return 70;
    }
    if (voltage >= 1.43) {
        return 75;
    }
    if (voltage >= 1.40) {
        return 80;
    }
    if (voltage >= 1.38) {
        return 85;
    }
    if (voltage >= 1.37) {
        return 90;
    }
    if (voltage >= 1.35) {
        return 95;
    }
    if (voltage >= 1.34) {
        return 100;
    }
    if (voltage >= 1.33) {
        return 105;
    }
    if (voltage >= 1.32) {
        return 110;
    }
    if (voltage >= 1.31) {
        return 115;
    }
    if (voltage >= 1.30) {
        return 120;
    }

    // Voltage is below the minimum expected value.
    // You might choose to return an error code or throw an exception.
    return static_cast<int8_t>(-999); // For example, -999 indicates an error.
}


#endif // TABLES_H
