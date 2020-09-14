#ifndef _CONFIG_H
#define _CONFIG_H

#include <avr/pgmspace.h>

#define DEBUG_MODE true

#define TINY_GSM_MODEM_SIM7020

#define debugSerial Serial
#define TINY_GSM_DEBUG debugSerial

/* Set serial for AT commands (to the module) */
#define nbiotSerial Serial2

// Set reset pin
#define RESET_PIN 7

/* See all AT commands, if wanted */
// #define DUMP_AT_COMMANDS

/* ---nbiot config--- */
// #define APN "twm.nbiot"
// #define BAND 28
#define APN "internet.iot"
#define BAND 8

/* ---mqtt config--- */
#define BROKER "io.adafruit.com"

#define TEST_TOPIC "<YOUR TOPIC>"
#define MQTT_USERNAME "<YOUR USERNAME>"
#define MQTT_PASSWORD "<YOUR KEY>"

#endif /* _CONFIG_H */