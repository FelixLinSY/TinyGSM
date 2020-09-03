#ifndef _CONFIG_H
#define _CONFIG_H

#include <avr/pgmspace.h>

#define DEBUG_MODE true

#define TINY_GSM_MODEM_SIM7020

#define debugSerial Serial

/* Set serial for AT commands (to the module) */
/* Use Hardware Serial on MEGA2560 */
// #define nbiotSerial Serial2

// Set reset pin
#define RESET_PIN 7

/* See all AT commands, if wanted */
// #define DUMP_AT_COMMANDS

// Range to attempt to autobaud
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 9600
// #define GSM_AUTOBAUD_MAX 115200

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

/* ---io.adafruit.com root certificate--- */
const char root_ca[] PROGMEM = {"-----BEGIN CERTIFICATE-----\\r\\n"
                                "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\\r\\n"
                                "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\\r\\n"
                                "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\\r\\n"
                                "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\\r\\n"
                                "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\\r\\n"
                                "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\\r\\n"
                                "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\\r\\n"
                                "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\\r\\n"
                                "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\\r\\n"
                                "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\\r\\n"
                                "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\\r\\n"
                                "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\\r\\n"
                                "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\\r\\n"
                                "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\\r\\n"
                                "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\\r\\n"
                                "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\\r\\n"
                                "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\\r\\n"
                                "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\\r\\n"
                                "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\\r\\n"
                                "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\\r\\n"
                                "-----END CERTIFICATE-----"};

#endif /* _CONFIG_H */