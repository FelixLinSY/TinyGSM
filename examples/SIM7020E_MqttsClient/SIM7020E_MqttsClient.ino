/**
 * @file       SIM7020E_MqttsClient.ino
 * @author     Zack Huang
 * @license
 * @copyright
 * @date       2019/09/14
 */

#include "config.h"
#include <PubSubClient.h>
#include <TinyGsmClient.h>

const char apn[] = APN;

/* ---mqtt config--- */
const char *broker     = BROKER;
const char *test_topic = TEST_TOPIC;

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(nbiotSerial, debugSerial);
TinyGsm        modem(debugger, RESET_PIN);
#else
TinyGsm modem(nbiotSerial, RESET_PIN);
#endif
TinyGsmClient client(modem, root_ca);
PubSubClient  mqtt(broker, 8883, client);

void mqttCallback(char *topic, byte *payload, unsigned int len);
void mqttConnect(void);
void nbConnect(void);

void setup()
{
    debugSerial.begin(115200);
    nbiotSerial.begin(115200);
#if DEBUG_MODE
    while (!debugSerial)
        ;
#endif
    randomSeed(analogRead(A0));
    nbConnect();

    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(300);
}

void loop()
{
    static unsigned long timer = millis();
    char                 buff[10];

    if (!mqtt.connected()) {
        if (!modem.isNetworkConnected()) {
            nbConnect();
        }
        debugSerial.println(F("=== MQTT NOT CONNECTED ==="));
        mqttConnect();
    }

    if (millis() - timer >= 10000) {
        timer = millis();
        sprintf(buff, "%ld", random(65535));
        debugSerial.print(F("Publish: "));
        debugSerial.println(buff);
        mqtt.publish(TEST_TOPIC, buff);
    }

    mqtt.loop();
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
    debugSerial.print(F("Message arrived ["));
    debugSerial.print(topic);
    debugSerial.print(F("]: "));
    debugSerial.write(payload, len);
    debugSerial.println();
}

void mqttConnect(void)
{
    debugSerial.print(F("Connecting to "));
    debugSerial.print(broker);
    debugSerial.print(F("..."));

    // Connect to MQTT Broker
    String mqttid = ("MQTTID_" + String(random(65536)));
    while (!mqtt.connect(mqttid.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
        debugSerial.println(F(" fail"));
    }
    debugSerial.println(F(" success"));
    mqtt.subscribe(TEST_TOPIC);
}

void nbConnect(void)
{
    debugSerial.println(F("Initializing modem..."));
    while (!modem.init() || !modem.nbiotConnect(apn, BAND)) {
        debugSerial.print(F("."));
    };

    debugSerial.print(F("Waiting for network..."));
    while (!modem.waitForNetwork()) {
        debugSerial.print(F("."));
    }
    debugSerial.println(F(" success"));
}