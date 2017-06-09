/**************************************************************
#
#	Project: LightDNA - Device code of ESP8266
#	Author: Anderson Ignacio da Silva
# Date: 13/04/2016
#	Target: ESP-8266
#	Version: 1.6 - Char nulll fixed
#
**************************************************************/

/**
 * @file user_main.c
 * @author Ânderson Ignacio da Silva
 * @date 8 Jun 2017
 * @brief FW for the LightDNA ESP8266 Light.
 * @version 1.6
 *
 * This code is the mainly file for the ESP8266 firmware for the
 * LightDNA Application that runs embedded on the Light.
 * @see --
 */

#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"

// Macros and typedefs
//#define UART_PEN
#define LED_DEBUG
#define TOPIC_DEFAULT   "lights/%08X"
#define TOPIC_DIMMER    "lights/%08X/dimmer"
#define TOPIC_MSP       "lights/%08X/msp"
#define TOPIC_STATUS    "lights/%08X/status"
#define TOPIC_CURRENT   "lights/%08X/current"
#define TOPIC_PIR       "lights/%08X/pir"
#define TOPIC_LDR       "lights/%08X/ldr"
#define TOPIC_RSSI      "lights/%08X/rssi"
#define TOPIC_IP        "lights/%08X/ip"
#define TOPIC_TEMP      "lights/%08X/temperature"
#define MESSAGE_OFFLINE "Device %08X it's offline!"

#define FW_VERSION_LIGHTDNA "1.6-07/06/2017"
#define GPIO_4(x)   GPIO_OUTPUT_SET(GPIO_ID_PIN(4), x)
#define GPIO_5(x)   GPIO_OUTPUT_SET(GPIO_ID_PIN(5), x)

typedef enum states
{
        FIRST_IDT,
        DATA_B,
        COMPLETE
}states_buffer;


// Global variables
states_buffer serialState = FIRST_IDT;

static ETSTimer gStatusTimerBlink;

bool gBlinkStatus = false;

uint8_t rxPosCharIndex = 0,
        gBufferInput[40],
        gMACSta[6],
        topicDefaultDimmer[64],
        topicDefaultCurrent[64],
        topicDefaultStatus[64],
        topicDefaultPir[64],
        topicDefaultTemp[64],
        topicDefaultRSSI[64],
        topicDefaultIp[64],
        topicDefaultLdr[64],
        topicDefaultMsp[64],
        topicDefault[64],
        MessageDead[64],
        DeviceID[8],
        InitMessage[100],
        StatusMessage[100],
        StatusRSSI[10],
        IPMessage[64];

MQTT_Client mqttClient;

/**
 * @brief Callback for WIFI connection
 *
 * It'll be called when the device connects to a broker MQTT
 * @param args Args used by MQTT structure
 */
static void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status)
{
        if (status == STATION_GOT_IP) {
                MQTT_Connect(&mqttClient);
        } else {
                MQTT_Disconnect(&mqttClient);
        }
}

/**
 * @brief Callback for the MQTT connection when it connects to a broker
 *
 * It'll be called when device connects to a broker MQTT.
 * @param args Args used by MQTT structure
 */
static void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args)
{
        MQTT_Client* client = (MQTT_Client*)args;

        INFO("MQTT: Connected\r\n");
        os_sprintf(InitMessage, "Hello AWGES DEV: %08X MAC: %02X-%02X-%02X-%02X-%02X-%02X", system_get_chip_id(), gMACSta[0], gMACSta[1], gMACSta[2], gMACSta[3], gMACSta[4], gMACSta[5]);
        MQTT_Publish(&mqttClient, topicDefaultStatus, InitMessage, strlen(InitMessage), 0, 0);

        MQTT_Subscribe(client, topicDefault, 1);
        MQTT_Subscribe(client, "lights", 1);

}

/**
 * @brief Callback for the disconnected connection from MQTT broker
 *
 * It'll be called when the connection with the broker is stopped
 * @param args Args used by MQTT structure
 */
static void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args)
{
        MQTT_Client* client = (MQTT_Client*)args;
        INFO("MQTT: Disconnected\r\n");
}

/**
 * @brief Callback for the published data in MQTT
 *
 * It'll be called when finishes to publish some data
 * @param args Args used by MQTT structure
 */
static void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args)
{
        MQTT_Client* client = (MQTT_Client*)args;
        INFO("MQTT: Published\r\n");
}

/**
 * @brief Get the default IPv4 from device in STA mode
 *
 * When the ESP8266 receives the R0253 it'll print the info. about IPv4 Address
 * in the determined topic in the MQTT.
 */
void ICACHE_FLASH_ATTR networkIPGet(void) {
        struct ip_info ipconfig;
        wifi_get_ip_info(STATION_IF, &ipconfig);
        if (wifi_station_get_connect_status() == STATION_GOT_IP && ipconfig.ip.addr != 0) {
                os_printf("IP found\n\r");
                os_sprintf(IPMessage, "IPv4 - " IPSTR, IP2STR(&ipconfig.ip));
                MQTT_Publish(&mqttClient, topicDefaultIp, IPMessage, strlen(IPMessage), 0, 0);
        } else {
                os_printf("No IP found\n\r");
                os_sprintf(IPMessage, "No IPv4 found");
                MQTT_Publish(&mqttClient, topicDefaultIp, IPMessage, strlen(IPMessage), 0, 0);
        }
}

/**
 * @brief Print information about the RSSI of the last message received
 *
 * When the ESP8266 receives the R0254 it'll print the last RSSI value on it's
 * antenna.
 */
void infoRSSI(void){
        sint8 rssiSignal = wifi_station_get_rssi();
        os_sprintf(StatusRSSI, "%d", rssiSignal);
        MQTT_Publish(&mqttClient, topicDefaultRSSI, StatusRSSI, strlen(StatusRSSI), 0, 0);
}

/**
 * @brief Print information about the ESP8266 device
 *
 * When the EPS8266 received the R0255 command from MQTT topics, it'll print the
 * information about ESP8266 firmware data.
 */
void ShowID(void)
{
        os_sprintf(StatusMessage, "OK - %s - DEV: %08X MAC: %02X-%02X-%02X-%02X-%02X-%02X", FW_VERSION_LIGHTDNA, system_get_chip_id(),gMACSta[0], gMACSta[1], gMACSta[2], gMACSta[3], gMACSta[4], gMACSta[5]);
        MQTT_Publish(&mqttClient, topicDefaultStatus, StatusMessage, strlen(StatusMessage), 0, 0);
}

/**
 * @brief Receive by callback the data from MQTT topics subscribed
 *
 * When new message comes from a topic that the ESP is subscribed, it'll call th
 * is function and then the device will process if it's for himself or if he nee
 * ds to pass to the MSP430 through the UART-TX.
 * @param args Used by MQTT structure
 * @param topic The topic in that the ESP received the message
 * @param topicDefaultlen Length of the topic
 * @param data Data or the "message" received in the topic subscribed
 * @param data_len Length of the data received
 */
static void ICACHE_FLASH_ATTR mqttDataCb(uint32_t *args, const char* topic, uint32_t topicDefaultlen, const char *data, uint32_t data_len)
{
        char *topicBuf = (char*)os_zalloc(topicDefaultlen + 1),
             *dataBuf = (char*)os_zalloc(data_len + 1);

        MQTT_Client* client = (MQTT_Client*)args;
        os_memcpy(topicBuf, topic, topicDefaultlen);
        topicBuf[topicDefaultlen] = 0;
        os_memcpy(dataBuf, data, data_len);
        dataBuf[data_len] = 0;
        // INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
        if(dataBuf[0]=='R'&&dataBuf[1]=='0'&&dataBuf[2]=='2'&&dataBuf[3]=='5'&&dataBuf[4]=='5')
                ShowID();
        else if(dataBuf[0]=='R'&&dataBuf[1]=='0'&&dataBuf[2]=='2'&&dataBuf[3]=='5'&&dataBuf[4]=='4') {
                infoRSSI();
        }
        else if(dataBuf[0]=='R'&&dataBuf[1]=='0'&&dataBuf[2]=='2'&&dataBuf[3]=='5'&&dataBuf[4]=='3') {
                networkIPGet();
        }
        else{
        #ifdef UART_PEN
                GPIO_5(0);
                GPIO_4(0);
        #endif
                INFO("%s",dataBuf);
        }

        os_free(topicBuf);
        os_free(dataBuf);
}

/**
 * @brief Print information about AWGES info in the FW
 *
 * @param void --
 */
void printLogoAWGES(void)
{
        INFO("[INFO] LightDNA ESP8266 Firmware\r\n\n");
        INFO("\t    ___      _____ ___ ___  \n");
        INFO("\t   /_\\ \\    / / __| __/ __| \n");
        INFO("\t  / _ \\ \\/\\/ | (_ | _|\\__ \\ \n");
        INFO("\t /_/ \\_\\_/\\_/ \\___|___|___/ \n\n");
        INFO("[INFO] Dev. Anderson Ignacio da Silva\r\n");
        INFO("[INFO] Hardware Address:%s\n",DeviceID);
        uint8_t macaddr_ap[6],
                macaddr_sta[6],
                MACStatus = 1;
        static char temp[18];

        MACStatus = wifi_get_macaddr(STATION_IF,macaddr_sta);
        MACStatus =  wifi_get_macaddr(SOFTAP_IF,macaddr_sta);
        if (MACStatus) {
                os_sprintf(temp, MACSTR, macaddr_ap[0], macaddr_ap[1], macaddr_ap[2], macaddr_ap[3], macaddr_ap[4], macaddr_ap[5]);
                INFO("[INFO] MAC Address AP:  [%s]\n",temp);
                os_sprintf(temp, MACSTR, macaddr_sta[0], macaddr_sta[1], macaddr_sta[2], macaddr_sta[3], macaddr_sta[4], macaddr_sta[5]);
                INFO("[INFO] MAC Address STA: [%s]\n",temp);
        }
        else
                INFO("[INFO] Error to get MAC Address\n");
}

/**
 * @brief Print information about the device when it boots
 *
 * @param void --
 */
void ICACHE_FLASH_ATTR printInfo(void)
{
        INFO("\r\n\r\n[INFO] BOOTUP...\r\n");
        printLogoAWGES();
        INFO("[INFO] SDK: %s\r\n", system_get_sdk_version());
        INFO("[INFO] Firmware LightDNA: %s\r\n", FW_VERSION_LIGHTDNA);
        INFO("[INFO] Chip ID: %08X\r\n", system_get_chip_id());
        INFO("[INFO] Memory info:\r\n");
        system_print_meminfo();
        INFO("[INFO] -------------------------------------------\n");
        INFO("[INFO] Build time: %s\n", BUID_TIME);
        INFO("[INFO] -------------------------------------------\n");

}

/**
 * @brief Blink a LED to inform that's running ok
 *
 *
 * @param void --
 */
void BlinkStatusCB(void)
{
        if(gBlinkStatus) {
                GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 0);
                gBlinkStatus = false;
        }
        else{
                GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 1);
                gBlinkStatus = true;
        }
        os_timer_disarm(&gStatusTimerBlink);
        os_timer_setfn(&gStatusTimerBlink, BlinkStatusCB, NULL);
        os_timer_arm(&gStatusTimerBlink, 500, 0);
}

/**
 * @brief Send through MQTT the correspond message of the type of topic received
 * .
 *
 * When the ESP8266 receives a message from the MSP430 in the LightDNA protocol
 * it format the data received in send this though the MQTT correspond topic
 * @param void --
 */
void SendMQTT(void)
{
        rxPosCharIndex = 0;
#ifdef UART_PEN
        GPIO_5(1);
        GPIO_4(1);
#endif
        switch(gBufferInput[0]) {
        case 'F':
                gBufferInput[0] = ' ';
                MQTT_Publish(&mqttClient, topicDefaultMsp, gBufferInput, strlen(gBufferInput), 0, 0);
                break;
        case 'D':
                gBufferInput[0] = ' ';
                MQTT_Publish(&mqttClient, topicDefaultDimmer, gBufferInput, strlen(gBufferInput), 0, 0);
                break;
        case 'C':
                gBufferInput[0] = ' ';
                MQTT_Publish(&mqttClient, topicDefaultCurrent, gBufferInput, strlen(gBufferInput), 0, 0);
                break;
        case 'P':
                gBufferInput[0] = ' ';
                MQTT_Publish(&mqttClient, topicDefaultPir, gBufferInput, strlen(gBufferInput), 0, 0);
                break;
        case 'T':
                gBufferInput[0] = ' ';
                MQTT_Publish(&mqttClient, topicDefaultTemp, gBufferInput, strlen(gBufferInput), 0, 0);
                break;
        case 'L':
                gBufferInput[0] = ' ';
                MQTT_Publish(&mqttClient, topicDefaultLdr, gBufferInput, strlen(gBufferInput), 0, 0);
                break;
        }
        serialState = FIRST_IDT;
}

/**
 * @brief Callback function for serial input - rx.
 *
 * This function is called in the uart.c, located at driver/uart.c where it receives the
 * serial input char on RX of ESP8266. Every time a new char is received by the ESP8266
 * the device calls this callback function to treat this. This callback function also buf
 * fers the default commands of LightDNA Protocol (which is located at /docs), received
 * by the MSP430 or other MCU connected on his serial. The global variable that store the
 * UART received commands is gBufferInput. When the ESP8266 receives the return command
 * #data#, it send through the MQTT network broker.
 * @param CharBuffer New char received on serial
 * @note Something to note.
 */
void CbRXSerial(uint8 CharBuffer)
{
        switch(serialState)
        {
        case FIRST_IDT:
                if(CharBuffer == '#')
                {
                        serialState = DATA_B;
                        for(rxPosCharIndex=0; rxPosCharIndex<40; rxPosCharIndex++)
                                gBufferInput[rxPosCharIndex] = '\0';
                        rxPosCharIndex = 0;
                }
                break;
        case DATA_B:
                if(CharBuffer != '#' && rxPosCharIndex < 40)
                {
                        gBufferInput[rxPosCharIndex] = CharBuffer;
                        rxPosCharIndex++;
                }
                else
                {
                        serialState = COMPLETE;
                }
                break;
        }

        if(serialState == COMPLETE) SendMQTT();
}

/**
 * @brief Prepare the topics copying the devices hardware address to the MQTT to
 * pics.
 *
 * @param void --
 */
void PrepareTopics(void){
        os_sprintf(topicDefault, TOPIC_DEFAULT, system_get_chip_id());
        os_sprintf(topicDefaultDimmer, TOPIC_DIMMER, system_get_chip_id());
        os_sprintf(topicDefaultCurrent, TOPIC_CURRENT, system_get_chip_id());
        os_sprintf(topicDefaultStatus, TOPIC_STATUS, system_get_chip_id());
        os_sprintf(topicDefaultTemp, TOPIC_TEMP, system_get_chip_id());
        os_sprintf(topicDefaultPir, TOPIC_PIR, system_get_chip_id());
        os_sprintf(topicDefaultLdr, TOPIC_LDR, system_get_chip_id());
        os_sprintf(topicDefaultRSSI, TOPIC_RSSI, system_get_chip_id());
        os_sprintf(topicDefaultMsp, TOPIC_MSP, system_get_chip_id());
        os_sprintf(topicDefaultIp, TOPIC_IP, system_get_chip_id());
        os_sprintf(MessageDead, MESSAGE_OFFLINE, system_get_chip_id());
        os_sprintf(DeviceID, "%08X", system_get_chip_id());
}

/**
 * @brief Init the application registering all connections.
 *
 * Initializes the application on ESP8266 with all network connections
 * @param void --
 */
static void ICACHE_FLASH_ATTR app_init(void)
{
        char clientIDMQTT[20];
        uart_init(BIT_RATE_115200, BIT_RATE_115200);
        PrepareTopics();
        printInfo();
        wifi_get_macaddr(STATION_IF, gMACSta);

        os_sprintf(clientIDMQTT, MQTT_CLIENT_ID, system_get_chip_id());
#ifdef LED_DEBUG
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U,   FUNC_GPIO13);
        os_timer_disarm(&gStatusTimerBlink);
        os_timer_setfn(&gStatusTimerBlink, BlinkStatusCB, NULL);
        os_timer_arm(&gStatusTimerBlink, 500, 0);
#endif
#ifdef UART_PEN
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,  FUNC_GPIO5);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U,  FUNC_GPIO4);
        GPIO_5(1);
        GPIO_4(1);
#endif
        MQTT_InitConnection(&mqttClient, MQTT_HOST, MQTT_PORT, DEFAULT_SECURITY);
        if ( !MQTT_InitClient(&mqttClient, clientIDMQTT, MQTT_USER, MQTT_PASS, MQTT_KEEPALIVE, MQTT_CLEAN_SESSION) )
        {
                INFO("Failed to initialize properly. Check MQTT version.\r\n");
                return;
        }

        MQTT_InitLWT(&mqttClient, topicDefaultStatus, MessageDead, 0, 0);    //Last Will Teastment - Indicate that device has been dead, /lights/device_addres/status(Topic) Device device_address it's offline!(message)
        MQTT_OnConnected(&mqttClient, mqttConnectedCb);
        MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
        MQTT_OnPublished(&mqttClient, mqttPublishedCb);
        MQTT_OnData(&mqttClient, mqttDataCb);

        WIFI_Connect(STA_SSID, STA_PASS, wifiConnectCb);
}

void user_init(void)
{
        system_init_done_cb(app_init);
}
