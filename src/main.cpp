/***********************************************************************
 * Project      :     tiny32_SX1278_LoRa_Master
 * Description  :     Example code tiny32 board interface with SX1278 to be LoRa Master
 * Hardware     :     tiny32_v3
 * Author       :     Tenergy Innovation Co., Ltd.
 * Date         :     28/11/2024
 * Revision     :     1.1
 * Rev1.0       :     Origital
 * Rev1.1       :     check and filter duplicate data
 * website      :     http://www.tenergyinnovation.co.th
 * Email        :     uten.boonliam@tenergyinnovation.co.th
 * TEL          :     +66 89-140-7205
 ***********************************************************************/
#include <Arduino.h>
#include <tiny32_v3.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include <esp_task_wdt.h>

/**************************************/
/*          Firmware Version          */
/**************************************/
#define FIRMWARE_VERSION "1.1"

/**************************************/
/*          Header project            */
/**************************************/
void header_print(void)
{
    Serial.printf("\r\n***********************************************************************\r\n");
    Serial.printf("* Project      :     tiny32_SX1278_LoRa_Master\r\n");
    Serial.printf("* Description  :     Example code tiny32 board interface with SX1278 to be LoRa Master\r\n");
    Serial.printf("* Hardware     :     tiny32_v3\r\n");
    Serial.printf("* Author       :     Tenergy Innovation Co., Ltd.\r\n");
    Serial.printf("* Date         :     04/07/2022\r\n");
    Serial.printf("* Revision     :     %s\r\n", FIRMWARE_VERSION);
    Serial.printf("* website      :     http://www.tenergyinnovation.co.th\r\n");
    Serial.printf("* Email        :     uten.boonliam@tenergyinnovation.co.th\r\n");
    Serial.printf("* TEL          :     +66 89-140-7205\r\n");
    Serial.printf("***********************************************************************/\r\n");
}

/**************************************/
/*        define object variable      */
/**************************************/
tiny32_v3 mcu;
WiFiManager wm;

/**************************************/
/*            GPIO define             */
/**************************************/
#define SCK 18
#define MISO 23
#define MOSI 19
#define SS 5
#define RST 26
#define DIO0 27

/**************************************/
/*           Bandwidth  define        */
/**************************************/
// 433E6 for Asia
// 866E6 for Europe
// 915E6 for North America
#define BAND 433E6

/**************************************/
/*       Constand define value        */
/**************************************/
// 10 seconds WDT
#define WDT_TIMEOUT 10

/**************************************/
/*       eeprom address define        */
/**************************************/

/**************************************/
/*        define global variable      */
/**************************************/
char unit[20];
char macid[50];
uint16_t recordCount = 0; // this variable for count boot time
long lastMsg = 0;
char msg[50];
char string[40];
String client_param = "";

/* LoRa variable */
byte msgCount = 0;             // count of outgoing messages
byte LoRa_ID_Gateway = 1;      // LoRa Gate Way ID {Every Client will indicate to this ID}
byte LoRa_ID_local = 1;        // address of this device **
byte LoRa_ID_destination = 99; // 0x99 is mean all client node **
byte LoRa_Client_ID_2 = 2;
long lastSendTime = 0;

struct param_LoRa
{
    String id;
    String fw;
    String rssi;
    String topic;
    float param_1;
    float param_2;
    float param_3;
    float param_4;
    float param_5;
    float param_6;
    float param_7;
    float param_8;
    float param_9;
    float param_10;
};

param_LoRa LoRa_client_1;
byte incomingMsgId_LoRa = 0;

/**************************************/
/*           define function          */
/**************************************/
void LoRa_onReceive(int packetSize);                                                    // LoRa get message funct
void LoRa_sendMessage(String outgoing);                                                 // LoRa send message funct
void LoRa_sendMessage_toDestination(String outgoing, byte destination = LoRa_Client_ID_2); // LoRa send message funct
bool LoRa_ConvertParam(String incoming);

/***********************************************************************
 * FUNCTION:    setup
 * DESCRIPTION: setup process
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void setup()
{
    Serial.begin(115200);
    header_print();

    //*** Define Unit name ***
    String _mac1 = WiFi.macAddress().c_str();
    Serial.printf("Setup: macAddress = %s\r\n", WiFi.macAddress().c_str());
    String _mac2 = _mac1.substring(9, 17);
    _mac2.replace(":", "");
    String _mac3 = "tiny32-" + _mac2;
    _mac3.toCharArray(unit, _mac3.length() + 1);
    Serial.printf("Setup: Unit ID => %s\r\n", unit);

    /* LoRa configuration */
    Serial.println("Info: LoRa Duplex config....");
    // SPI LoRa pins
    SPI.begin(SCK, MISO, MOSI, SS);
    // setup LoRa transceiver module
    LoRa.setPins(SS, RST, DIO0);
    if (!LoRa.begin(BAND))
    { // initialize ratio at 866 MHz
        Serial.println("\r\nError: LoRa init failed. Check your connections.");
        mcu.buzzer_beep(3);
        while (true)
            ; // if failed, do nothing
    }
    Serial.printf("\tInfo: Frequency Band  = %dHz\r\n", int(BAND));
    Serial.printf("\tInfo: Gateway ID = %02d\r\n", LoRa_ID_Gateway);
    Serial.printf("\tInfo: My ID = %02d\r\n", LoRa_ID_local);
    Serial.printf("\tInfo: Destination ID = %02d\r\n", LoRa_ID_destination);

    Serial.println("Configuring WDT...");
    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);               // add current thread to WDT watch
    mcu.buzzer_beep(2);
}

/***********************************************************************
 * FUNCTION:    loop
 * DESCRIPTION: loop process
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void loop()
{
    LoRa_onReceive(LoRa.parsePacket());
    esp_task_wdt_reset();
    vTaskDelay(100);

    // send command via RoLa to ON Relay
    if (mcu.Sw1())
    {
        mcu.buzzer_beep(1);
        Serial.printf("Info: RELAY ON\r\n");
        while (mcu.Sw1())
            ;
        String _cmd = "relay:on";
        LoRa_sendMessage_toDestination(_cmd, LoRa_Client_ID_2);
        vTaskDelay(100);
    }

    // send command via RoLa to ON Relay
    if (mcu.Sw2())
    {
        mcu.buzzer_beep(1);
        Serial.printf("Info: RELAY OFF\r\n");
        while (mcu.Sw1())
            ;
        String _cmd = "relay:off";
        LoRa_sendMessage_toDestination(_cmd,LoRa_Client_ID_2);
        vTaskDelay(100);
    }
}

/***********************************************************************
 * FUNCTION:    LoRa_onReceive
 * DESCRIPTION: receive data to LoRaWan
 * PARAMETERS:  int packetSize
 * RETURNED:    nothing
 ***********************************************************************/
void LoRa_onReceive(int packetSize)
{
    static int _LoRa_sentForm = 0;
    static byte _LoRa_MessageID = 0;
    if (packetSize == 0)
        return; // if there's no packet, return

    // read packet header bytes:
    int recipient = LoRa.read();       // recipient address
    byte sender = LoRa.read();         // sender address
    byte incomingMsgId = LoRa.read();  // incoming msg ID
    byte incomingLength = LoRa.read(); // incoming msg length

    String incoming = "";
    String _JsonStr;
    incomingMsgId_LoRa = incomingMsgId;

    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }
    // Serial.printf("\r\n\r\n-----------------------------------------------------------------\r\n");
    // Serial.printf("Debug: incomingLength = %d\r\n", incomingLength);

    if (incomingLength != incoming.length())
    { // check length for error
        // Serial.println("error: message length does not match length");
        return; // skip rest of function
    }


    Serial.printf("Previde data: %d [%d] Dulication data\r\n", _LoRa_sentForm, _LoRa_MessageID);
    if ((sender == _LoRa_sentForm) && (incomingMsgId == _LoRa_MessageID))
    {
        Serial.printf("Error: %d [%d] Dulication data\r\n", recipient, incomingMsgId);
        return;
    }
    else
    {
        _LoRa_sentForm = sender;
        _LoRa_MessageID = incomingMsgId;
    }

    mcu.RedLED(1);
    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(incomingMsgId)); // ลำดับของข้อมูลที่ RoLa Client ส่งมา โดย Client จะทำการนับไปเรื่อยๆ
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));

    // if the recipient isn't this device or broadcast,
    if (recipient == LoRa_ID_local) // ผู้ส่งมาต้องไม่เท่ากับตัวมันเอง
    {
        Serial.printf("Pass: This message for me. [0x%02d]\r\n", LoRa_ID_local);
    }
    else
    {
        Serial.println("Fail: This message is not for me.");
        return; // skip rest of function
    }

    bool _LoRa_result = LoRa_ConvertParam(incoming);
    if (_LoRa_result) // check project name is correct or not?
    {
        // do something
    }
    else
    {
        // do something
    }
    mcu.RedLED(0);
    Serial.println("------------------------------------");
}

/***********************************************************************
 * FUNCTION:    LoRa_sendMessage
 * DESCRIPTION: send data to LoRaWan
 * PARAMETERS:  String outgoing ** ขนาดห้ามเกิน 250 BYTE **
 * RETURNED:    nothing
 ***********************************************************************/
void LoRa_sendMessage(String outgoing)
{
    Serial.printf("outgoing.length() = %d\r\n", outgoing.length());
    if (outgoing.length() > 250)
    {
        mcu.buzzer_beep(3);
        Serial.println("Error: Message length is over, it can't send via LoRa protocol!");
        return;
    }
    LoRa.beginPacket();              // start packet
    LoRa.write(LoRa_ID_destination); // add destination address
    LoRa.write(LoRa_ID_local);       // add sender address
    LoRa.write(msgCount);            // add message ID
    LoRa.write(outgoing.length());   // add payload length
    LoRa.print(outgoing);            // add payload
    LoRa.endPacket();                // finish packet and send it
    msgCount++;                      // increment message ID
    if (msgCount >= 100)
        msgCount = 0;
}

/***********************************************************************
 * FUNCTION:    LoRa_sendMessage_toDestination
 * DESCRIPTION: send data to LoRaWan
 * PARAMETERS:  byte destination , String outgoing ** ขนาดห้ามเกิน 250 BYTE **
 * RETURNED:    nothing
 ***********************************************************************/
void LoRa_sendMessage_toDestination(String outgoing, byte destination)
{
    Serial.printf("outgoing.length() = %d\r\n", outgoing.length());
    if (outgoing.length() > 250)
    {
        mcu.buzzer_beep(3);
        Serial.println("Error: Message length is over, it can't send via LoRa protocol!");
        return;
    }
    LoRa.beginPacket();            // start packet
    LoRa.write(destination);       // add destination address
    LoRa.write(LoRa_ID_local);     // add sender address
    LoRa.write(msgCount);          // add message ID
    LoRa.write(outgoing.length()); // add payload length
    LoRa.print(outgoing);          // add payload
    LoRa.endPacket();              // finish packet and send it
    msgCount++;                    // increment message ID
    if (msgCount >= 100)
        msgCount = 0;
}

/***********************************************************************
 * FUNCTION:    LoRa_ConvertParam
 * DESCRIPTION: read value sensor from Inverter ATESS
 * PARAMETERS:  nothing
 * RETURNED:    true/false
 ***********************************************************************/
bool LoRa_ConvertParam(String incoming)
{

    incoming.trim();
    Serial.printf("incoming[org] = %s\r\n", incoming.c_str());

    int _index_start;
    int _index_stop;

    //-------------------- id -----------------------//
    _index_start = incoming.indexOf("id\":\"");
    incoming.remove(0, _index_start + 5); // remove ` id:" `
    _index_stop = incoming.indexOf("\"");
    LoRa_client_1.id = incoming.substring(0, _index_stop);
    Serial.printf("id = %s \r\n", LoRa_client_1.id);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //----------------------- fw --------------------//
    _index_start = incoming.indexOf("fw\":\"");
    incoming.remove(0, _index_start + 5);
    _index_stop = incoming.indexOf("\"");
    LoRa_client_1.fw = incoming.substring(0, _index_stop);
    Serial.printf("fw = %s \r\n", LoRa_client_1.fw);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- topic ---------------------//
    _index_start = incoming.indexOf("topic\":\"");
    incoming.remove(0, _index_start + 8);
    _index_stop = incoming.indexOf("\"");
    LoRa_client_1.topic = incoming.substring(0, _index_stop);
    Serial.printf("topic = %s \r\n", LoRa_client_1.topic);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_1 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf(",");
    LoRa_client_1.param_1 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_1 = %.2f \r\n", LoRa_client_1.param_1);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_2 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf(",");
    LoRa_client_1.param_2 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_2 = %.2f \r\n", LoRa_client_1.param_2);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_3 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf(",");
    LoRa_client_1.param_3 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_3 = %.2f \r\n", LoRa_client_1.param_3);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_4 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf(",");
    LoRa_client_1.param_4 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_4 = %.2f \r\n", LoRa_client_1.param_4);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_5 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf(",");
    LoRa_client_1.param_5 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_5 = %.2f \r\n", LoRa_client_1.param_5);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_6 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf(",");
    LoRa_client_1.param_6 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_6 = %.2f \r\n", LoRa_client_1.param_6);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_7 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf(",");
    LoRa_client_1.param_7 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_7 = %.2f \r\n", LoRa_client_1.param_7);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_8 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf(",");
    LoRa_client_1.param_8 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_8 = %.2f \r\n", LoRa_client_1.param_8);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_9 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf(",");
    LoRa_client_1.param_9 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_9 = %.2f \r\n", LoRa_client_1.param_9);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    //---------------------- param_10 ---------------------//
    _index_start = incoming.indexOf(",");
    incoming.remove(0, _index_start + 1);
    _index_stop = incoming.indexOf("}");
    LoRa_client_1.param_10 = incoming.substring(0, _index_stop).toFloat();
    Serial.printf("LoRa_client_1.param_10 = %.2f \r\n", LoRa_client_1.param_10);
    // Serial, printf("\tincoming => %s\r\n", incoming.c_str());

    return true;
}
