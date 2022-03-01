/* mbed Microcontroller Application
 * Copyright (c) 2022 C Gerrish (Gerrikoio)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"

#include "ble_app2.h"
#include "DeviceInformationService.h"

#include "Panasonic_SNGCJA5.h"

// Xenon Pin Map for Digital - Pin Numbers differ to nRF52840
#define XEN_D2      p33
#define XEN_D3      p34
#define XEN_D4      p40
#define XEN_D5      p42
#define XEN_D6      p43
#define XEN_D7      p44
#define XEN_D8      p35

static const uint8_t ARRSIZE = 2;

static uint16_t pmcountchar_values[ARRSIZE] = {0x00};
static uint8_t interval_value = 0x0A;           //10sec

// Handles for button and led and connection
static uint8_t pmcount_handle = 0;
static uint8_t pminterval_handle = 0;
static uint8_t connectionhandle = 0;

// We create our own user LED to indicate BLE status
DigitalOut ble_led(XEN_D7, 0);

// Define our I2C Global Interface
I2C i2c(I2C_SDA0, I2C_SCL0);

Panasonic_SNGCJA5 PM(i2c, SNGCJA5_ADDRESS);

int PMSenseEventNo;

Ticker LED_Blink;

BLEApp app;

void LED_Blinkhandler()
{
    ble_led = !ble_led;
}

void debug_printhandler(uint16_t pmval1, uint16_t pmval2)
{
    
    printf("\r\nPM Counts (0.5um to 2.5um): %u\r\n", pmval1);
    printf("PM Counts (greater than 2.5um): %u\r\n", pmval2);
    
}

void PMSense_tickerhandler()
{
    static uint8_t sample_cntr = 1;

    char TXdata[1] = {SNGCJA5_STATUS};      // start by looking at sensor status
    uint8_t SensorStatus[1] = {0x01};
    int i2cError = 0;
    // check sensor status to ensure no sensor error
    if ((i2cError = PM.getData(TXdata, SensorStatus, sizeof(SensorStatus))) == 0) {
        if (SensorStatus[0] == 0) {
            TXdata[0] = {SNGCJA5_REG2};      // We want data for 0.5um and above
            uint8_t UM05data[4] = {'\0'};
            if ((i2cError = PM.getData(TXdata, UM05data, sizeof(UM05data))) == 0) {
                TXdata[0] = {SNGCJA5_REG4};      // start by looking at sensor status
                uint8_t UM25data[6] = {'\0'};
                if ((i2cError = PM.getData(TXdata, UM25data, sizeof(UM25data))) == 0) {
                    pmcountchar_values[0] += (PM.convert2byte(UM05data) + PM.convert2byte(UM05data+2));
                    pmcountchar_values[1] += (PM.convert2byte(UM25data) + PM.convert2byte(UM25data+2) + PM.convert2byte(UM25data+4));
                }
            }
        }
    }
    
    if (sample_cntr < interval_value) {
        sample_cntr++;
    }
    else {
        // Update the BLE data
        // We now divide the data to get the average over the sample period
        pmcountchar_values[0] = pmcountchar_values[0]/sample_cntr;
        pmcountchar_values[1] = pmcountchar_values[1]/sample_cntr;
        app.updateCharacteristicShortValue(pmcount_handle, pmcountchar_values, sizeof(pmcountchar_values)/sizeof(pmcountchar_values[0]));
        //event.call(debug_printhandler, pmcountchar_values[0], pmcountchar_values[0]);
        printf("\r\nPM Counts (0.5um to 2.5um): %u\r\n", pmcountchar_values[0]);
        printf("PM Counts (greater than 2.5um): %u\r\n", pmcountchar_values[1]);
        // Reset sample counters and data arrays
        sample_cntr = 1;
        memset(pmcountchar_values, '\0', ARRSIZE);
    }
    /*
    if (btnvalue != prev_btnvalue) {
        app.updateCharacteristicValue(pmcount_handle, btnvalue, sizeof(btnvalue));
    }    
    prev_btnvalue = btnvalue;
    */
}

void bleApp_InitCompletehandler(BLE &ble, events::EventQueue &_event)
{

    /* Declare our device name - note that the ble_app library does not 
       automatically shorten full names if too long.
    */
    const char *DEVICE_NAME =         "PMsense";

    // The Gatt Service UUID which is also used to advertise the service
    const char *GATTSERVICE_UUID =    "20220214-1313-1313-1313-f8f381aa84ed";
    // PM Count and PM Interval Characteristics
    const char *PMCOUNTCHAR_UUID =     "20220214-1515-1515-1515-f8f381aa84ed";
    const char *PMINTERVALCHAR_UUID =  "20220214-1616-1616-1616-f8f381aa84ed";
    //const char *PMSenseApp::PMDENSITYCHAR_UUID =        "20220214-1414-1414-1414-f8f381aa84ed";
    //const char *PMSenseApp::PMSTATUSCHAR_UUID =        "20220214-1717-1717-1717-f8f381aa84ed";

    UUID PMSENSE_ATTRI_2901 = 0x2901;                       // attribute UUID containing user description
    UUID PMSENSE_ATTRI_2904 = 0x2904;                       // attribute UUID containing presentation format

    /* setup the default phy used in connection to 2M to reduce power consumption */
    if (ble.gap().isFeatureSupported(ble::controller_supported_features_t::LE_2M_PHY)) {
        ble::phy_set_t phys(/* 1M */ false, /* 2M */ true, /* coded */ false);
        ble_error_t error = ble.gap().setPreferredPhys(/* tx */&phys, /* rx */&phys);
        
        /* PHY 2M communication will only take place if both peers support it */
        if (error) {
            print_error(error, "GAP::setPreferedPhys failed\r\n");
        }
        else {
            printf("using 2M PHY\r\n");
            fflush(stdout);           // Just for serial output
        }
    } else {
        /* otherwise it will use 1M by default */
        printf("2M not supported. Sticking with 1M PHY\r\n");
    }

    // Add in new service
    printf("Adding Device Information Service\r\n");
    fflush(stdout);           // Just for serial output
    // Start by adding our Device Information data
    DeviceInformationService::add_service(
        ble,
        "Panasonic", 
        "SN-GCJA5",
        "0000",
        "18P:2021-08-23",
        "nRF52840 v0.01",
        nullptr,
        nullptr,
        nullptr,
        nullptr
    );

    // Add in new service
    printf("Adding PM Sense Service\r\n");
    fflush(stdout);           // Just for serial output

    uint8_t PMCOUNTCHAR_DESCR[28] = "+0.5um and +2.5um PM Counts";
    GattAttribute *pmcount_descriptor_attribute  = new GattAttribute( 
                                PMSENSE_ATTRI_2901, // attribute type
                                PMCOUNTCHAR_DESCR,           // descriptor 
                                28,           // length of the buffer containing the value
                                32,         // max length
                                true // variable length
                            );

    // Interval Presentation Format: 0x1B: opaque structure; 0x00: no exponent; 0x27B5: unit = concentration(count per m3); 0x01: Bluetooth SIG namespace; 0x0000: No description
    uint8_t PMCOUNT_PRESENTFORMAT_STR[7] = {0x1B, 0x00, 0xB5, 0x27, 0x01, 0x00, 0x00};
    GattAttribute *pmcount_presentformat_attribute = new GattAttribute( 
                                PMSENSE_ATTRI_2904, // attribute type
                                PMCOUNT_PRESENTFORMAT_STR,           // descriptor 
                                7,           // length of the buffer containing the value
                                7,         // max length
                                true // variable length
                            );
    GattAttribute *pmcount_descriptors[] = {pmcount_descriptor_attribute, pmcount_presentformat_attribute};


    uint8_t PMINTERVALCHAR_DESCR[28] = "Update Interval (>= 10 sec)";
    GattAttribute *pminterval_descriptor_attribute = new GattAttribute( 
                                PMSENSE_ATTRI_2901, // attribute type
                                PMINTERVALCHAR_DESCR,           // descriptor 
                                28,           // length of the buffer containing the value
                                32,         // max length
                                true // variable length
                            );

    // Interval Presentation Format: 0x04: unsigned 8 bit integer; 0x00: no exponent; 0x2703: unit = time(second); 0x01: Bluetooth SIG namespace; 0x0000: No description
    uint8_t PMINTERVAL_PRESENTFORMAT_STR[7] = {0x04, 0x00, 0x03, 0x27, 0x01, 0x00, 0x00};
    GattAttribute *pminterval_presentformat_attribute = new GattAttribute( 
                                PMSENSE_ATTRI_2904, // attribute type
                                PMINTERVAL_PRESENTFORMAT_STR,           // descriptor 
                                7,           // length of the buffer containing the value
                                7,         // max length
                                true // variable length
                            );
    GattAttribute *pminterval_descriptors[] = {pminterval_descriptor_attribute, pminterval_presentformat_attribute};

    // Create our Gatt Service Profile
    // For PM Count Characteristic, we add in an additional notification property and our descriptors
    ReadOnlyArrayGattCharacteristic<uint16_t,ARRSIZE> pmcount_characteristic(UUID(PMCOUNTCHAR_UUID), pmcountchar_values, 
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY, pmcount_descriptors, 2);
    
    ReadWriteGattCharacteristic<uint8_t> pminterval_characteristic(UUID(PMINTERVALCHAR_UUID), &interval_value, 
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NONE, pminterval_descriptors, 2);

    GattCharacteristic *charTable[] = { &pmcount_characteristic, & pminterval_characteristic };
    GattService BLS_GattService(UUID(GATTSERVICE_UUID), charTable, sizeof(charTable) / sizeof(charTable[0]));
    
    // We now add in our button & led service
    app.add_new_gatt_service(BLS_GattService);

    pmcount_handle = pmcount_characteristic.getValueHandle();
    pminterval_handle = pminterval_characteristic.getValueHandle();
    printf("PM Count Charactertistic handle: %u\r\n", pmcount_handle);
    printf("PM Interval Charactertistic handle: %u\r\n", pminterval_handle);
    fflush(stdout);           // Just for serial output

    // Set up advertising information
    app.set_GattUUID_128(GATTSERVICE_UUID);

    app.set_advertising_name(DEVICE_NAME);

}

void bleApp_Connectionhandler(BLE &ble, events::EventQueue &event, const ble::ConnectionCompleteEvent &params)
{
    LED_Blink.detach();
    ble_led = 1;

    connectionhandle = params.getConnectionHandle();

    printf("Now connected to: ");
    print_address(params.getPeerAddress());
    printf("Connection handle %u.\r\n", connectionhandle);

    // Initialise a event call every 1 second to retrieve data from the panasonic PM sensor (spec says data updated every 1 second)
    PMSenseEventNo = event.call_every(1000ms, &PMSense_tickerhandler);
}

void bleApp_Disconnectionhandler(BLE &ble, events::EventQueue &event, const ble::DisconnectionCompleteEvent &params)
{
    printf("Disconnection event. Handle %u\r\n", params.getConnectionHandle());
    // Detach ticker
    event.cancel(PMSenseEventNo);
    LED_Blink.attach(LED_Blinkhandler, 1s);
}

void bleApp_UpdatesEnabledhandler(const GattUpdatesEnabledCallbackParams &params)
{
    printf("Updates Enabled.\r\n");
    
}

void bleApp_UpdatesDisabledhandler(const GattUpdatesDisabledCallbackParams &params)
{
    printf("Updates Disabled.\r\n");
    
}

void bleApp_WriteEventhandler(const GattWriteCallbackParams &params)
{
    printf("Write Event via connection handle %u.\r\n", params.connHandle);
    if (params.handle == pminterval_handle) {
        interval_value = params.data[0];
        printf("Update Interval changed to %u seconds\r\n", interval_value);
        //led = !ledvalue;
    }
    
}

void bleApp_ReadEventhandler(const GattReadCallbackParams &params)
{
    printf("Read Event via connection handle %u.\r\n", params.connHandle);
    if (params.handle == pminterval_handle) printf("Sample Interval characteristic data read: %u\r\n", params.data[0]);
    else if (params.handle == pmcount_handle) printf("PM Count characteristic data read: %u %u\r\n", params.data[0], params.data[2]);
    
}

/**
    * Handler called when a notification or an indication has been sent.
    */
void onDataSenthandler(const GattDataSentCallbackParams &params)
{
    if (params.attHandle == pmcount_handle) {
        printf("PM Count update callback\r\n");
    }
}


void bleApp_MTUchangehandler(ble::connection_handle_t connectionHandle, uint16_t attMtuSize)
{
    printf("MTU change alert.\r\n");
    printf("connection handle: %u\r\n", connectionHandle);
    printf("New Mtu Size: %u\r\n", attMtuSize);
    
}


int main()
{

    printf("\r\nPanasonic SN-GCJA5 Particulate Matter Sensing BLE Application\r\n");
    printf("Monitoring 0.5um to 2.5um and +2.5um counts\r\n");

    // Set our i2c frequency for project
    i2c.frequency(400000);      //400kHz

    app.set_single_connection_only(true);       // only want one connection for demo

    // We set up all our optional Gatt Server event handlers   
    app.on_connect(bleApp_Connectionhandler);
    app.on_disconnect(bleApp_Disconnectionhandler);
    app.on_updatesenabled(bleApp_UpdatesEnabledhandler);
    app.on_updatesdisabled(bleApp_UpdatesDisabledhandler);
    app.on_serverwriteevent(bleApp_WriteEventhandler);
    app.on_serverreadevent(bleApp_ReadEventhandler);
    app.on_serversentevent(onDataSenthandler);
    app.on_AttMtuChange(bleApp_MTUchangehandler);

    printf("Waiting for PM Sensor to warm up (takes 8 seconds)...");
    fflush(stdout);           // Just for serial output

    // sleep for 10 seconds to allow for PM sensor to warmup
    ThisThread::sleep_for(TIME2FIRSTREAD);

    printf("done!\r\n");
    fflush(stdout);           // Just for serial output
    LED_Blink.attach(LED_Blinkhandler,1s);

    // Start app and include our BLE Initialise Complete handler
    app.start(bleApp_InitCompletehandler);

    while (true) {
        ThisThread::sleep_for(500ms);
    }
}
