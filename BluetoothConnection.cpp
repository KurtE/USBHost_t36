/* USB EHCI Host for Teensy 3.6
 * Copyright 2017 Paul Stoffregen (paul@pjrc.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * information about the BlueTooth HCI comes from logic analyzer captures
 * plus... http://affon.narod.ru/BT/bluetooth_app_c10.pdf
 */

//=============================================================================
// Bluetooth connections code
//=============================================================================


#include <Arduino.h>
#include "USBHost_t36.h"  // Read this header first for key info
#include "utility/bt_defines.h"

#define print   USBHost::print_
#define println USBHost::println_//#define DEBUG_BT

#define DEBUG_BT
#define DEBUG_BT_VERBOSE

#ifndef DEBUG_BT
#undef DEBUG_BT_VERBOSE
void inline DBGPrintf(...) {};
#else
#define DBGPrintf USBHDBGSerial.printf
#endif

#ifndef DEBUG_BT_VERBOSE
void inline VDBGPrintf(...) {};
#else
#define VDBGPrintf USBHDBGSerial.printf
#endif


// This is a list of all the drivers inherited from the BTHIDInput class.
// Unlike the list of USBDriver (managed in enumeration.cpp), drivers stay
// on this list even when they have claimed a top level collection.
BluetoothConnection *BluetoothConnection::s_first_ = NULL;

// When a new top level collection is found, this function asks drivers
// if they wish to claim it.  The driver taking ownership of the
// collection is returned, or NULL if no driver wants it.

//=============================================================================
// initialize the connection data
//=============================================================================
void BluetoothConnection::initializeConnection(BluetoothController *btController, uint8_t bdaddr[6], uint32_t class_of_device)
{
    device_driver_ = nullptr;
    btController_ = btController;  // back pointer to main object
    connection_rxid_ = 0;
    control_dcid_ = 0x70;
    interrupt_dcid_ = 0x71;
    sdp_dcid_ = 0x40;
    connection_complete_ = 0;
    if (!connection_started_) {
        connection_started_ = true;
        btController_->setTimer(nullptr, 0); // clear out timer
    }
    use_hid_protocol_ = false;
    sdp_connected_ = false;
    pending_control_tx_ = 0;

    device_driver_ = find_driver(nullptr, 0);

    // We need to save away the BDADDR and class link type?
    for (uint8_t i = 0; i < 6; i++) device_bdaddr_[i] = bdaddr[i];
    device_class_ = class_of_device;
}

//=============================================================================
// Find a driver
//=============================================================================
BTHIDInput * BluetoothConnection::find_driver(uint8_t *remoteName, int type)
{
    DBGPrintf("BluetoothController::find_driver(%x) type: %d\n", device_class_, type);
    if (device_class_ & 0x2000) DBGPrintf("  (0x2000)Limited Discoverable Mode\n");
    DBGPrintf("  (0x500)Peripheral device\n");
    if (device_class_ & 0x80) DBGPrintf("    Mouse\n");
    if (device_class_ & 0x40) DBGPrintf("    Keyboard\n");
    switch (device_class_ & 0x3c) {
    case 4: DBGPrintf("    Joystick\n"); break;
    case 8: DBGPrintf("    Gamepad\n"); break;
    case 0xc: DBGPrintf("    Remote Control\n"); break;
    }
    check_for_hid_descriptor_ = false;
    BTHIDInput *driver = BluetoothController::available_bthid_drivers_list;
    while (driver) {
        DBGPrintf("  driver %x\n", (uint32_t)driver);
        hidclaim_t claim_type = driver->claim_bluetooth(this, device_class_, remoteName, type);
        if (claim_type == CLAIM_INTERFACE) {
            DBGPrintf("    *** Claimed ***\n");
            return driver;
        } else if (claim_type == CLAIM_REPORT) {
            DBGPrintf("    *** Check for HID  ***\n");
            check_for_hid_descriptor_ = true;
        }
        driver = driver->next;
    }
    return NULL;
}

//=============================================================================
// Handle the rx2_
//=============================================================================
void BluetoothConnection::rx2_data(uint8_t *rx2buf) // called from rx2_data of BluetoothController
{
    // need to detect if these are L2CAP or SDP or ...
    uint16_t dcid =  rx2buf[6] + ((uint16_t)rx2buf[7] << 8);
    //DBGPrintf("@@@@@@ SDP MSG? %x %x %x @@@@@", dcid, sdp_dcid_, rx2buf[8]);

    if (dcid == sdp_dcid_) {
        switch (rx2buf[8]) {
        case SDP_SERVICE_SEARCH_REQUEST:
            process_sdp_service_search_request(&rx2buf[8]);
            break;
        case SDP_SERVICE_SEARCH_RESPONSE:
            process_sdp_service_search_response(&rx2buf[8]);
            break;
        case SDP_SERVICE_ATTRIBUTE_REQUEST:
            process_sdp_service_attribute_request(&rx2buf[8]);
            break;
        case SDP_SERVICE_ATTRIBUTE_RESPONSE:
            process_sdp_service_attribute_response(&rx2buf[8]);
            break;
        case SDP_SERVICE_SEARCH_ATTRIBUTE_REQUEST:
            process_sdp_service_search_attribute_request(&rx2buf[8]);
            break;
        case SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE:
            process_sdp_service_search_attribute_response(&rx2buf[8]);
            break;
        }
    } else {
        switch (rx2buf[8]) {
        case L2CAP_CMD_CONNECTION_REQUEST:
            process_l2cap_connection_request(&rx2buf[8]);
            break;
        case L2CAP_CMD_CONNECTION_RESPONSE:
            process_l2cap_connection_response(&rx2buf[8]);
            break;
        case L2CAP_CMD_CONFIG_REQUEST:
            process_l2cap_config_request(&rx2buf[8]);
            break;
        case L2CAP_CMD_CONFIG_RESPONSE:
            process_l2cap_config_response(&rx2buf[8]);
            break;

        case HID_THDR_DATA_INPUT:
            handleHIDTHDRData(rx2buf);    // Pass the whole buffer...
            break;
        case L2CAP_CMD_COMMAND_REJECT:
            process_l2cap_command_reject(&rx2buf[8]);
            break;
        case L2CAP_CMD_DISCONNECT_REQUEST:
            process_l2cap_disconnect_request(&rx2buf[8]);
            break;
        }
    }

}

//=============================================================================
// Process tx_data and state
//=============================================================================
void BluetoothConnection::tx_data(uint8_t *data, uint16_t length)
{

#ifdef DEBUG_BT_VERBOSE
    DBGPrintf("tx_data callback (bluetooth): %d : ", pending_control_tx_);
    for (uint8_t i = 0; i < length; i++) DBGPrintf("%x ", data[i]);
    DBGPrintf("\n");
#endif
    switch (pending_control_tx_) {
    case STATE_TX_SEND_CONNECT_INT:
        delay(1);
        connection_rxid_++;
        sendl2cap_ConnectionRequest(device_connection_handle_, connection_rxid_, interrupt_dcid_, HID_INTR_PSM);
        pending_control_tx_ = 0;
        break;
    case STATE_TX_SEND_CONECT_RSP_SUCCESS:
        delay(1);
        // Tell the device we are ready
        sendl2cap_ConnectionResponse(device_connection_handle_, connection_rxid_++, control_dcid_, control_scid_, SUCCESSFUL);
        pending_control_tx_ = STATE_TX_SEND_CONFIG_REQ;
        break;
    case STATE_TX_SEND_CONFIG_REQ:
        delay(1);
        sendl2cap_ConfigRequest(device_connection_handle_, connection_rxid_, control_scid_);
        pending_control_tx_ = 0;
        break;
    case STATE_TX_SEND_CONECT_ISR_RSP_SUCCESS:
        delay(1);
        // Tell the device we are ready
        sendl2cap_ConnectionResponse(device_connection_handle_, connection_rxid_++, interrupt_dcid_, interrupt_scid_, SUCCESSFUL);
        pending_control_tx_ = STATE_TX_SEND_CONFIG_ISR_REQ;
        break;
    case STATE_TX_SEND_CONFIG_ISR_REQ:
        delay(1);
        sendl2cap_ConfigRequest(device_connection_handle_, connection_rxid_, interrupt_scid_);
        pending_control_tx_ = 0;
        break;
    case STATE_TX_SEND_CONECT_SDP_RSP_SUCCESS:
        delay(1);
        sendl2cap_ConnectionResponse(device_connection_handle_, connection_rxid_, sdp_dcid_, sdp_scid_, SUCCESSFUL);
        pending_control_tx_ = STATE_TX_SEND_CONFIG_SDP_REQ;
        break;
    case STATE_TX_SEND_CONFIG_SDP_REQ:
        delay(1);
        sendl2cap_ConfigRequest(device_connection_handle_, connection_rxid_, sdp_scid_);
        pending_control_tx_ = 0;
        break;
    }
}

//=============================================================================
// Moved handling of L2CAP and HID messages
//=============================================================================
// Experiment to see if we can get SDP connectin setup
void BluetoothConnection::connectToSDP() {
    DBGPrintf("$$$ BluetoothController::connectToSDP() Called\n");
    connection_rxid_++;
    sendl2cap_ConnectionRequest(device_connection_handle_, connection_rxid_,
                                sdp_dcid_, SDP_PSM);
    pending_control_tx_ = 0;
}


void  BluetoothConnection::process_l2cap_connection_request(uint8_t *data) {
    //       ID   LEN  LEN PSM  PSM  SCID SCID
    // 0x02 0x02 0x04 0x00 0x11 0x00 0x43 0x00

    uint16_t psm = data[4] + ((uint16_t)data[5] << 8);
    uint16_t scid = data[6] + ((uint16_t)data[7] << 8);
    connection_started_ = true;
    connection_rxid_ = data[1];
    DBGPrintf("    L2CAP Connection Request: ID: %d, PSM: %x, SCID: %x\n", connection_rxid_, psm, scid);

    // Assuming not pair mode Send response like:
    //      RXID Len  LEN  DCID DCID  SCID SCID RES 0     0    0
    // 0x03 0x02 0x08 0x00 0x70 0x00 0x43 0x00 0x01 0x00 0x00 0x00
    if (psm == HID_CTRL_PSM) {
        control_scid_ = scid;
        sendl2cap_ConnectionResponse(device_connection_handle_, connection_rxid_, control_dcid_, control_scid_, PENDING);
        pending_control_tx_ = STATE_TX_SEND_CONECT_RSP_SUCCESS;
    } else if (psm == HID_INTR_PSM) {
        interrupt_scid_ = scid;
        sendl2cap_ConnectionResponse(device_connection_handle_, connection_rxid_, interrupt_dcid_, interrupt_scid_, PENDING);
        pending_control_tx_ = STATE_TX_SEND_CONECT_ISR_RSP_SUCCESS;

    } else if (psm == SDP_PSM) {
        DBGPrintf("        <<< SDP PSM >>>\n");
        sdp_scid_ = scid;
        sendl2cap_ConnectionResponse(device_connection_handle_, connection_rxid_, sdp_dcid_, sdp_scid_, PENDING);
        pending_control_tx_ = STATE_TX_SEND_CONECT_SDP_RSP_SUCCESS;
    }
}

// Process the l2cap_connection_response...
void BluetoothConnection::process_l2cap_connection_response(uint8_t *data) {

    uint16_t scid = data[4] + ((uint16_t)data[5] << 8);
    uint16_t dcid = data[6] + ((uint16_t)data[7] << 8);
    uint16_t result = data[8] + ((uint16_t)data[9] << 8);

    DBGPrintf("    L2CAP Connection Response: ID: %d, Dest:%x, Source:%x, Result:%x, Status: %x pending control: %x %x\n",
              data[1], scid, dcid,
              result, data[10] + ((uint16_t)data[11] << 8), btController_->pending_control_, pending_control_tx_);

    // Experiment ignore if: pending_control_tx_ = STATE_TX_SEND_CONFIG_REQ;
    if ((pending_control_tx_ == STATE_TX_SEND_CONFIG_REQ) || (pending_control_tx_ == STATE_TX_SEND_CONECT_RSP_SUCCESS)) {
        DBGPrintf("    *** State == STATE_TX_SEND_CONFIG_REQ - try ignoring\n");
        return;
    }

    //48 20 10 0 | c 0 1 0 | 3 0 8 0 44 0 70 0 0 0 0 0
    if (dcid == interrupt_dcid_) {
        interrupt_scid_ = scid;
        DBGPrintf("      Interrupt Response\n");
        connection_rxid_++;
        sendl2cap_ConfigRequest(device_connection_handle_, connection_rxid_, scid);
    } else if (dcid == control_dcid_) {
        control_scid_ = scid;
        DBGPrintf("      Control Response\n");
        sendl2cap_ConfigRequest(device_connection_handle_, connection_rxid_, scid);
    } else if (dcid == sdp_dcid_) {
        // Check for failure!
        DBGPrintf("      SDP Response\n");
        if (result != 0) {
            DBGPrintf("      Failed - No SDP!\n");
            // Enable SCan to page mode
            sdp_connected_ = false;
            connection_complete_ |= CCON_SDP;
            if (connection_complete_ == CCON_ALL)
                btController_->sendHCIWriteScanEnable(2);
        } else {
            sdp_scid_ = scid;
            sendl2cap_ConfigRequest(device_connection_handle_, connection_rxid_, scid);
        }
    }
}

void BluetoothConnection::process_l2cap_config_request(uint8_t *data) {
    //48 20 10 0 c 0 1 0 *4 2 8 0 70 0 0 0 1 2 30 0
    uint16_t dcid = data[4] + ((uint16_t)data[5] << 8);
    DBGPrintf("    L2CAP config Request: ID: %d, Dest:%x, Flags:%x,  Options: %x %x %x %x\n",
              data[1], dcid, data[6] + ((uint16_t)data[7] << 8),
              data[8], data[9], data[10], data[11]);
    // Now see which dest was specified
    if (dcid == control_dcid_) {
        DBGPrintf("      Control Configuration request\n");
        sendl2cap_ConfigResponse(device_connection_handle_, data[1], control_scid_);
    } else if (dcid == interrupt_dcid_) {
        DBGPrintf("      Interrupt Configuration request\n");
        sendl2cap_ConfigResponse(device_connection_handle_, data[1], interrupt_scid_);
    } else if (dcid == sdp_dcid_) {
        DBGPrintf("      SDP Configuration request\n");
        sendl2cap_ConfigResponse(device_connection_handle_, data[1], sdp_scid_);
    }
}

void BluetoothConnection::process_l2cap_config_response(uint8_t *data) {
    // 48 20 12 0 e 0 1 0 5 0 a 0 70 0 0 0 0 0 1 2 30 0
    uint16_t scid = data[4] + ((uint16_t)data[5] << 8);
    DBGPrintf("    L2CAP config Response: ID: %d, Source:%x, Flags:%x, Result:%x, Config: %x\n",
              data[1], scid, data[6] + ((uint16_t)data[7] << 8),
              data[8] + ((uint16_t)data[9] << 8), data[10] + ((uint16_t)data[11] << 8));
    if (scid == control_dcid_) {
        // Set HID Boot mode
        // Don't do if PS3... Or if class told us not to
        if (use_hid_protocol_) {
            // see what happens if I tell it to
            btController_->setHIDProtocol(HID_RPT_PROTOCOL);

        } else {
            // don't call through Null pointer
            if ((device_driver_ == nullptr) ||
                    !(device_driver_->special_process_required & BTHIDInput::SP_PS3_IDS)) {
                btController_->setHIDProtocol(HID_BOOT_PROTOCOL);  //
            }
        }
        //setHIDProtocol(HID_RPT_PROTOCOL);  //HID_RPT_PROTOCOL
        if (btController_->do_pair_device_ && !(device_driver_ && (device_driver_->special_process_required & BTHIDInput::SP_DONT_NEED_CONNECT))) {
            pending_control_tx_ = STATE_TX_SEND_CONNECT_INT;
        } else if (device_driver_ && (device_driver_->special_process_required & BTHIDInput::SP_NEED_CONNECT)) {
            DBGPrintf("   Needs connect to device INT(PS4?)\n");
            // The PS4 requires a connection request to it.
            pending_control_tx_ = STATE_TX_SEND_CONNECT_INT;
        } else {
            btController_->pending_control_ = 0;
        }
        connection_complete_ |= CCON_CONT;
    } else if (scid == interrupt_dcid_) {
        // Lets try SDP connectin
        connectToSDP(); // temp to see if we can do this later...

        // Enable SCan to page mode
        //connection_complete_ = true;
        //sendHCIWriteScanEnable(2);
        connection_complete_ |= CCON_INT;
    } else if (scid == sdp_dcid_) {
        // Enable SCan to page mode
        DBGPrintf("    SDP configuration complete\n");
        // Enable SCan to page mode
        connection_complete_ |= CCON_SDP;
        sdp_connected_ = true;
    }

    if (connection_complete_ == CCON_ALL) {
        btController_->sendHCIWriteScanEnable(2);
    }
}

void BluetoothConnection::process_l2cap_command_reject(uint8_t *data) {
    // 48 20 b 0 7 0 70 0 *1 0 0 0 2 0 4
    DBGPrintf("    L2CAP command reject: ID: %d, length:%x, Reason:%x,  Data: %x %x \n",
              data[1], data[2] + ((uint16_t)data[3] << 8), data[4], data[5], data[6]);

}

void BluetoothConnection::process_l2cap_disconnect_request(uint8_t *data) {
    uint16_t dcid = data[4] + ((uint16_t)data[5] << 8);
    uint16_t scid = data[6] + ((uint16_t)data[7] << 8);
    DBGPrintf("    L2CAP disconnect request: ID: %d, Length:%x, Dest:%x, Source:%x\n",
              data[1], data[2] + ((uint16_t)data[3] << 8), dcid, scid);
    // May need to respond in some cases...
    if (dcid == control_dcid_) {
        DBGPrintf("      Control disconnect request\n");
    } else if (dcid == interrupt_dcid_) {
        DBGPrintf("      Interrupt disconnect request\n");
    } else if (dcid == sdp_dcid_) {
        DBGPrintf("      SDP disconnect request\n");
        sdp_connected_ = false; // say we are not connected
        sendl2cap_DisconnectResponse(device_connection_handle_, data[1],
                                     sdp_dcid_,
                                     sdp_scid_);
    }

}

void BluetoothConnection::handleHIDTHDRData(uint8_t *data) {
    // Example
    //                      T HID data
    //48 20 d 0 9 0 71 0 a1 3 8a cc c5 a 23 22 79
    uint16_t len = data[4] + ((uint16_t)data[5] << 8);
    DBGPrintf("HID HDR Data: len: %d, Type: %d Con:%p\n", len, data[9], this);

    // ??? How to parse??? Use HID object???
    if (device_driver_) {
        device_driver_->process_bluetooth_HID_data(&data[9], len - 1); // We skip the first byte...
    } else if (have_hid_descriptor_) {
        // nead to bias to location within data.
        parse(0x0100 | data[9], &data[10], len - 2);

    } else {
        switch (data[9]) {
        case 1:
            DBGPrintf("    Keyboard report type\n");
            break;
        case 2:
            DBGPrintf("    Mouse report type\n");
            break;
        case 3:
            DBGPrintf("    Combo keyboard/pointing\n");
            break;
        default:
            DBGPrintf("    Unknown report\n");
        }
    }
}

void BluetoothConnection::handle_HCI_WRITE_SCAN_ENABLE_complete(uint8_t *rxbuf)
{
    // See if we have driver and a remote
    DBGPrintf("Write_Scan_enable Completed - connection state: %x\n", connection_complete_);
    if (connection_complete_ == CCON_ALL) {    // We have a driver call their
        if (device_driver_) {
            device_driver_->connectionComplete();
        } else if (check_for_hid_descriptor_) {
            have_hid_descriptor_ = false;
            sdp_buffer_ = descriptor_;
            sdp_buffer_len_ = sizeof(descriptor_);

            if (!startSDP_ServiceSearchAttributeRequest(0x206, 0x206, sdp_buffer_, sdp_buffer_len_)) {
                // Maybe try to claim_driver as we won't get a HID report.
                DBGPrintf("Failed to start SDP attribute request - so lets try again to find a driver");
                device_driver_ = find_driver(nullptr, 1);
            }
        }
        connection_complete_ = 0;  // only call once
    }

}

void BluetoothConnection::handle_HCI_OP_ROLE_DISCOVERY_complete(uint8_t *rxbuf)
{
    // PS4 looks something like: 0E 07 01 09 08 00 47 00 01
    // Others look like        : 0E 07 01 09 08 00 48 00 00
    // last byte is the interesting one says what role. 
    DBGPrintf("ROLE_DISCOVERY Completed - Role: %x ", rxbuf[8]);
    if (rxbuf[8] == 0) {
        DBGPrintf("(Central)\n");

        // Set a timeout
        btController_->setTimer(this, CONNECTION_TIMEOUT_US);

    } else {
        DBGPrintf("(Peripheral)\n");

        // Should maybe do some state checking before doing this, but...
        DBGPrintf("\t** We will issue connection requsts **\n ");
        connection_started_ = true;
        sendl2cap_ConnectionRequest(device_connection_handle_, connection_rxid_, control_dcid_, HID_CTRL_PSM);
    }
}

void BluetoothConnection::timer_event()
{
    DBGPrintf("BluetoothConnection::timer_event(%p) : %u\n", this, connection_started_);

    if (!connection_started_) {
        DBGPrintf("\t** Timed out now try issue connection requsts **\n ");
        connection_started_ = true;
        sendl2cap_ConnectionRequest(device_connection_handle_, connection_rxid_, control_dcid_, HID_CTRL_PSM);
    }
}    

// l2cap support functions.
void BluetoothConnection::sendl2cap_ConnectionResponse(uint16_t handle, uint8_t rxid, uint16_t dcid, uint16_t scid, uint8_t result) {
    uint8_t l2capbuf[12];
    l2capbuf[0] = L2CAP_CMD_CONNECTION_RESPONSE; // Code
    l2capbuf[1] = rxid; // Identifier
    l2capbuf[2] = 0x08; // Length
    l2capbuf[3] = 0x00;
    l2capbuf[4] = dcid & 0xff; // Destination CID
    l2capbuf[5] = dcid >> 8;
    l2capbuf[6] = scid & 0xff; // Source CID
    l2capbuf[7] = scid >> 8;
    l2capbuf[8] = result; // Result: Pending or Success
    l2capbuf[9] = 0x00;
    l2capbuf[10] = 0x00; // No further information
    l2capbuf[11] = 0x00;

    DBGPrintf("L2CAP_CMD_CONNECTION_RESPONSE(RXID:%x, DCID:%x, SCID:%x RES:%x)\n", rxid, dcid, scid, result);
    btController_->sendL2CapCommand(handle, l2capbuf, sizeof(l2capbuf));
}



void BluetoothConnection::sendl2cap_ConnectionRequest(uint16_t handle, uint8_t rxid, uint16_t scid, uint16_t psm) {
    uint8_t l2capbuf[8];
    l2capbuf[0] = L2CAP_CMD_CONNECTION_REQUEST; // Code
    l2capbuf[1] = rxid; // Identifier
    l2capbuf[2] = 0x04; // Length
    l2capbuf[3] = 0x00;
    l2capbuf[4] = (uint8_t)(psm & 0xff); // PSM
    l2capbuf[5] = (uint8_t)(psm >> 8);
    l2capbuf[6] = scid & 0xff; // Source CID
    l2capbuf[7] = (scid >> 8) & 0xff;

    DBGPrintf("ConnectionRequest (RXID:%x, SCID:%x, PSM:%x)\n", rxid, scid, psm);
    btController_->sendL2CapCommand(handle, l2capbuf, sizeof(l2capbuf));
}

void BluetoothConnection::sendl2cap_ConfigRequest(uint16_t handle, uint8_t rxid, uint16_t dcid) {
    uint8_t l2capbuf[12];
    l2capbuf[0] = L2CAP_CMD_CONFIG_REQUEST; // Code
    l2capbuf[1] = rxid; // Identifier
    l2capbuf[2] = 0x08; // Length
    l2capbuf[3] = 0x00;
    l2capbuf[4] = dcid & 0xff; // Destination CID
    l2capbuf[5] = (dcid >> 8) & 0xff;
    l2capbuf[6] = 0x00; // Flags
    l2capbuf[7] = 0x00;
    l2capbuf[8] = 0x01; // Config Opt: type = MTU (Maximum Transmission Unit) - Hint
    l2capbuf[9] = 0x02; // Config Opt: length
    l2capbuf[10] = 0xFF; // MTU
    l2capbuf[11] = 0xFF;

    DBGPrintf("L2CAP_ConfigRequest(RXID:%x, DCID:%x)\n", rxid, dcid);
    btController_->sendL2CapCommand(handle, l2capbuf, sizeof(l2capbuf));
}

void BluetoothConnection::sendl2cap_ConfigResponse(uint16_t handle, uint8_t rxid, uint16_t scid) {
    uint8_t l2capbuf[14];
    l2capbuf[0] = L2CAP_CMD_CONFIG_RESPONSE; // Code
    l2capbuf[1] = rxid; // Identifier
    l2capbuf[2] = 0x0A; // Length
    l2capbuf[3] = 0x00;
    l2capbuf[4] = scid & 0xff; // Source CID
    l2capbuf[5] = (scid >> 8) & 0xff;
    l2capbuf[6] = 0x00; // Flag
    l2capbuf[7] = 0x00;
    l2capbuf[8] = 0x00; // Result
    l2capbuf[9] = 0x00;
    l2capbuf[10] = 0x01; // Config
    l2capbuf[11] = 0x02;
    l2capbuf[12] = 0xA0;
    l2capbuf[13] = 0x02;

    DBGPrintf("L2CAP_ConfigResponse(RXID:%x, SCID:%x)\n", rxid, scid);
    btController_->sendL2CapCommand(handle, l2capbuf, sizeof(l2capbuf));
}

void BluetoothConnection::sendl2cap_DisconnectResponse(uint16_t handle, uint8_t rxid, uint16_t dcid, uint16_t scid) {
    uint8_t l2capbuf[8];
    l2capbuf[0] = L2CAP_CMD_DISCONNECT_RESPONSE; // Code
    l2capbuf[1] = rxid; // Identifier
    l2capbuf[2] = 0x04; // Length
    l2capbuf[3] = 0x00;
    l2capbuf[4] = dcid & 0xff; // dcid CID
    l2capbuf[5] = (dcid >> 8) & 0xff;
    l2capbuf[6] = scid & 0xff; // SCID CID
    l2capbuf[7] = (scid >> 8) & 0xff;

    DBGPrintf("L2CAP_DisconnectResponse(RXID:%x, DCID:%x, SCID:%x)\n", rxid, dcid, scid);
    btController_->sendL2CapCommand(handle, l2capbuf, sizeof(l2capbuf));
}

//=============================================================================
// Process the SDP stuff.
//=============================================================================
bool BluetoothConnection::startSDP_ServiceSearchAttributeRequest(uint16_t range_low, uint16_t range_high, uint8_t *buffer, uint32_t cb)
{
    if (!sdp_connected_) return false;

    sdp_request_range_low_ = range_low;
    sdp_reqest_range_high_ = range_high;
    sdp_request_buffer_ = buffer;
    sdp_request_buffer_cb_ = cb;
    sdp_request_buffer_used_cnt_ = 0; // cnt in bytes used.
    sdp_request_completed_ = false;

    // So start it up
    send_SDP_ServiceSearchAttributeRequest(nullptr, 0);
    return true;
}


void BluetoothConnection::send_SDP_ServiceSearchRequest(uint8_t *continue_state, uint8_t cb)
{
    // Example message: first part l2cap. Which will let them fill that in
    //  l2cap: 0x47 0x0 0x18 0x0 0x14 0x0 0x42 0x0
    //  PSM header: 0x6 0x0 0x0 0x0 0xf
    //  req: 0x35 0x3 0x19 0x1 0x0 0xff 0xff 0x35 0x5 0xa 0x0 0x0 0xff 0xff 0x0

    uint8_t sdpcmdbuf[30]; // 20 + up to 10 continue
    seq_number_++;
    uint16_t packet_size = (13 - 5) + cb;

    sdpcmdbuf[0] = 0x2; // SDP_ServiceSearchRequest
    sdpcmdbuf[1] = seq_number_ >> 8;   // Sequence number
    sdpcmdbuf[2] = seq_number_ & 0xff; //
    sdpcmdbuf[3] = packet_size >> 8;   // Data in the rest...
    sdpcmdbuf[4] = packet_size & 0xff; //

    // Now lets build the attribute request data.
    // 0x35 0x3 0x19 0x1 0x0 0xff 0xff 0x35 0x5 0xa 0x0 0x0 0xff 0xff 0x2 0x0 0x26

    sdpcmdbuf[5] = 0x35; // type sequence
    sdpcmdbuf[6] = 0x03;  //   3 bytes
    sdpcmdbuf[7] = 0x19;  //  UUID 2 bytes
    sdpcmdbuf[8] = 0x01;  //   UUID
    sdpcmdbuf[9] = 0x00;  //   UUID
    sdpcmdbuf[10] = 0xff;  //  MAX attributes
    sdpcmdbuf[11] = 0xff; //
    sdpcmdbuf[12] = cb;    // Count of continuation bytes
    for (uint8_t i = 0; i < cb; i++) sdpcmdbuf[13 + i] = continue_state[i];

    // Now lets try to send the packet
    btController_->sendL2CapCommand(sdpcmdbuf, 13 + cb, sdp_scid_);
}

void BluetoothConnection::send_SDP_ServiceSearchAttributeRequest(uint8_t *continue_state, uint8_t cb)
{
    // Example message: first part l2cap. Which will let them fill that in
    //  l2cap: 0x47 0x0 0x18 0x0 0x14 0x0 0x42 0x0
    //  PSM header: 0x6 0x0 0x0 0x0 0xf
    //  req: 0x35 0x3 0x19 0x1 0x0 0xff 0xff 0x35 0x5 0xa 0x0 0x0 0xff 0xff 0x0

    uint16_t continue_data_offset = 17;
    uint8_t sdpcmdbuf[37]; // 20 + up to 17 continue
    seq_number_++;


    sdpcmdbuf[0] = 0x6; // SDP_ServiceSearchAttributeReques
    sdpcmdbuf[1] = seq_number_ >> 8;   // Sequence number
    sdpcmdbuf[2] = seq_number_ & 0xff; //
    //sdpcmdbuf[3] = packet_size >> 8;   // Data in the rest...
    //sdpcmdbuf[4] = packet_size & 0xff; //

    // Now lets build the attribute request data.
    // 0x35 0x3 0x19 0x1 0x0 0xff 0xff 0x35 0x5 0xa 0x0 0x0 0xff 0xff 0x2 0x0 0x26

    sdpcmdbuf[5] = 0x35; // type sequence
    sdpcmdbuf[6] = 0x03;  //   3 bytes
    sdpcmdbuf[7] = 0x19;  //  UUID 2 bytes
    sdpcmdbuf[8] = 0x01;  //   UUID
    sdpcmdbuf[9] = 0x00;  //   UUID
    sdpcmdbuf[10] = 0xff;  //  MAX size
    sdpcmdbuf[11] = 0xff; //
    sdpcmdbuf[12] = 0x35; // Sequence

    if (sdp_request_range_low_ == sdp_reqest_range_high_) {
        // doing specific
        sdpcmdbuf[13] = 0x03;  //  3 bytes
        sdpcmdbuf[14] = 0x09;  //  2 byte integer
        sdpcmdbuf[15] = sdp_request_range_low_ >> 8;  //    Attribute low
        sdpcmdbuf[16] = sdp_request_range_low_ & 0xff;  //
    } else {
        // doing range
        sdpcmdbuf[13] = 0x05;  //  5 bytes
        sdpcmdbuf[14] = 0x0A;  //  4 byte integer
        sdpcmdbuf[15] = sdp_request_range_low_ >> 8;  //    Attribute low
        sdpcmdbuf[16] = sdp_request_range_low_ & 0xff;  //
        sdpcmdbuf[17] = sdp_reqest_range_high_ & 0xff;  //    high
        sdpcmdbuf[18] = sdp_reqest_range_high_ & 0xff;  //    high
        continue_data_offset = 19;
    }
    sdpcmdbuf[continue_data_offset++] = cb;    // Count of continuation bytes
    for (uint8_t i = 0; i < cb; i++) sdpcmdbuf[continue_data_offset + i] = continue_state[i];

    uint16_t packet_size = (continue_data_offset - 5) + cb;
    sdpcmdbuf[3] = packet_size >> 8;   // Data in the rest...
    sdpcmdbuf[4] = packet_size & 0xff; //

    // Now lets try to send the packet
    btController_->sendL2CapCommand(sdpcmdbuf, continue_data_offset + cb, sdp_scid_);
}

//----------------------------------------------------------------
// Some SDP stuff.
void BluetoothConnection::process_sdp_service_search_request(uint8_t *data) {
    DBGPrintf("process_sdp_service_search_request\n");
}

void BluetoothConnection::process_sdp_service_search_response(uint8_t *data) {
    DBGPrintf("process_sdp_service_search_response\n");

    // (processed before us)(0)  48 20 1A 00 16 00 40 00
    // (0)  03 00 01 00 11
    // (5) 00 03 00 03 00 00 00 00 00 01 00 00 00 01 00 01 00
    uint16_t total_count = (data[5] << 8) + data[6];
    uint16_t current_count = (data[7] << 8) + data[8];
    uint8_t offset = 9;
    DBGPrintf("\tTotal Count:%u Current:%u", total_count, current_count);
    for (uint8_t i = 0; i < current_count; i++) {
        uint32_t srh = ((uint32_t)data[offset + 0] << 24) + ((uint32_t)data[offset + 1] << 16) +
                       ((uint32_t)data[offset + 2] << 8) + (data[offset + 3]);
        DBGPrintf(" %08X", srh);
        offset += 4;
    }
    uint8_t continue_state_count = data[offset++];
    DBGPrintf(" Cont CB:%u", continue_state_count);
    for (uint8_t i = 0; i < continue_state_count; i++) DBGPrintf(" %02X",  data[offset++]);
    DBGPrintf("\n");
}

void BluetoothConnection::process_sdp_service_attribute_request(uint8_t *data) {
    DBGPrintf("process_sdp_service_attribute_request\n");

}

void BluetoothConnection::process_sdp_service_attribute_response(uint8_t *data) {
    DBGPrintf("process_sdp_service_attribute_response:\n");
}

void BluetoothConnection::process_sdp_service_search_attribute_request(uint8_t *data) {
    DBGPrintf("\n### process_sdp_service_search_attribute_request ###\n");
    // Print out the data like UHS2
#ifdef DEBUG_BT
    uint16_t service = data[1] << 8 | data[2];
    uint16_t length = data[3] << 8 | data[4];
    uint16_t uuid = (data[8] << 8 | data[9]);
    if (uuid == 0x0000) // Check if it's sending the UUID as a 128-bit UUID
        uuid = (data[10] << 8 | data[11]);

    DBGPrintf("  Service:%x UUID:%x Length:%u\n", service, uuid, length);
    DBGPrintf("  Data:");
    for (uint8_t i = 0; i < length; i++) {
        DBGPrintf("%02x ", data[5 + i]);
    }
    DBGPrintf("\n");
#endif
    // lets respond saying we don't support the service
    uint8_t l2capbuf[10];
    l2capbuf[0] = SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE;
    l2capbuf[1] = data[1];
    l2capbuf[2] = data[2];
    l2capbuf[3] = 0x00; // MSB Parameter Length
    l2capbuf[4] = 0x05; // LSB Parameter Length = 5
    l2capbuf[5] = 0x00; // MSB AttributeListsByteCount
    l2capbuf[6] = 0x02; // LSB AttributeListsByteCount = 2

    /* Attribute ID/Value Sequence: */
    l2capbuf[7] = 0x35; // Data element sequence - length in next byte
    l2capbuf[8] = 0x00; // Length = 0
    l2capbuf[9] = 0x00; // No continuation state

    DBGPrintf("Send SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE not supported\n");
    btController_->sendL2CapCommand(device_connection_handle_, l2capbuf, sizeof(l2capbuf),
                                    sdp_scid_ & 0xff, sdp_scid_ >> 8);

}

void BluetoothConnection::process_sdp_service_search_attribute_response(uint8_t *data) {
    //before :0B 20 34 00 30 00 40 00
    // (00) 07 00 01 00 2B
    // (5) 00 26  // cb_data
    // (7) 36 03 A2 36 00 8E 09 00 00 0A 00 00 00 00 09 00 01 35 03 19 10 00 09 00 04 35 0D 35 06 19 01 00 09 00 01 35 03 19 // data
    // (7 + cb_data) 02 00 26 //

    uint16_t cb_data = (data[5] << 8) + data[6];
    uint8_t cb_cont = data[7 + cb_data];
    uint8_t *pb_cont = &data[8 + cb_data];

    DBGPrintf("process_sdp_service_search_attribute_response: cb dat:%u cont:%u ", cb_data, cb_cont);
    for (uint8_t i = 0; i < cb_cont; i++) DBGPrintf(" %02X", pb_cont[i]);
    DBGPrintf("\n");

    // lets copy the data in...
    uint16_t cb_space = sdp_request_buffer_cb_ - sdp_request_buffer_used_cnt_;
    if (cb_data > cb_space) cb_data = cb_space;
    memcpy(&sdp_request_buffer_[sdp_request_buffer_used_cnt_], &data[7], cb_data);
    sdp_request_buffer_used_cnt_ += cb_data;

    // Now see if we are done or not, if not start up next request
    if ((cb_cont == 0) || (sdp_request_buffer_used_cnt_ == sdp_request_buffer_cb_)) {
        sdp_request_completed_ = true;
        if (device_driver_) {
            device_driver_->sdp_command_completed(true); // We skip the first byte...
        } else {
            // Must be our own... we should now try to proces it.
            completeSDPRequest(true);
        }
    } else {
        send_SDP_ServiceSearchAttributeRequest(pb_cont, cb_cont);

    }
}

//=============================================================================
// HID SUPPORT - maybe split out
//=============================================================================
// lets use the SDP Support functions to try to retrieve the
// HID Descriptor using SDP
//=============================================================================

bool BluetoothConnection::completeSDPRequest(bool success)
{

    if (!success) return false;
    // Now real hack:
    // Lets see if we can now print out the report descriptor.
    uint32_t cb_left = sdp_request_buffer_used_cnt_;
    uint8_t *pb = sdp_buffer_; // start at second byte;

    sdp_element_t sdpe;
    while (cb_left > 0) {
        int cb = extract_next_SDP_Token(pb, cb_left, sdpe);
        if (cb < 0 ) break;
        // Should do a lot more validation, but ...
        if ((sdpe.element_type == 4) && (sdpe.dtype == DPB)) {
            descsize_ = sdpe.element_size;
            memcpy(descriptor_, sdpe.data.pb, descsize_);
            dumpHIDReportDescriptor();
            have_hid_descriptor_ = true;
            parse();
            return true;
        }

        cb_left -= cb;
        pb += cb;
    }
    return false;
}




//BUGBUG: move to class or ...

int BluetoothConnection::extract_next_SDP_Token(uint8_t *pbElement, int cb_left, sdp_element_t &sdpe) {
    uint8_t element = *pbElement; // first byte is type of element
    sdpe.element_type = element >> 3;
    sdpe.element_size = element & 7;
    sdpe.data.luw = 0; // start off 0

    switch (element) {
    case 0: // nil
        sdpe.dtype = DNIL;
        return 1; // one byte used.

    case 0x08: // unsigned one byte
    case 0x18: // UUID one byte
    case 0x28: // bool one byte
        sdpe.dtype = DU32;
        sdpe.data.uw = pbElement[1];
        return 2;

    case 0x09: // unsigned 2  byte
    case 0x19: // uuid 2  byte
        sdpe.dtype = DU32;
        sdpe.data.uw = (pbElement[1] << 8) + pbElement[2];
        return 3;
    case 0x0A: // unsigned 4  byte
    case 0x1A: // UUID 4  byte
        sdpe.dtype = DU32;
        sdpe.data.uw =  (uint32_t)(pbElement[1] << 24) + (uint32_t)(pbElement[2] << 16) + (pbElement[3] << 8) + pbElement[4];
        return 5;
    case 0x0B: // unsigned 8  byte
        sdpe.dtype = DU64;
        sdpe.data.luw =  ((uint64_t)pbElement[1] << 52) + ((uint64_t)pbElement[2] << 48) + ((uint64_t)pbElement[3] << 40) + ((uint64_t)pbElement[4] << 32) +
                         (uint32_t)(pbElement[5] << 24) + (uint32_t)(pbElement[6] << 16) + (pbElement[7] << 8) + pbElement[8];
        return 9;

    // type = 2 signed
    case 0x10: // unsigned one byte
        sdpe.dtype = DS32;
        sdpe.data.sw = (int8_t)pbElement[1];
        return 2;
    case 0x11: // unsigned 2  byte
        sdpe.dtype = DS32;
        sdpe.data.sw = (int16_t)((pbElement[1] << 8) + pbElement[2]);
        return 3;

    case 0x12: // unsigned 4  byte
        sdpe.dtype = DS32;
        sdpe.data.sw =  (int32_t)((uint32_t)(pbElement[1] << 24) + (uint32_t)(pbElement[2] << 16) + (pbElement[3] << 8) + pbElement[4]);
        return 5;
    case 0x13: //
        sdpe.dtype = DS64;
        sdpe.data.lsw =  (int64_t)(((uint64_t)pbElement[1] << 52) + ((uint64_t)pbElement[2] << 48) + ((uint64_t)pbElement[3] << 40) + ((uint64_t)pbElement[4] << 32) +
                                   (uint32_t)(pbElement[5] << 24) + (uint32_t)(pbElement[6] << 16) + (pbElement[7] << 8) + pbElement[8]);
        return 9;

    // string one byte size.
    case 0x25:
        sdpe.dtype = DPB;
        sdpe.element_size = pbElement[1];
        sdpe.data.pb = &pbElement[2];
        return sdpe.element_size + 2;

    case 0x26:
        sdpe.dtype = DPB;
        sdpe.element_size = (pbElement[1] << 8) + pbElement[2];
        sdpe.data.pb = &pbElement[3];
        return sdpe.element_size + 3;

    // type = 7 Data element sequence
    case 0x35: //
    case 0x3D: //
        sdpe.dtype = DLVL;
        sdpe.element_size = pbElement[1];
        sdpe.data.pb = &pbElement[2];
        return 2;
    case 0x36: //
    case 0x3E: //
        sdpe.dtype = DLVL;
        sdpe.element_size = (pbElement[1] << 8) + pbElement[2];
        sdpe.data.pb = &pbElement[3];
        return 3;
    case 0x37: //
    case 0x3F: //
        sdpe.dtype = DLVL;
        sdpe.element_size = (uint32_t)(pbElement[1] << 24) + (uint32_t)(pbElement[2] << 16) + (pbElement[3] << 8) + pbElement[4];
        sdpe.data.pb = &pbElement[3];
        return 5;
    default:
        DBGPrintf("### DECODE failed %x ###\n", element);
        return -1;

    }

}




void BluetoothConnection::dumpHIDReportDescriptor() {
#ifdef DEBUG_BT_VERBOSE
    uint8_t *pb = descriptor_;
    uint16_t cb = descsize_;
    const uint8_t *p = pb;
    uint16_t report_size = cb;

    const uint8_t *pend = p + report_size;
    uint8_t collection_level = 0;
    uint16_t usage_page = 0;
    enum { USAGE_LIST_LEN = 24 };
    uint16_t usage[USAGE_LIST_LEN] = { 0, 0 };
    uint8_t usage_count = 0;
    uint32_t topusage;
    //cnt_feature_reports_ = 0;
    //uint8_t last_report_id = 0;
    DBGPrintf("\nHID Report Descriptor (%p) size: %u\n", p, report_size);
    while (p < pend) {
        uint8_t tag = *p;
        for (uint8_t i = 0; i < collection_level; i++) DBGPrintf("  ");
        DBGPrintf("  %02X", tag);

        if (tag == 0xFE) {  // Long Item (unsupported)
            p += p[1] + 3;
            continue;
        }
        uint32_t val;
        switch (tag & 0x03) {  // Short Item data
        case 0:
            val = 0;
            p++;
            break;
        case 1:
            val = p[1];
            // could be better;
            DBGPrintf(" %02X", p[1]);
            p += 2;
            break;
        case 2:
            val = p[1] | (p[2] << 8);
            DBGPrintf(" %02X %02X", p[1], p[2]);
            p += 3;
            break;
        case 3:
            val = p[1] | (p[2] << 8) | (p[3] << 16) | (p[4] << 24);
            DBGPrintf(" %02X %02X %02X %02X", p[1], p[2], p[3], p[4]);
            p += 5;
            break;
        }
        if (p > pend) break;

        bool reset_local = false;
        switch (tag & 0xfc) {
        case 0x4:  //usage Page
        {
            usage_page = val;
            DBGPrintf("\t// Usage Page(%x) - ", val);
            switch (usage_page) {
            case 0x01: DBGPrintf("Generic Desktop"); break;
            case 0x06: DBGPrintf("Generic Device Controls"); break;
            case 0x07: DBGPrintf("Keycode"); break;
            case 0x08: DBGPrintf("LEDs"); break;
            case 0x09: DBGPrintf("Button"); break;
            case 0x0C: DBGPrintf("Consumer"); break;
            case 0x0D:
            case 0xFF0D: DBGPrintf("Digitizer"); break;
            default:
                if (usage_page >= 0xFF00) DBGPrintf("Vendor Defined");
                else DBGPrintf("Other ?");
                break;
            }
        }
        break;
        case 0x08:  //usage
            DBGPrintf("\t// Usage(%x) -", val);
            //printUsageInfo(usage_page, val);
            if (usage_count < USAGE_LIST_LEN) {
                // Usages: 0 is reserved 0x1-0x1f is sort of reserved for top level things like
                // 0x1 - Pointer - A collection... So lets try ignoring these
                if (val > 0x1f) {
                    usage[usage_count++] = val;
                }
            }
            break;
        case 0x14:  // Logical Minimum (global)
            DBGPrintf("\t// Logical Minimum(%x)", val);
            break;
        case 0x24:  // Logical Maximum (global)
            DBGPrintf("\t// Logical maximum(%x)", val);
            break;
        case 0x74:  // Report Size (global)
            DBGPrintf("\t// Report Size(%x)", val);
            break;
        case 0x94:  // Report Count (global)
            DBGPrintf("\t// Report Count(%x)", val);
            break;
        case 0x84:  // Report ID (global)
            DBGPrintf("\t// Report ID(%x)", val);
            //last_report_id = val;
            break;
        case 0x18:  // Usage Minimum (local)
            usage[0] = val;
            usage_count = 255;
            DBGPrintf("\t// Usage Minimum(%x) - ", val);
            //printUsageInfo(usage_page, val);
            break;
        case 0x28:  // Usage Maximum (local)
            usage[1] = val;
            usage_count = 255;
            DBGPrintf("\t// Usage Maximum(%x) - ", val);
            //printUsageInfo(usage_page, val);
            break;
        case 0xA0:  // Collection
            DBGPrintf("\t// Collection(%x)", val);
            // discard collection info if not top level, hopefully that's ok?
            if (collection_level == 0) {
                topusage = ((uint32_t)usage_page << 16) | usage[0];
                DBGPrintf(" top Usage(%x)", topusage);
                collection_level++;
            }
            reset_local = true;
            break;
        case 0xC0:  // End Collection
            DBGPrintf("\t// End Collection");
            if (collection_level > 0) collection_level--;
            break;

        case 0x80:  // Input
            DBGPrintf("\t// Input(%x)\t// (", val);
            print_input_output_feature_bits(val);
            reset_local = true;
            break;
        case 0x90:  // Output
            DBGPrintf("\t// Output(%x)\t// (", val);
            print_input_output_feature_bits(val);
            reset_local = true;
            break;
        case 0xB0:  // Feature
            DBGPrintf("\t// Feature(%x)\t// (", val);
            print_input_output_feature_bits(val);
            //if (cnt_feature_reports_ < MAX_FEATURE_REPORTS) {
            //  feature_report_ids_[cnt_feature_reports_++] = last_report_id;
            //}
            reset_local = true;
            break;

        case 0x34:  // Physical Minimum (global)
            DBGPrintf("\t// Physical Minimum(%x)", val);
            break;
        case 0x44:  // Physical Maximum (global)
            DBGPrintf("\t// Physical Maximum(%x)", val);
            break;
        case 0x54:  // Unit Exponent (global)
            DBGPrintf("\t// Unit Exponent(%x)", val);
            break;
        case 0x64:  // Unit (global)
            DBGPrintf("\t// Unit(%x)", val);
            break;
        }
        if (reset_local) {
            usage_count = 0;
            usage[0] = 0;
            usage[1] = 0;
        }

        DBGPrintf("\n");
    }
#endif
}

#ifdef DEBUG_BT_VERBOSE
void BluetoothConnection::print_input_output_feature_bits(uint8_t val) {
    DBGPrintf((val & 0x01) ? "Constant" : "Data");
    DBGPrintf((val & 0x02) ? ", Variable" : ", Array");
    DBGPrintf((val & 0x04) ? ", Relative" : ", Absolute");
    if (val & 0x08) DBGPrintf(", Wrap");
    if (val & 0x10) DBGPrintf(", Non Linear");
    if (val & 0x20) DBGPrintf(", No Preferred");
    if (val & 0x40) DBGPrintf(", Null State");
    if (val & 0x80) DBGPrintf(", Volatile");
    if (val & 0x100) DBGPrintf(", Buffered Bytes");
    DBGPrintf(")");
}
#endif

//=============================================================================
// Lets try copy of the HID Parse code and see what happens with with it.
//=============================================================================

// Extract 1 to 32 bits from the data array, starting at bitindex.
static uint32_t bitfield(const uint8_t *data, uint32_t bitindex, uint32_t numbits)
{
    uint32_t output = 0;
    uint32_t bitcount = 0;
    data += (bitindex >> 3);
    uint32_t offset = bitindex & 7;
    if (offset) {
        output = (*data++) >> offset;
        bitcount = 8 - offset;
    }
    while (bitcount < numbits) {
        output |= (uint32_t)(*data++) << bitcount;
        bitcount += 8;
    }
    if (bitcount > numbits && numbits < 32) {
        output &= ((1 << numbits) - 1);
    }
    return output;
}

// convert a number with the specified number of bits from unsigned to signed,
// so the result is a proper 32 bit signed integer.
static int32_t signext(uint32_t num, uint32_t bitcount)
{
    if (bitcount < 32 && bitcount > 0 && (num & (1 << (bitcount - 1)))) {
        num |= ~((1 << bitcount) - 1);
    }
    return (int32_t)num;
}

// convert a tag's value to a signed integer.
static int32_t signedval(uint32_t num, uint8_t tag)
{
    tag &= 3;
    if (tag == 1) return (int8_t)num;
    if (tag == 2) return (int16_t)num;
    return (int32_t)num;
}

// This no-inputs parse is meant to be used when we first get the
// HID report descriptor.  It finds all the top level collections
// and allows drivers to claim them.  This is always where we
// learn whether the reports will or will not use a Report ID byte.
void BluetoothConnection::parse()
{
    DBGPrintf("BluetoothConnection::parse() called\n");
    const uint8_t *p = descriptor_;
    const uint8_t *end = p + descsize_;
    uint16_t usage_page = 0;
    uint16_t usage = 0;
    uint8_t collection_level = 0;
    uint8_t topusage_count = 0;

    //bool use_report_id = false;
    while (p < end) {
        uint8_t tag = *p;
        if (tag == 0xFE) { // Long Item
            p += *p + 3;
            continue;
        }
        uint32_t val;
        switch (tag & 0x03) { // Short Item data
        case 0: val = 0;
            p++;
            break;
        case 1: val = p[1];
            p += 2;
            break;
        case 2: val = p[1] | (p[2] << 8);
            p += 3;
            break;
        case 3: val = p[1] | (p[2] << 8) | (p[3] << 16) | (p[4] << 24);
            p += 5;
            break;
        }
        if (p > end) break;

        switch (tag & 0xFC) {
        case 0x84: // Report ID (global)
            //use_report_id = true;
            break;
        case 0x04: // Usage Page (global)
            usage_page = val;
            break;
        case 0x08: // Usage (local)
            usage = val;
            break;
        case 0xA0: // Collection
            if (collection_level == 0 && topusage_count < TOPUSAGE_LIST_LEN) {
                uint32_t topusage = ((uint32_t)usage_page << 16) | usage;
                println("Found top level collection ", topusage, HEX);
                DBGPrintf("\ttopusage:%x\n", topusage);

                //topusage_list[topusage_count] = topusage;
                topusage_drivers[topusage_count] = find_driver(topusage);
                topusage_count++;
            }
            collection_level++;
            usage = 0;
            break;
        case 0xC0: // End Collection
            if (collection_level > 0) {
                collection_level--;
            }
        case 0x80: // Input
        case 0x90: // Output
        case 0xB0: // Feature
            usage = 0;
            break;
        }
    }
    while (topusage_count < TOPUSAGE_LIST_LEN) {
        //topusage_list[topusage_count] = 0;
        topusage_drivers[topusage_count] = NULL;
        topusage_count++;
    }
}

BTHIDInput * BluetoothConnection::find_driver(uint32_t topusage)
{
    println("find_driver");
    BTHIDInput *driver = BluetoothController::available_bthid_drivers_list;
    hidclaim_t claim_type;
    while (driver) {
        //println("  driver ", (uint32_t)driver, HEX);
        if ((claim_type = driver->bt_claim_collection(this, device_class_, topusage)) != CLAIM_NO) {
            //if (claim_type == CLAIM_INTERFACE) hid_driver_claimed_control_ = true;
            return driver;
        }
        driver = driver->next;
    }
    println("No Driver claimed topusage: ", topusage, HEX);
    return NULL;
}

void BluetoothConnection::parse(uint16_t type_and_report_id, const uint8_t *data, uint32_t len)
{
    const bool use_report_id = true;
    const uint8_t *p = descriptor_;
    const uint8_t *end = p + descsize_;
    BTHIDInput *driver = NULL;
    // USBHIDInput *driver = hidi_; // hack for now everything feeds back to us...
    uint32_t topusage = 0;
    uint8_t topusage_index = 0;
    uint8_t collection_level = 0;
    uint16_t usage[USAGE_LIST_LEN] = {0, 0};
    uint8_t usage_count = 0;
    uint8_t usage_min_max_count = 0;
    uint8_t usage_min_max_mask = 0;
    uint8_t report_id = 0;
    uint16_t report_size = 0;
    uint16_t report_count = 0;
    uint16_t usage_page = 0;
    uint32_t last_usage = 0;
    int32_t logical_min = 0;
    int32_t logical_max = 0;
    uint32_t bitindex = 0;

    while (p < end) {
        uint8_t tag = *p;
        if (tag == 0xFE) { // Long Item (unsupported)
            p += p[1] + 3;
            continue;
        }
        uint32_t val;
        switch (tag & 0x03) { // Short Item data
        case 0: val = 0;
            p++;
            break;
        case 1: val = p[1];
            p += 2;
            break;
        case 2: val = p[1] | (p[2] << 8);
            p += 3;
            break;
        case 3: val = p[1] | (p[2] << 8) | (p[3] << 16) | (p[4] << 24);
            p += 5;
            break;
        }
        if (p > end) break;
        bool reset_local = false;
        switch (tag & 0xFC) {
        case 0x04: // Usage Page (global)
            usage_page = val;
            break;
        case 0x14: // Logical Minimum (global)
            logical_min = signedval(val, tag);
            break;
        case 0x24: // Logical Maximum (global)
            logical_max = signedval(val, tag);
            break;
        case 0x74: // Report Size (global)
            report_size = val;
            break;
        case 0x94: // Report Count (global)
            report_count = val;
            break;
        case 0x84: // Report ID (global)
            report_id = val;
            break;
        case 0x08: // Usage (local)
            if (usage_count < USAGE_LIST_LEN) {
                // Usages: 0 is reserved 0x1-0x1f is sort of reserved for top level things like
                // 0x1 - Pointer - A collection... So lets try ignoring these
                if (val > 0x1f) {
                    usage[usage_count++] = val;
                }
            }
            break;
        case 0x18: // Usage Minimum (local)
            // Note: Found a report with multiple min/max
            if (usage_count != 255) {
                usage_count = 255;
                usage_min_max_count = 0;
                usage_min_max_mask = 0;
            }
            usage[usage_min_max_count * 2] = val;
            usage_min_max_mask |= 1;
            if (usage_min_max_mask == 3) {
                usage_min_max_count++;
                usage_min_max_mask = 0;
            }
            break;
        case 0x28: // Usage Maximum (local)
            if (usage_count != 255) {
                usage_count = 255;
                usage_min_max_count = 0;
                usage_min_max_mask = 0;
            }
            usage[usage_min_max_count * 2 + 1] = val;
            usage_min_max_mask |= 2;
            if (usage_min_max_mask == 3) {
                usage_min_max_count++;
                usage_min_max_mask = 0;
            }
            break;
        case 0xA0: // Collection
            if (collection_level == 0) {
                topusage = ((uint32_t)usage_page << 16) | usage[0];
                driver = NULL;
                if (topusage_index < TOPUSAGE_LIST_LEN) {
                    driver = topusage_drivers[topusage_index++];
                }
            }
            // discard collection info if not top level, hopefully that's ok?
            collection_level++;
            reset_local = true;
            break;
        case 0xC0: // End Collection
            if (collection_level > 0) {
                collection_level--;
                if (collection_level == 0 && driver != NULL) {
                    driver->bt_hid_input_end();
                    //driver = NULL;
                }
            }
            reset_local = true;
            break;
        case 0x80: // Input
            if (use_report_id && (report_id != (type_and_report_id & 0xFF))) {
                // completely ignore and do not advance bitindex
                // for descriptors of other report IDs
                reset_local = true;
                break;
            }
            if ((val & 1) || (driver == NULL)) {
                // skip past constant fields or when no driver is listening
                bitindex += report_count * report_size;
            } else {
                println("begin, usage=", topusage, HEX);
                println("       type= ", val, HEX);
                println("       min=  ", logical_min);
                println("       max=  ", logical_max);
                println("       reportcount=", report_count);
                println("       usage count=", usage_count);
                println("       usage min max count=", usage_min_max_count);

                driver->bt_hid_input_begin(topusage, val, logical_min, logical_max);
                println("Input, total bits=", report_count * report_size);
                if ((val & 2)) {
                    // ordinary variable format
                    uint32_t uindex = 0;
                    uint32_t uindex_max = 0xffff; // assume no MAX
                    bool uminmax = false;
                    uint8_t uminmax_index = 0;
                    if (usage_count > USAGE_LIST_LEN) {
                        // usage numbers by min/max, not from list
                        uindex = usage[0];
                        uindex_max = usage[1];
                        uminmax = true;
                    } else if ((report_count > 1) && (usage_count <= 1)) {
                        // Special cases:  Either only one or no usages specified and there are more than one
                        // report counts .
                        if (usage_count == 1) {
                            uindex = usage[0];
                        } else {
                            // BUGBUG:: Not sure good place to start?  maybe round up from last usage to next higher group up of 0x100?
                            uindex = (last_usage & 0xff00) + 0x100;
                        }
                        uminmax = true;
                    }
                    //USBHDBGDBGSerial.printf("TU:%x US:%x %x %d %d: C:%d, %d, MM:%d, %x %x\n", topusage, usage_page, val, logical_min, logical_max,
                    //      report_count, usage_count, uminmax, usage[0], usage[1]);
                    for (uint32_t i = 0; i < report_count; i++) {
                        uint32_t u;
                        if (uminmax) {
                            u = uindex;
                            if (uindex < uindex_max) uindex++;
                            else if (uminmax_index < usage_min_max_count) {
                                uminmax_index++;
                                uindex = usage[uminmax_index * 2];
                                uindex_max = usage[uminmax_index * 2 + 1];
                                //USBHDBGPDBGSerial.printf("$$ next min/max pair: %u %u %u\n", uminmax_index, uindex, uindex_max);
                            }
                        } else {
                            u = usage[uindex++];
                            if (uindex >= USAGE_LIST_LEN - 1) {
                                uindex = USAGE_LIST_LEN - 1;
                            }
                        }
                        last_usage = u; // remember the last one we used...
                        u |= (uint32_t)usage_page << 16;
                        print("  usage = ", u, HEX);

                        uint32_t n = bitfield(data, bitindex, report_size);
                        if (logical_min >= 0) {
                            println("  data = ", n);
                            driver->bt_hid_input_data(u, n);
                        } else {
                            int32_t sn = signext(n, report_size);
                            println("  sdata = ", sn);
                            driver->bt_hid_input_data(u, sn);
                        }
                        bitindex += report_size;
                    }
                } else {
                    // array format, each item is a usage number
                    // maybe act like the 2 case...
                    if (usage_min_max_count && (report_size == 1)) {
                        uint32_t uindex = usage[0];
                        uint32_t uindex_max = usage[1];
                        uint8_t uminmax_index = 0;
                        uint32_t u;

                        for (uint32_t i = 0; i < report_count; i++) {
                            u = uindex;
                            if (uindex < uindex_max) uindex++;
                            else if (uminmax_index < usage_min_max_count) {
                                uminmax_index++;
                                uindex = usage[uminmax_index * 2];
                                uindex_max = usage[uminmax_index * 2 + 1];
                                //USBHDBGPDBGSerial.printf("$$ next min/max pair: %u %u %u\n", uminmax_index, uindex, uindex_max);
                            }

                            u |= (uint32_t)usage_page << 16;
                            uint32_t n = bitfield(data, bitindex, report_size);
                            if (logical_min >= 0) {
                                println("  data = ", n);
                                driver->bt_hid_input_data(u, n);
                            } else {
                                int32_t sn = signext(n, report_size);
                                println("  sdata = ", sn);
                                driver->bt_hid_input_data(u, sn);
                            }

                            bitindex += report_size;
                        }

                    } else {
                        for (uint32_t i = 0; i < report_count; i++) {
                            uint32_t u = bitfield(data, bitindex, report_size);
                            int n = u;
                            if (n >= logical_min && n <= logical_max) {
                                u |= (uint32_t)usage_page << 16;
                                print("  usage = ", u, HEX);
                                println("  data = 1");
                                driver->bt_hid_input_data(u, 1);
                            } else {
                                print ("  usage =", u, HEX);
                                print(" out of range: ", logical_min, HEX);
                                println(" ", logical_max, HEX);
                            }
                            bitindex += report_size;
                        }
                    }
                }
            }
            reset_local = true;
            break;
        case 0x90: // Output
            // TODO.....
            reset_local = true;
            break;
        case 0xB0: // Feature
            // TODO.....
            reset_local = true;
            break;

        case 0x34: // Physical Minimum (global)
        case 0x44: // Physical Maximum (global)
        case 0x54: // Unit Exponent (global)
        case 0x64: // Unit (global)
            break; // Ignore these commonly used tags.  Hopefully not needed?

        case 0xA4: // Push (yikes! Hope nobody really uses this?!)
        case 0xB4: // Pop (yikes! Hope nobody really uses this?!)
        case 0x38: // Designator Index (local)
        case 0x48: // Designator Minimum (local)
        case 0x58: // Designator Maximum (local)
        case 0x78: // String Index (local)
        case 0x88: // String Minimum (local)
        case 0x98: // String Maximum (local)
        case 0xA8: // Delimiter (local)
        default:
            println("Ruh Roh, unsupported tag, not a good thing Scoob ", tag, HEX);
            break;
        }
        if (reset_local) {
            usage_count = 0;
            usage_min_max_count = 0;
            usage[0] = 0;
            usage[1] = 0;
        }
    }
}
