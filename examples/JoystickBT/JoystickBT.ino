
// Simple test of USB Host Mouse/Keyboard
//
// This example is in the public domain

#include "USBHost_t36.h"
//#include "debug_tt.h"

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBHIDParser hid4(myusb);
USBHIDParser hid5(myusb);
JoystickController joystick1(myusb);
//BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
BluetoothController bluet(myusb);   // version assumes it already was paired
int user_axis[64];
uint16_t buttons_prev = 0;
uint16_t buttons;
RawHIDController rawhid1(myusb);
RawHIDController rawhid2(myusb, 0xffc90004);

USBDriver *drivers[] = {&hub1, &hub2, &joystick1, &bluet, &hid1, &hid2, &hid3, &hid4, &hid5};

#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1","Hub2", "JOY1D", "Bluet", "HID1" , "HID2", "HID3", "HID4", "HID5"};

bool driver_active[CNT_DEVICES] = {false, false, false, false};

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&joystick1, &rawhid1, &rawhid2};

#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_DEVICES] = {"Joystick1", "RawHid1", "RawHid2"};

bool hid_driver_active[CNT_DEVICES] = {false, false, false};
bool show_changed_only = false; 
bool show_raw_data = false;

uint8_t joystick_left_trigger_value = 0;
uint8_t joystick_right_trigger_value = 0;
uint64_t joystick_full_notify_mask = (uint64_t)-1;

int psAxis[64];
bool first_joystick_message = true;

void setup()
{
/*  Serial4.begin( 1843200 );
  debBegin_tt( &Serial4, LED_BUILTIN, 12);
  debTraceShow_tt( -1, "", "", "" );
  Serial4.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  Serial4.println("\n********\n T4 connected Serial1 *******\n");
  Serial4.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  Serial4.println("\n********\n T4 connected Serial4 *******\n");
*/
Serial1.begin(1843200);
  while (!Serial) ; // wait for Arduino Serial Monitor
  //debTraceShow_tt( -2, "", "", "" );
  //Serial4.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  //Serial1.begin( 1843200 );
  Serial.println("\n\nUSB Host Testing");
  Serial.println(sizeof(USBHub), DEC);
  myusb.begin();

  delay(2000);

  rawhid1.attachReceive(OnReceiveHidData);
  rawhid2.attachReceive(OnReceiveHidData);
}


void loop()
{
  myusb.Task();
 
  if (Serial.available()) {
    int ch = Serial.read(); // get the first char. 
    while (Serial.read() != -1) ; 
    if ((ch == 'b') || (ch == 'B')) {
      Serial.println("Only notify on Basic Axis changes");
      joystick1.axisChangeNotifyMask(0x3ff);
    } else if ((ch == 'f') || (ch == 'F')) {
      Serial.println("Only notify on Full Axis changes");
      joystick1.axisChangeNotifyMask(joystick_full_notify_mask);

    } else {
      if (show_changed_only) {
        show_changed_only = false;
        Serial.println("\n*** Show All fields mode ***");
      } else {
        show_changed_only = true;
        Serial.println("\n*** Show only changed fields mode ***");
      }
    }
 }

  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }

  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (*hiddrivers[i] != hid_driver_active[i]) {
      if (hid_driver_active[i]) {
        Serial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
        hid_driver_active[i] = false;
      } else {
        Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        hid_driver_active[i] = true;

        const uint8_t *psz = hiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = hiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = hiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }

  if (joystick1.available()) {
      if (first_joystick_message) {
        Serial.printf("*** First Joystick message %x:%x ***\n", 
            joystick1.idVendor(), joystick1.idProduct());
        first_joystick_message = false;

        const uint8_t *psz = joystick1.manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = joystick1.product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz =joystick1.serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);        
      }
      
      for (uint8_t i = 0; i<64; i++) {
          psAxis[i] = joystick1.getAxis(i);
      }
      
      switch (joystick1.joystickType) {
        case JoystickController::UNKNOWN:
        case JoystickController::PS4:
          displayPS4Data();
          break;
        case JoystickController::PS3:
          displayPS3Data();
          break;
        case JoystickController::XBOXONE:
        case JoystickController::XBOX360:;
          displayRawData();
          break;
      }
      //for (uint8_t i = 0; i < 24; i++) {
      //  Serial.printf(" %d:%d", i, psAxis[i]);
      //}
      //Serial.println();
        

     joystick1.joystickDataClear();
  }

  // See if we have some RAW data
  if (rawhid1) {
    int ch;
    uint8_t buffer[64];
    uint8_t count_chars = 0; 
    memset(buffer, 0, sizeof(buffer));
    if (Serial.available()) {
      while (((ch = Serial.read()) != -1) && (count_chars < sizeof(buffer))) {
        buffer[count_chars++] = ch;
      }
      rawhid1.sendPacket(buffer);
    }
  }
}

void displayPS4Data()
{
  Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d \r\n", psAxis[1], psAxis[2], psAxis[3], psAxis[4]);
  Serial.printf("L-Trig: %d, R-Trig: %d, Trig-Button: %d \r\n", psAxis[8], psAxis[9], psAxis[6]);
  Serial.printf("Buttons: %d, PS: %d\r\n", psAxis[5], psAxis[7]);
  Serial.printf("Arrows: %d\r\n", psAxis[10]);
  Serial.printf("Battery Status: %d\n", (psAxis[30] & (1 << 4) - 1));
  Serial.println();

  uint8_t ltv;
  uint8_t rtv;

    ltv = psAxis[8];
    rtv = psAxis[9];

    if ((ltv != joystick_left_trigger_value) || (rtv != joystick_right_trigger_value)) {
      joystick_left_trigger_value = ltv;
      joystick_right_trigger_value = rtv;
      Serial.printf("Rumbling: %d, %d\r\n", ltv, rtv);
      joystick1.setRumble(ltv, rtv);
    }
    
  /* Arrow Buttons (psAxis[0]):
   *    0x08 is released, 
   *    0=N, 1=NE, 2=E, 3=SE, 4=S, 
   *    5=SW, 6=W, 7=NW)
   */

  if (psAxis[5] != buttons_prev) {
      uint8_t lr = (psAxis[5] & 1) ? 0xff : 0;   //Srq
      uint8_t lg = (psAxis[5] & 4) ? 0xff : 0;   //Cir
      uint8_t lb = (psAxis[5] & 8) ? 0xff : 0;   //Tri
                                                 //Cross = 2
      Serial.print(psAxis[5]); Serial.print(", ");
      Serial.print(lr); Serial.print(", "); 
      Serial.print(lg); Serial.print(", "); 
      Serial.println(lb); 

      joystick1.setLEDs(lr, lg, lb);
      buttons_prev =psAxis[5];
  }
}

void displayPS3Data()

{
  Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d \r\n", psAxis[1], psAxis[2], psAxis[3], psAxis[4]);
  Serial.printf("L-Trig: %d, R-Trig: %d, Trig-Button: %d \r\n", psAxis[8], psAxis[9], psAxis[6]);
  Serial.printf("Buttons: %d, PS: %d\r\n", psAxis[5], psAxis[7]);
  /*Arrows:
   *  N = 16
   * NE = 48
   *  E = 32
   * SE = 96
   *  S = 64
   * SW = 192
   *  W = 128
   * NW = 144
   */
  Serial.printf("Arrows: %d\r\n", psAxis[10]);
  Serial.println();

  uint8_t ltv;
  uint8_t rtv;

    ltv = psAxis[8];
    rtv = psAxis[9];

    if ((ltv != joystick_left_trigger_value) || (rtv != joystick_right_trigger_value)) {
      joystick_left_trigger_value = ltv;
      joystick_right_trigger_value = rtv;
      Serial.printf("Rumbling: %d, %d\r\n", ltv, rtv);
      joystick1.setRumble(ltv, rtv);
    }
    
  if (psAxis[5] != buttons_prev) {
      uint8_t lr = (psAxis[5] & 128) ? 0xff : 0;   //Srq
      uint8_t lg = (psAxis[5] & 32) ? 0xff : 0;   //Cir
      uint8_t lb = (psAxis[5] & 16) ? 0xff : 0;   //Tri
                                                 //Cross = 64
      Serial.print(psAxis[5]); Serial.print(", ");
      Serial.print(lr); Serial.print(", "); 
      Serial.print(lg); Serial.print(", "); 
      Serial.println(lb); 

      joystick1.setLEDs(lr, lg, lb);
      buttons_prev =psAxis[5];
  }
}
 

void displayRawData() {
  uint64_t axis_mask = joystick1.axisMask();
  uint64_t changed_mask = joystick1.axisChangedMask();
  if (!changed_mask) return;

  Serial.printf("%lx %lx:", axis_mask, changed_mask);
  for (uint16_t index=0; axis_mask; index++) {
    Serial.printf("%02x ", psAxis[index]);
    axis_mask >>= 1;
  }
  Serial.println();
  //for (uint8_t i = 0; i < 24; i++) {
  //  Serial.printf(" %d:%d", i, psAxis[i]);
  //}
  //Serial.println();
 
}


bool OnReceiveHidData(uint32_t usage, const uint8_t *data, uint32_t len) {
  // Called for maybe both HIDS for rawhid basic test.  One is for the Teensy
  // to output to Serial. while still having Raw Hid... 
  if (usage == 0xffc90004) {
    // Lets trim off trailing null characters.
    while ((len > 0) && (data[len-1] == 0)) {
      len--;
    }
    if (len) {
      Serial.print("RawHid Serial: ");
      Serial.write(data, len);
    }
  } else {
    Serial.print("RawHID data: ");
    Serial.println(usage, HEX);
    while (len) {
      uint8_t cb = (len > 16)? 16 : len;
      const uint8_t *p = data;
      uint8_t i;
      for (i = 0; i < cb; i++) {
        Serial.printf("%02x ", *p++);
      }
      Serial.print(": ");
      for (i = 0; i < cb; i++) {
        Serial.write(((*data >= ' ')&&(*data <= '~'))? *data : '.');
        data++;
      }
      len -= cb;
      Serial.println();
    }
  }

  return true;
}
