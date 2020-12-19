// HIDDeviceInfo - Simple HID device example
//
// This Simple test sketch is setup to print out HID information about a device
// The other two tabs are a simple C++ subclass of the USBHIDInput class that is part
// of the USBHost_t36 library.
//
// This subclass simply tries to connect to each different HID object and
// the only thing it does is to try to print out all of the data it receives
// in a reasonable way.
//
// The idea is that with the output from this sketch we can hopefully add support
// for some additional devices that are not currently supported or allows you to
// develop your own.
//
// You can use Serial Input to control how much data is displayed per each HID packet
// received by the sketch.
//
// By Default it displays both the RAW (Hex dump) of the data received, as well
// as the data as the HID interpreter walks through the data into the individual
// fields, which we then print out.
//
// There are options to turn off some of this output, also an option that you can
// toggle on or off (C) to only try to show the changed fields.
//
// This example is in the public domain

#include <USBHost_t36.h>
#include <MTPDevice.h>

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
MTPDevice mtpd(myusb);
USBSerial_BigBuffer userial(myusb, 1);  // USB Serial  big or little... 
SerEMUController seremu(myusb);


USBDriver *drivers[] = {&hub1, &mtpd, &hid1, &hid2, &userial};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "MTPD", "HID1", "HID2", "USerial"};
bool driver_active[CNT_DEVICES] = {false, false};

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&seremu};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_DEVICES] = {"seremu"};
bool hid_driver_active[CNT_DEVICES] = {false, false};

Stream *debug_stream = nullptr;

elapsedMillis emBlink = 0;

void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(2000000);
  while (!Serial) ; // wait for Arduino Serial Monitor
  Serial.println("\n\nUSB MTP Device Test Program");

  myusb.begin();

}


char buffer[512];

bool mtpd_setup_complete = false;
uint8_t count_storages = 0;
uint8_t enum_storage_index = 0;
const MTPDevice::storage_list_t * enum_storage_list_item = nullptr;


void loop()
{
  if (emBlink > 500) {
    emBlink = 0;
    digitalToggleFast(13);
  }
  myusb.Task();
  CheckHostDevicesChanged();
  processDebugStreams();

  if (mtpd && mtpd.setupComplete()) {
    // We have an active mtpd... 
    if (!mtpd_setup_complete) {
      // first time through... 
      mtpd_setup_complete = true;
      count_storages = mtpd.countStorages();
      ShowStorageList();

      // Will user drive in a bit... 
      mtpd.startEnumStorageIndex(0);
      enum_storage_index = 0;
    }

    if ((enum_storage_index < count_storages) && mtpd.enumComplete()) {
      // Again real klude to manually walk the node list. 
      const MTPDevice::storage_info_t *storage_info = mtpd.getStorageInfo(enum_storage_index);
      Serial.printf("\n========== %s ===========\n", storage_info->storage.name);
      const MTPDevice::storage_list_t *node = storage_info->storage.child;
      while (node) {
        Serial.printf("%s(%x) FMT:%x Size:%llu\n", node->name, node->id, node->format, node->size);
        node = node->next;
      }

      if(++enum_storage_index < count_storages) mtpd.startEnumStorageIndex(enum_storage_index);
    }

    if (enum_storage_list_item && mtpd.enumComplete()) {
      // Again real klude to manually walk the node list. 
      Serial.printf("\n========== Enum completed (%u:%s ===========\n", enum_storage_list_item->id, enum_storage_list_item->name);
      enum_storage_list_item = enum_storage_list_item->child;  
      while (enum_storage_list_item) {
        Serial.printf("%s(%x) FMT:%x Size:%llu\n", enum_storage_list_item->name, enum_storage_list_item->id, 
              enum_storage_list_item->format, enum_storage_list_item->size);
        enum_storage_list_item = enum_storage_list_item->next;
      }
    }
  } 
  
  if (Serial.available()) {
    int cmd = Serial.read(); // get the first char.
    int ch = Serial.read(); // get next character
    uint32_t id = 0;
    while (ch == ' ') ch = Serial.read();
    for(;;) {
      if ((ch >= '0') && (ch <= '9')) id = id * 16 + ch - '0';
      else if ((ch >= 'a') && (ch <= 'f')) id = id * 16 + 10 + ch - 'a';
      else if ((ch >= 'A') && (ch <= 'F')) id = id * 16 + 10 + ch - 'A';
      else break;
      ch = Serial.read();
    }

    switch (cmd) {
      case 's':
        ShowStorageList();
        break;
      case 'd':
        mtpd.printNodeList();
        break;  
      case 'e':
        // lets see if we can find that storage iD in our list:
        {
          const MTPDevice::storage_list_t * storage_list_item = mtpd.findStorageItemByID(id);
          if (storage_list_item) {
            if (storage_list_item->format == 0x3001) {
              Serial.printf("/n/ ================= Start ENUM(%u:%s) =================\n", storage_list_item->id, storage_list_item->name);
              mtpd.startEnumStorageNode(storage_list_item);
              enum_storage_list_item = storage_list_item; // remember this one
            } else {
              Serial.printf("Item %s(%x) is not what I think is a directory %x\n", 
                    storage_list_item->name, storage_list_item->id, storage_list_item->format);
            }
            
          } else {
            Serial.printf("Item: %x Not found in list\n", id);
          }
        }
        break;
      default: 
        Serial.println("\n---------- Commands ----------");
        Serial.println("  s - Show storage list");
        Serial.println("  e - enum <ID>");
        Serial.println("  d - dump storage list");
        break;
    }

    while (Serial.read() != -1) ;
  }

}

void ShowStorageList() {
    Serial.printf("\n*** MPT connected ***\nConnected to:%s\n", mtpd.DeviceFriendlyName());
    Serial.printf("Count of Storages: %u\n",count_storages);
    for (uint8_t i = 0; i < count_storages; i++) {
      const MTPDevice::storage_info_t *storage_info = mtpd.getStorageInfo(i);
      if (storage_info) {
        Serial.printf("%d(%x): %s(%s) type:%x %x max:%llu, free: %llu access:%u\n", i, storage_info->storage.id,
          storage_info->storage.name, storage_info->volume_id, 
          storage_info->storage_type, storage_info->filesystem_type,
          storage_info->max_capacity, storage_info->free_space, storage_info->access);
      }
    }
}

void CheckHostDevicesChanged()
{
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device % s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device % s % x: % x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: % s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: % s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: % s\n", psz);
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
}

void processDebugStreams() 
{
  //==============================================
  //  Debug stream stuff
  if (seremu) debug_stream = &seremu;
  else if (userial) debug_stream = &userial;
  else debug_stream = nullptr;

#if  defined(USB_TRIPLE_SERIAL) || defined(USB_DUAL_SERIAL)
  uint16_t avail;
  uint16_t avail_for_write;
  if (debug_stream) {
    if ((avail = debug_stream->available())) {
      avail_for_write = SerialUSB1.availableForWrite();
      if (avail > avail_for_write) avail = avail_for_write;
      if (avail > sizeof(buffer)) avail = sizeof(buffer);
      debug_stream->readBytes(buffer, avail);
      SerialUSB1.write(buffer, avail);
    }

    if ((avail = SerialUSB1.available())) {
      avail_for_write = debug_stream->availableForWrite();
      if (avail > avail_for_write) avail = avail_for_write;
      if (avail > sizeof(buffer)) avail = sizeof(buffer);
      SerialUSB1.readBytes(buffer, avail);
      debug_stream->write(buffer, avail);
      Serial.printf("USB1->");
      Serial.write(buffer, avail);
    }
  }
#endif  
}