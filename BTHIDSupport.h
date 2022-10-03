#ifndef __BTHIDSupport_h__
#define __BTHIDSupport_h__
// note this will probably be merged into USBHost_t36.h
#include <Arduino.h>
#include <USBHost_t36.h>

class USBHIDInput;
class BluetoothController;

class BTHIDSupport {
public:
  BTHIDSupport( ) {}

  void setDriver(BluetoothController *driver) {btdriver_ = driver;}
  void setUSBHIDInput(USBHIDInput *hidi) {hidi_ = hidi;}

  bool startRetrieveHIDReportDescriptor();
  bool completeSDPRequest(bool success);

  bool process_bluetooth_HID_data(const uint8_t *data, uint32_t len);

  // will default to our descriptor buffer if none specified
  // which we will overwrite with actual one.
  // but may be needed if general query done. 
  void setBuffer(uint8_t *buffer, uint32_t len);


  static void dump_hexbytes(const void *ptr, uint32_t len, uint32_t indent);
  
  enum {DNIL=0, DU32, DS32, DU64, DS64, DPB, DLVL};
  typedef struct {
    uint8_t  element_type;
    uint8_t dtype;
    uint16_t element_size;    // size of element
    union {
      uint32_t uw;
      int32_t sw;
      uint64_t luw;
      int64_t lsw;
      uint8_t *pb;
    } data;
  } sdp_element_t;

  int  extract_next_SDP_Token(uint8_t *pbElement, int cb_left, sdp_element_t &sdpe);
  void print_sdpe_val(sdp_element_t &sdpe, bool verbose);
  void decode_SDP_buffer(bool verbose_output = false);
  void decode_SDP_Data(bool by_user_command);
  
protected:
private:
  void init();

  void dumpHIDReportDescriptor(uint8_t *pb, uint16_t cb);
  void printUsageInfo(uint8_t usage_page, uint16_t usage);
  void print_input_output_feature_bits(uint8_t val);
  
  bool decode_boot_report1(const uint8_t *data, uint16_t length);


  // Stuff from USBHost HID parse code
  void parse(uint16_t type_and_report_id, const uint8_t *data, uint32_t len);
	enum { TOPUSAGE_LIST_LEN = 6 };
	enum { USAGE_LIST_LEN = 24 };
	uint8_t descriptor[800];
	uint16_t descsize;
	bool use_report_id = true;


  USBHIDInput *hidi_;
  BluetoothController *btdriver_ = nullptr;
  uint8_t *sdp_buffer_ = nullptr;
  uint32_t sdp_buffer_len_ = 0;
  bool have_hid_descriptor_ = false;

  uint8_t collections_claimed = 0;
  volatile int hid_input_begin_level_ = 0;
  uint32_t index_;
  uint32_t fixed_usage_;
  volatile  bool connection_complete_ = false;
  
  bool decode_input_boot_data_ = false;
  uint8_t bluetooth_class_low_byte_ = 0;
  
  uint32_t usage_ = 0;
  // Track changing fields. 
   const static int MAX_CHANGE_TRACKED = 512;
  uint32_t usages_[MAX_CHANGE_TRACKED];
  int32_t values_[MAX_CHANGE_TRACKED];
  int count_usages_ = 0;
  int index_usages_ = 0;
  
  uint8_t packet_buffer_[256];
  uint16_t seq_number_ = 0;
  // experiment to see if we can receive data from Feature reports.
  enum {MAX_FEATURE_REPORTS=20};
  uint8_t feature_report_ids_[MAX_FEATURE_REPORTS];
  uint8_t cnt_feature_reports_ = 0;
};
#endif // __HIDDumper_h_
