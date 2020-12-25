/* USB EHCI Host for Teensy 3.6 and Teensy 4.x
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
 */

#ifndef USB_HOST_MTPDEVICE_H_
#define USB_HOST_MTPDEVICE_H_

#include "USBHost_t36.h"
#include "utility/MTPDefines.h"

//--------------------------------------------------------------------------
class MTPDevice : public USBDriver {
public:
	MTPDevice(USBHost &host) { init(); }
	MTPDevice(USBHost *host) { init(); }
	enum {MAX_STORAGES=10, MAX_OBJECT_FORMATS=5, MAX_OBJECT_PROPERTY_IDS=15, MAX_PENDING_EVENTS=10};

	// need their own versions as both USBDriver and USBHIDInput provide
	uint16_t idVendor();
	uint16_t idProduct();
	const uint8_t *manufacturer();
	const uint8_t *product();
	const uint8_t *serialNumber();

	// MTP return data... 
	bool setupComplete() {return setup_complete_;}
	uint8_t countStorages() {return cnt_storages_;}
	const uint8_t *DeviceFriendlyName()  {return (const uint8_t *)device_friendly_name_;}

	// Not sure if public for long, but... this is an experiment...  
	typedef struct storage_list_ {
		uint32_t 				id;			// our ID
		uint64_t				size;		// size of the file
		uint32_t				format;		// format of object...
		struct storage_list_  	*parent; 	// parent ID
		struct storage_list_  	*next;	// first child ID;
		struct storage_list_  	*child;	// first child ID;
		uint8_t					*name;	// the name the node
		uint32_t				storage_id; // Storage ID.  
	} storage_list_t;	

	typedef struct {
		storage_list_t 			storage;
		uint16_t				storage_type;
		uint16_t 				filesystem_type;
		uint16_t				access;
		uint64_t				max_capacity;
		uint64_t				free_space;
		uint32_t				free_space_objects;
		uint8_t					*volume_id;
	} storage_info_t;

	typedef struct {
		uint32_t				event;		// which type of event;
		uint32_t				id;			// id of object;
		uint32_t				prop_code;	// if event has other data. 
		storage_list_t			*item_node;	// pointer to item may be null at times.
		bool 					delete_node; // need to delete after notify...
	} event_data_t;		

	typedef void (*eventCompleteCB_t) (const event_data_t *pevent);

	const storage_info_t *getStorageInfo(uint8_t index) {
		if (index >= cnt_storages_) return nullptr;
		return (const storage_info_t *)&storage_info_[index]; 
	}
	bool startEnumStorageIndex(uint8_t index);
	static const storage_list_t *findStorageItem(storage_list_t *item, uint32_t id);
    const storage_list_t *findStorageItemByID(uint32_t id);
	
	void startEnumStorageNode(const storage_list_t *node);
	bool enumComplete() {return enum_node_ == nullptr;}
	void printNodeListItem(storage_list_t *item, uint8_t level=0);
	void printNodeList() ;
	
	uint32_t deleteObject(uint32_t id, uint32_t format=0, uint32_t timeoutMS=1000);
	void setEventCompleteCB(eventCompleteCB_t pcb) {peventCompeteCB_ = pcb;}

protected:
	virtual bool claim(Device_t *device, int type, const uint8_t *descriptors, uint32_t len);
	virtual void control(const Transfer_t *transfer);
	virtual void disconnect();
	static void callback(const Transfer_t *transfer);
	void init();

	static void rx_callback(const Transfer_t *transfer);
	static void tx_callback(const Transfer_t *transfer);
	static void event_callback(const Transfer_t *transfer);
	void rx_data(const Transfer_t *transfer);
	void tx_data(const Transfer_t *transfer);
	void event_data(const Transfer_t *transfer);

	struct MTPHeader {
    uint32_t len;  // 0
    uint16_t type; // 4
    uint16_t op;   // 6
    uint32_t transaction_id; // 8
  } __attribute__((packed));

  struct MTPContainer {
    uint32_t len;  // 0
    uint16_t type; // 4
    uint16_t op;   // 6
    uint32_t transaction_id; // 8
    uint32_t params[5];    // 12
  } __attribute__((packed, __may_alias__)) ;

  void sendMsg(uint16_t operation);
  void sendMsg(uint16_t operation, uint32_t p1);
  void sendMsg(uint16_t operation, uint32_t p1, uint32_t p2);
  void sendMsg(uint16_t operation, uint32_t p1, uint32_t p2, uint32_t p3);
  void printContainer(MTPContainer *c, const char *msg = nullptr);
  void processMTPCommand(MTPContainer *c);
  void processMTPData(MTPContainer *c);
  void processMTPResponse(MTPContainer *c);
  static void freeStorageListTree(storage_list_t *item);  // recurses and no need for other data


  void processDescriptorData(MTPContainer *c);
  void processDevicePropDesc(MTPContainer *c);
  void processGetStorageIDs(MTPContainer *c);
  void processGetStoreInfo(MTPContainer *c);
  void processObjectPropsSupported(MTPContainer *c);
  void processObjectPropDesc(MTPContainer *c);
  void processGetObjectHandles(MTPContainer *c);
  void processGetObjectPropValue(MTPContainer *c);
  
  void start_process_next_event();
  void complete_processing_event(bool start_next_event);
  bool process_object_added_event(uint32_t event_index);
  bool process_object_removed_event(uint32_t event_index);


  uint8_t read8(const uint8_t **pdata);
  uint16_t read16(const uint8_t **pdata); 
  uint32_t read32(const uint8_t **pdata);
  uint64_t read64(const uint8_t **pdata);
  void readStr(uint8_t *str, const uint8_t **pdata);
  uint8_t *readAndAllocStr(const uint8_t **pdata);


  uint32_t transaction_id_ = 0;
  uint32_t session_id_ = 0;
  uint16_t last_mtp_op_ = 0;

private:
	Pipe_t mypipes[4] __attribute__ ((aligned(32)));
	Transfer_t mytransfers[4] __attribute__ ((aligned(32)));
	strbuf_t mystring_bufs[7];

	Pipe_t 			*rxpipe_;
	Pipe_t 			*txpipe_;
	Pipe_t 			*eventpipe_;

	uint16_t rx_size_ = 0;
	uint16_t tx_size_ = 0;
	uint16_t event_size_ = 0;

	uint8_t txbuffer[512];
	uint8_t rx1[512];
	uint8_t rx2[512];
	uint8_t rxevent[64];	// make sure 

	// storage information
	uint8_t		cnt_storages_ = 0;
	uint8_t		get_store_info_index_ = 0xff;
	uint8_t    *device_friendly_name_ = nullptr;
	storage_info_t storage_info_[MAX_STORAGES];

	uint16_t 		cnt_object_formats_ = 0;
	uint16_t		object_formats_[MAX_OBJECT_FORMATS];
	uint8_t			cnt_object_property_ids_ = 0;
	uint16_t		object_property_ids_[MAX_OBJECT_PROPERTY_IDS];
	bool 			setup_complete_ = false;
	volatile storage_list_t	*enum_node_ = nullptr;

	storage_list_t	*prop_node_ = nullptr;
	volatile uint8_t			prop_index_ = 0;
	volatile uint32_t	last_response_ = 0;
	event_data_t		pending_events_[MAX_PENDING_EVENTS];
	volatile uint32_t	pending_events_head_ = 0;
	volatile uint32_t	pending_events_tail_ = 0;
	volatile bool 		pending_events_active_ = false;

	eventCompleteCB_t	peventCompeteCB_ = nullptr;

};

#endif //USB_HOST_MTPDEVICE_H_