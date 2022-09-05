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
 */

#include <Arduino.h>
#include "USBHost_t36.h"  // Read this header first for key info
#include "keylayouts.h"   // from Teensyduino core library

typedef struct {
	KEYCODE_TYPE code;
	uint8_t		 ascii;
} keycode_extra_t;

typedef struct {
	KEYCODE_TYPE code;
	KEYCODE_TYPE codeNumlockOff;
	uint8_t charNumlockOn;		// We will assume when num lock is on we have all characters...
} keycode_numlock_t;

typedef struct {
	uint16_t	idVendor;		// vendor id of keyboard
	uint16_t	idProduct;		// product id - 0 implies all of the ones from vendor; 
} vid_pid_t;	// list of products to force into boot protocol

#ifdef M
#undef M
#endif
#define M(n) ((n) & KEYCODE_MASK)

static const keycode_extra_t keycode_extras[] = {
	{M(KEY_ENTER), '\n'},
	{M(KEY_ESC), 0x1b},
	{M(KEY_TAB), 0x9 },
	{M(KEY_UP), KEYD_UP },
	{M(KEY_DOWN), KEYD_DOWN },
	{M(KEY_LEFT), KEYD_LEFT },
	{M(KEY_RIGHT), KEYD_RIGHT },
	{M(KEY_INSERT), KEYD_INSERT },
	{M(KEY_DELETE), KEYD_DELETE }, 
	{M(KEY_PAGE_UP), KEYD_PAGE_UP },
	{M(KEY_PAGE_DOWN), KEYD_PAGE_DOWN }, 
	{M(KEY_HOME), KEYD_HOME },
	{M(KEY_END), KEYD_END },   
	{M(KEY_F1), KEYD_F1 },
	{M(KEY_F2), KEYD_F2 },     
	{M(KEY_F3), KEYD_F3 },     
	{M(KEY_F4), KEYD_F4 },     
	{M(KEY_F5), KEYD_F5 },     
	{M(KEY_F6), KEYD_F6 },     
	{M(KEY_F7), KEYD_F7 },     
	{M(KEY_F8), KEYD_F8 },     
	{M(KEY_F9), KEYD_F9  },     
	{M(KEY_F10), KEYD_F10 },    
	{M(KEY_F11), KEYD_F11 },    
	{M(KEY_F12), KEYD_F12 }    
};

// Some of these mapped to key + shift.
static const keycode_numlock_t keycode_numlock[] = {
	{M(KEYPAD_SLASH), 	'/', '/'},
	{M(KEYPAD_ASTERIX), '*', '*'},
	{M(KEYPAD_MINUS),	'-', '-'},
	{M(KEYPAD_PLUS), 	'+', '+'},
	{M(KEYPAD_ENTER), 	'\n', '\n'},
	{M(KEYPAD_1), 		0x80 | M(KEY_END), '1'},
	{M(KEYPAD_2), 		0x80 | M(KEY_DOWN), '2'},
	{M(KEYPAD_3), 		0x80 | M(KEY_PAGE_DOWN), '3'},
	{M(KEYPAD_4), 		0x80 | M(KEY_LEFT), '4'},
	{M(KEYPAD_5), 		0x00, '5'},
	{M(KEYPAD_6), 		0x80 | M(KEY_RIGHT), '6'},
	{M(KEYPAD_7), 		0x80 | M(KEY_HOME),  '7'},
	{M(KEYPAD_8), 		0x80 | M(KEY_UP),  '8'},
	{M(KEYPAD_9), 		0x80 | M(KEY_PAGE_UP), '9'},
	{M(KEYPAD_0), 		0x80 | M(KEY_INSERT), '0'},
	{M(KEYPAD_PERIOD), 	0x80 | M(KEY_DELETE), '.'}
};

//============================================================
// Items in the list we will try to force into Boot mode.
//============================================================
static const vid_pid_t keyboard_forceBootMode[] = {
	{0x04D9, 0}
};

//============================================================
// Items in the list allow HID Parser to claim
//============================================================
bool KeyboardController::s_forceHIDMode = false;

static const vid_pid_t keyboard_use_hid_mode[] = {
	{0x04D9, 0},

	{0x046D, 0xC547}
};


#define print   print_
#define println println_




void KeyboardController::init()
{
	USBHIDParser::driver_ready_for_hid_collection(this);
	BluetoothController::driver_ready_for_bluetooth(this);
}

void KeyboardController::forceBootProtocol()
{
#if 0
	if (device && !control_queued) {
		mk_setup(setup, 0x21, 11, 0, 0, 0); // 11=SET_PROTOCOL  BOOT
		control_queued = true;
		queue_Control_Transfer(device, &setup, NULL, this);		
	} else {
		force_boot_protocol = true;	// let system know we want to force this.
	}
#endif
}


// Arduino defined this static weak symbol callback, and their
// examples use it as the only way to detect new key presses,
// so unfortunate as static weak callbacks are, it probably
// needs to be supported for compatibility
extern "C" {
void __keyboardControllerEmptyCallback() { }
}
void keyPressed()  __attribute__ ((weak, alias("__keyboardControllerEmptyCallback")));
void keyReleased() __attribute__ ((weak, alias("__keyboardControllerEmptyCallback")));

static bool contains(uint8_t b, const uint8_t *data)
{
	if (data[2] == b || data[3] == b || data[4] == b) return true;
	if (data[5] == b || data[6] == b || data[7] == b) return true;
	return false;
}

void KeyboardController::numLock(bool f) {
	if (leds_.numLock != f) {
		leds_.numLock = f;
		updateLEDS();
	}
}

void KeyboardController::capsLock(bool f) {
	if (leds_.capsLock != f) {
		leds_.capsLock = f;
		updateLEDS();
	}
}

void KeyboardController::scrollLock(bool f) {
	if (leds_.scrollLock != f) {
		leds_.scrollLock = f;
		updateLEDS();
	}
}

void KeyboardController::key_press(uint32_t mod, uint32_t key)
{
	// TODO: queue events, perform callback from Task
	println("  press, key=", key);
	//USBHDBGSerial.printf("key_press: %x %x\n", mod, key);
	modifiers_ = mod;
	keyOEM_ = key;
	keyCode = convert_to_unicode(mod, key);
	println("  unicode = ", keyCode);
	if (keyPressedFunction) {
		keyPressedFunction(keyCode);
	} else {
		keyPressed();
	}
}

void KeyboardController::key_release(uint32_t mod, uint32_t key)
{
	// TODO: queue events, perform callback from Task
	println("  release, key=", key);
	modifiers_ = mod;
	keyOEM_ = key;

	// Look for modifier keys
	if (key == M(KEY_NUM_LOCK)) {
		numLock(!leds_.numLock);
		// Lets toggle Numlock
	} else if (key == M(KEY_CAPS_LOCK)) {
		capsLock(!leds_.capsLock);

	} else if (key == M(KEY_SCROLL_LOCK)) {
		scrollLock(!leds_.scrollLock);
	} else {
		keyCode = convert_to_unicode(mod, key);
		if (keyReleasedFunction) {
			keyReleasedFunction(keyCode);
		} else {
			keyReleased();
		}
	}
}

uint16_t KeyboardController::convert_to_unicode(uint32_t mod, uint32_t key)
{
	// WIP: special keys
	// TODO: dead key sequences


	if (key & SHIFT_MASK) {
		// Many of these keys will look like they are other keys with shift mask...
		// Check for any of our mapped extra keys
		for (uint8_t i = 0; i < (sizeof(keycode_numlock)/sizeof(keycode_numlock[0])); i++) {
			if (keycode_numlock[i].code == key) {
				// See if the user is using numlock or not...
				if (leds_.numLock) {
					return keycode_numlock[i].charNumlockOn;
				} else {
					key = keycode_numlock[i].codeNumlockOff;
					if (!(key & 0x80)) return key;	// we have hard coded value
					key &= 0x7f;	// mask off the extra and break out to process as other characters...
					break;
				}
			}
		}
	}

	// Check for any of our mapped extra keys - Done early as some of these keys are 
	// above and some below the SHIFT_MASK value
	for (uint8_t i = 0; i < (sizeof(keycode_extras)/sizeof(keycode_extras[0])); i++) {
		if (keycode_extras[i].code == key) {
			return keycode_extras[i].ascii;
		}
	}

	// If we made it here without doing something then return 0;
	if (key & SHIFT_MASK) return 0;

	if ((mod & 0x02) || (mod & 0x20)) key |= SHIFT_MASK;
	if (leds_.capsLock) key ^= SHIFT_MASK;		// Caps lock will switch the Shift;
	for (int i=0; i < 96; i++) {
		if (keycodes_ascii[i] == key) {
			if ((mod & 1) || (mod & 0x10)) return (i+32) & 0x1f;	// Control key is down
			return i + 32;
		}
	}


#ifdef ISO_8859_1_A0
	for (int i=0; i < 96; i++) {
		if (keycodes_iso_8859_1[i] == key) return i + 160;
	}
#endif
	return 0;
}

void KeyboardController::LEDS(uint8_t leds) {
	println("Keyboard setLEDS ", leds, HEX);
	leds_.byte = leds;
	updateLEDS();
}

void KeyboardController::updateLEDS() {
	// Now lets tell keyboard new state.
	if (driver_[0] != nullptr) {
		// Only do it this way if we are a standard USB device
	    driver_[0]->sendControlPacket(0x21, 9, 0x200, 0, sizeof(leds_.byte), (void*) &leds_.byte); 
	} else {
		// Bluetooth, need to setup back channel to Bluetooth controller. 
	}
}

void KeyboardController::process_boot_keyboard_format(const uint8_t *report, bool process_mod_keys)
{
	//Serial.printf("** Process boot keyboard format **\n");
	for (int i=2; i < 8; i++) {
		uint32_t key = prev_report_[i];
		if (key >= 4 && !contains(key, report)) {
			key_release(prev_report_[0], key);
			if (rawKeyReleasedFunction) {
				rawKeyReleasedFunction(key);
			}
		}
	}
	if (process_mod_keys && rawKeyReleasedFunction) {
		// each modifier key is represented by a bit in the first byte
		for (int i = 0; i < 8; ++i)
		{
			uint8_t keybit = 1 << i;
			if ((prev_report_[0] & keybit) && !(report[0] & keybit)) {
				rawKeyReleasedFunction(103 + i);
			}
		}
	}
	for (int i=2; i < 8; i++) {
		uint32_t key = report[i];
		if (key >= 4 && !contains(key, prev_report_)) {
			key_press(report[0], key);
			if (rawKeyPressedFunction) {
				rawKeyPressedFunction(key);
			}
		}
	}
	if (process_mod_keys && rawKeyPressedFunction) {
		for (int i = 0; i < 8; ++i)
		{
			uint8_t keybit = 1 << i;
			if (!(prev_report_[0] & keybit) && (report[0] & keybit)) {
				rawKeyPressedFunction(103 + i);
			}
		}
	}
	memcpy(prev_report_, report, 8);
}

//=============================================================================
// Keyboard Extras - Combined from other object
//=============================================================================

#define TOPUSAGE_SYS_CONTROL 	0x10080
#define TOPUSAGE_CONSUMER_CONTROL	0x0c0001

#define TOPUSAGE_KEYBOARD 0X10006

hidclaim_t KeyboardController::claim_collection(USBHIDParser *driver, Device_t *dev, uint32_t topusage)
{
	// Lets try to claim a few specific Keyboard related collection/reports
	USBHDBGSerial.printf("KeyboardController::claim_collection(%p) Driver:%p(%u %u) Dev:%p Top:%x\n", this, driver, 
		driver->interfaceSubClass(), driver->interfaceProtocol(), dev, topusage);
	if ((topusage != TOPUSAGE_KEYBOARD) 
			 && (topusage != TOPUSAGE_SYS_CONTROL) 
			 && (topusage != TOPUSAGE_CONSUMER_CONTROL) )
		return CLAIM_NO;

	// only claim from one physical device
	// Lets only claim if this is the same device as claimed Keyboard... 
	//USBHDBGSerial.printf("\tdev=%p mydevice=%p\n", dev, mydevice);
	if (mydevice != NULL && dev != mydevice) return CLAIM_NO;

	// keep all three drivers:
	if (topusage == TOPUSAGE_KEYBOARD) driver_[0] = driver;
	else if (topusage == TOPUSAGE_SYS_CONTROL) driver_[1] = driver;
	else driver_[2] = driver;
	mydevice = dev;
	collections_claimed_++;
	USBHDBGSerial.printf("KeyboardController claim collection\n");
	return CLAIM_REPORT;
}

void KeyboardController::disconnect_collection(Device_t *dev)
{
	if (--collections_claimed_ == 0) {
		mydevice = NULL;
		driver_[0] = NULL;
	}
}

bool KeyboardController::hid_process_in_data(const Transfer_t *transfer)
{
	const uint8_t *buffer = (const uint8_t *)transfer->buffer;
	/*
	uint16_t len = transfer->length;
	const uint8_t *p = buffer;
	Serial.printf("HPID(%p, %u):", transfer->driver, len);
	  if (len > 32) len = 32;
	while (len--) Serial.printf(" %02X", *p++);
	Serial.printf("\n");
	*/
	// Probably need to do some more checking of the data, but
	// first pass if length == 8 assume boot format:
	if (transfer->length == 8) {
		process_boot_keyboard_format(buffer, true);
		return true;
	}

	return false;
}


void KeyboardController::hid_input_begin(uint32_t topusage, uint32_t type, int lgmin, int lgmax)
{
	//USBHDBGSerial.printf("KPC:hid_input_begin TUSE: %x TYPE: %x Range:%x %x\n", topusage, type, lgmin, lgmax);
	topusage_ = topusage;	// remember which report we are processing. 
	topusage_type_ = type;
	topusage_index_ = 2;  // hack we ignore first two bytes	
	hid_input_begin_ = true;
	hid_input_data_ = false;
}

void KeyboardController::hid_input_data(uint32_t usage, int32_t value)
{
	// Hack ignore 0xff00 high words as these are user values... 
	if ((usage & 0xffff0000) == 0xff000000) return; 
	//USBHDBGSerial.printf("KeyboardController: topusage= %x usage=%X, value=%d\n", topusage_, usage, value);
	// If this is the TOPUSAGE_KEYBOARD do in it's own function
	if (process_hid_keyboard_data(usage, value))
		return;

	// See if the value is in our keys_down list
	usage &= 0xffff;		// only keep the actual key
	if (usage == 0) return;	// lets not process 0, if only 0 happens, we will handle it on the end to remove existing pressed items.

	// Remember if we have received any logical key up events.  Some keyboard appear to send them
	// others do no...
	hid_input_data_ = true;

	uint8_t key_index;
	for (key_index = 0; key_index < count_keys_down_; key_index++) {
		if (keys_down[key_index] == usage) {
			if (value) return;		// still down

			if (extrasKeyReleasedFunction) {
				extrasKeyReleasedFunction(topusage_, usage);
			}

			// Remove from list
			count_keys_down_--;
			for (;key_index < count_keys_down_; key_index++) {
				keys_down[key_index] = keys_down[key_index+1];
			}
			return;
		}
	}
	// Was not in list
	if (!value) return;	// still 0
	if (extrasKeyPressedFunction) {
		extrasKeyPressedFunction(topusage_, usage);
	}
	if (count_keys_down_ < MAX_KEYS_DOWN) {
		keys_down[count_keys_down_++] = usage;
	}
}

bool KeyboardController::process_hid_keyboard_data(uint32_t usage, int32_t value)
{
	print("process_hid_keyboard_data Usage: ", usage, HEX);
	println(" value: ", value);
	//Serial.printf("process_hid_keyboard_data %x=%d\n", usage, value);

	if ((topusage_ & 0xffff0000) != (TOPUSAGE_KEYBOARD & 0xffff0000)) return false;
	// Lets first process modifier keys...
	// usage=700E0, value=0 (Left Control)
	// usage=700E1, value=0 (Left Shift)
	// usage=700E2, value=0 (Left Alt)
	// usage=700E3, value=0 (Left GUI)
	// usage=700E4, value=0 (Right Control)
	// usage=700E5, value=0 (Right Shift)
	// usage=700E6, value=0 (Right Alt)
	// usage=700E7, value=0 (Right GUI)
	if ((usage >= 0x700E0) && (usage <= 0x700E7)) {
		usage &= 7; 
		uint8_t keybit = 1 << usage;
		if (value) {
			if (!(modifiers_ & keybit))  {
				if (rawKeyPressedFunction) rawKeyPressedFunction(103 + usage);
				modifiers_ |= keybit;
			}

		} else {
			if (modifiers_ & keybit)  {
				if (rawKeyReleasedFunction) rawKeyReleasedFunction(103 + usage);
				modifiers_ &= ~keybit;
			}
		}
		return true;
	}


	// normal keys to be processed here. 
	// but two ways: for N key we receive an index per item
	// with Boot, we get an array of these items:

	if ((usage >= 0x70000) && (usage <= 0x70073)) {
		usage &= 0xff; // only use the low byte
		if (topusage_type_ & 0x2) {
			//normal variable - so use bitindex array to figure out what is new and what is old
			uint8_t key_byte_index = usage >> 3; //which byte in key_states_.
			uint8_t key_bit_mask = 1 << (usage & 0x7);

			if (value) {
				if (!(key_states_[key_byte_index] & key_bit_mask))  {
					key_press(modifiers_, usage);
					if (rawKeyPressedFunction) rawKeyPressedFunction(usage);
					key_states_[key_byte_index] |= key_bit_mask;
				}

			} else {
				if (key_states_[key_byte_index] & key_bit_mask)  {
					key_release(modifiers_, usage);
					if (rawKeyReleasedFunction) rawKeyReleasedFunction(usage);
					key_states_[key_byte_index] &= ~key_bit_mask;
				}
			}
		} else {
			// So array, We only see what keys are down.
			if (topusage_index_ < 8) {
				report_[topusage_index_++] = usage;
			}
		}
		return true;
	}
	return false;
}

void KeyboardController::hid_input_end()
{
	//USBHDBGSerial.printf("KPC:hid_input_end %u %u\n", hid_input_begin_, hid_input_data_);
	if (hid_input_begin_) {
		if (((topusage_type_ & 0x2) == 0) && (topusage_index_ > 2)) {
			// we have boot data.
			process_boot_keyboard_format(report_, false);
		}
		else if (!hid_input_data_ ) {
			if (extrasKeyReleasedFunction) {
				while (count_keys_down_) {
					count_keys_down_--;
					extrasKeyReleasedFunction(topusage_, keys_down[count_keys_down_]);
				}
			}
			count_keys_down_ = 0;
		}

		hid_input_begin_ = false;
	}		
}

bool KeyboardController::claim_bluetooth(BluetoothController *driver, uint32_t bluetooth_class, uint8_t *remoteName) 
{
	USBHDBGSerial.printf("Keyboard Controller::claim_bluetooth - Class %x\n", bluetooth_class);
	// If we are already in use than don't grab another one.  Likewise don't grab if it is used as USB or HID object
	if (btdevice && (btdevice != (Device_t*)driver)) return false;
	if (mydevice != NULL) return false;

	if ((((bluetooth_class & 0xff00) == 0x2500) || (((bluetooth_class & 0xff00) == 0x500))) && (bluetooth_class & 0x40)) {
		if (remoteName && (strncmp((const char *)remoteName, "PLAYSTATION(R)3", 15) == 0)) {
			USBHDBGSerial.printf("KeyboardController::claim_bluetooth Reject PS3 hack\n");
			btdevice = nullptr;	// remember this way 

			return false;
		}
		USBHDBGSerial.printf("KeyboardController::claim_bluetooth TRUE\n");
		btdevice = (Device_t*)driver;	// remember this way 
		return true;
	}
	return false;
}

bool KeyboardController::remoteNameComplete(const uint8_t *remoteName) 
{
	// Real Hack some PS3 controllers bluetoot class is keyboard... 
	if (strncmp((const char *)remoteName, "PLAYSTATION(R)3", 15) == 0) {
		USBHDBGSerial.printf("  KeyboardController::remoteNameComplete %s - Oops PS3 unclaim\n", remoteName);
		return false;
	}
	return true;
}




bool KeyboardController::process_bluetooth_HID_data(const uint8_t *data, uint16_t length) 
{
	// Example DATA from bluetooth keyboard:
	//                  0  1 2 3 4 5  6 7  8 910 1 2 3 4 5 6 7
	//                           LEN         D
	//BT rx2_data(18): 48 20 e 0 a 0 70 0 a1 1 2 0 0 0 0 0 0 0 
	//BT rx2_data(18): 48 20 e 0 a 0 70 0 a1 1 2 0 4 0 0 0 0 0 
	//BT rx2_data(18): 48 20 e 0 a 0 70 0 a1 1 2 0 0 0 0 0 0 0 
	// So Len=9 passed in data starting at report ID=1... 
	USBHDBGSerial.printf("KeyboardController::process_bluetooth_HID_data\n");
	if (data[0] != 1) return false;
	print("  KB Data: ");
	print_hexbytes(data, length);
	for (int i=2; i < length; i++) {
		uint32_t key = prev_report_[i];
		if (key >= 4 && !contains(key, &data[1])) {
			key_release(prev_report_[0], key);
		}
	}
	for (int i=2; i < 8; i++) {
		uint32_t key = data[i];
		if (key >= 4 && !contains(key, prev_report_)) {
			key_press(data[1], key);
		}
	}
	// Save away the data.. But shift down one byte... Don't need the report number
	memcpy(prev_report_, &data[1], 8);
	return true;
}

void KeyboardController::release_bluetooth() 
{
	btdevice = nullptr;
}

//*****************************************************************************
// Some simple query functions depend on which interface we are using...
//*****************************************************************************

uint16_t KeyboardController::idVendor() 
{
	if (mydevice != nullptr) return mydevice->idVendor;
	if (btdevice != nullptr) return btdevice->idVendor;
	return 0;
}

uint16_t KeyboardController::idProduct() 
{
	if (mydevice != nullptr) return mydevice->idProduct;
	if (btdevice != nullptr) return btdevice->idProduct;
	return 0;
}

const uint8_t *KeyboardController::manufacturer()
{
	if ((btdevice != nullptr) && (btdevice->strbuf != nullptr)) return &btdevice->strbuf->buffer[btdevice->strbuf->iStrings[strbuf_t::STR_ID_MAN]]; 
	if ((mydevice != nullptr) && (mydevice->strbuf != nullptr)) return &mydevice->strbuf->buffer[mydevice->strbuf->iStrings[strbuf_t::STR_ID_MAN]]; 
	return nullptr;
}

const uint8_t *KeyboardController::product()
{
	if ((mydevice != nullptr) && (mydevice->strbuf != nullptr)) return &mydevice->strbuf->buffer[mydevice->strbuf->iStrings[strbuf_t::STR_ID_PROD]]; 
	if ((btdevice != nullptr) && (btdevice->strbuf != nullptr)) return &btdevice->strbuf->buffer[btdevice->strbuf->iStrings[strbuf_t::STR_ID_PROD]]; 
	return nullptr;
}

const uint8_t *KeyboardController::serialNumber()
{
	if ((mydevice != nullptr) && (mydevice->strbuf != nullptr)) return &mydevice->strbuf->buffer[mydevice->strbuf->iStrings[strbuf_t::STR_ID_SERIAL]]; 
	if ((btdevice != nullptr) && (btdevice->strbuf != nullptr)) return &btdevice->strbuf->buffer[btdevice->strbuf->iStrings[strbuf_t::STR_ID_SERIAL]]; 
	return nullptr;
}

#ifdef USBHOST_PRINT_DEBUG
#undef print
#undef println

void KeyboardController::print_hexbytes(const void *ptr, uint32_t len)
{
	if (ptr == NULL || len == 0) return;
	const uint8_t *p = (const uint8_t *)ptr;
	do {
		if (*p < 16) USBHDBGSerial.print('0');
		USBHDBGSerial.print(*p++, HEX);
		USBHDBGSerial.print(' ');
	} while (--len);
	USBHDBGSerial.println();
}
#endif