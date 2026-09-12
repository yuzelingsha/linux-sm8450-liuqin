// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nanosic WN8030 magnetic-keyboard bridge, as fitted to the Xiaomi Pad 6 Pro.
 *
 * The bridge chip lives on the tablet side of the pogo-pin connector, on I2C,
 * and talks to a second MCU inside the detachable folio keyboard. It is not an
 * HID-over-I2C device: there is no HID descriptor register and no __HID_I2C
 * command set. What it does speak is a private fixed-length framing whose
 * payload is a run of plain HID input reports, each starting with its own
 * report ID. So everything above the framing already is HID, and the only job
 * left for a driver is to unwrap frames and hand the reports to the HID core -
 * which is exactly the shape of a transport driver.
 *
 * Four virtual HID devices are created rather than one, because the touchpad
 * report descriptor contains Contact ID: hid_scan_input_usage() then puts that
 * device in HID_GROUP_MULTITOUCH and hid-multitouch's wildcard match claims it,
 * which is how the touchpad gets slots, palm rejection and gestures for free.
 * Merging the four collections into a single device would hand the keyboard and
 * the mouse to hid-multitouch as well. The report descriptors are constants
 * below because the chip supplies none; they are the vendor driver's arrays,
 * with the touchpad's physical extents corrected (see NANOSIC_TP_PHYSICAL_*).
 *
 * Deliberately absent, all of it present in the vendor driver:
 *
 *  - The hall GPIOs. tlmm 10 and 23 are already described as gpio-keys
 *    SW_LID / SW_TABLET_MODE in sm8475-xiaomi-liuqin.dts, which is both the
 *    better semantics and an exclusive claim - a second consumer would fail
 *    with -EBUSY and take the whole probe down with it. The vendor driver only
 *    ever forwarded those levels to a Xiaomi userspace daemon. Do not "restore"
 *    them here.
 *  - The /dev/nanodev0 character device. It exists to give that same daemon a
 *    raw I2C write channel for keyboard-MCU firmware updates; an unauthenticated
 *    path to write arbitrary bytes onto an I2C bus is not something to recreate.
 *  - Any I2C transaction during probe. The vendor disabled its own detect hook
 *    for this reason: the bridge's boot-to-application time is not fixed, so a
 *    version read at probe time fails at random and would take the driver with
 *    it. The device attaches on interrupt-driven data instead.
 */

#include <linux/cleanup.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/hid.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

/*
 * Frame layout, bridge to AP:
 *
 *	byte 0	 0x57, magic; anything else is discarded
 *	byte 1	 sequence number, never validated by the vendor either
 *	byte 2	 0x39 / 0x4a / 0x5b / 0x6c mark a frame carrying reports;
 *		 0x00 means "nothing happened", which is what a spurious
 *		 interrupt reads back
 *	byte 3+	 back-to-back HID input reports, each identified by its
 *		 first byte, each of a length fixed by that byte - the
 *		 framing carries no length field of its own
 *
 * A read is two messages: one byte written, then a fixed 68 back. The written
 * byte is a register address that is not one, the bridge simply expects its own
 * slave address there.
 */
#define NANOSIC_FRAME_MAGIC		0x57
#define NANOSIC_FRAME_LEN		68
#define NANOSIC_FRAME_HDR_LEN		3
#define NANOSIC_FRAME_MARK_EMPTY	0x00

/*
 * Vendor firmware-version request. The write starts with the bridge address as
 * its register byte, followed by the vendor's fixed 66-byte payload. Bytes not
 * shown in the initializer are the required zero padding.
 */
#define NANOSIC_VERSION_COMMAND_LEN	67
#define NANOSIC_VERSION_RETRIES		3
#define NANOSIC_VERSION_RESPONSE_US	2000
#define NANOSIC_VERSION_LEN		20
#define NANOSIC_VERSION_OFFSET		7

static const u8 nanosic_version_command[NANOSIC_VERSION_COMMAND_LEN] = {
	0x4c, 0x32, 0x00, 0x4f, 0x30, 0x80, 0x18, 0x01, 0x00, 0x18,
};

/* Report IDs, which double as the framing's subpacket tags. */
#define NANOSIC_ID_MOUSE		0x02
#define NANOSIC_ID_KEYBOARD		0x05
#define NANOSIC_ID_CONSUMER		0x06
#define NANOSIC_ID_TOUCHPAD		0x19
#define NANOSIC_ID_VENDOR_16		0x22
#define NANOSIC_ID_VENDOR_32		0x23

#define NANOSIC_VENDOR_ID		0x15d9

/*
 * Runtime power. The wake line asks the bridge to stay awake; it is dropped
 * after this long without traffic and raised again before the next transfer.
 * The bridge wakes itself when the folio keyboard has something to send, so
 * dropping the line costs no events, only the latency of bringing its I2C back
 * up - which the vendor measured at up to 25 ms (nano_pm.c).
 */
#define NANOSIC_IDLE_TIMEOUT_MS		20000
#define NANOSIC_I2C_RETRY_US		25000
#define NANOSIC_RESET_ASSERT_MS		100
#define NANOSIC_RESET_READY_MS		500

static const u8 nanosic_keyboard_rdesc[] = {
	0x05, 0x01,		/* Usage Page (Generic Desktop)		*/
	0x09, 0x06,		/* Usage (Keyboard)			*/
	0xa1, 0x01,		/* Collection (Application)		*/
	0x85, NANOSIC_ID_KEYBOARD, /*   Report ID (5)			*/
	0x05, 0x07,		/*   Usage Page (Keyboard/Keypad)	*/
	0x19, 0xe0,		/*   Usage Minimum (LeftControl)	*/
	0x29, 0xe7,		/*   Usage Maximum (Right GUI)		*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x25, 0x01,		/*   Logical Maximum (1)		*/
	0x75, 0x01,		/*   Report Size (1)			*/
	0x95, 0x08,		/*   Report Count (8)			*/
	0x81, 0x02,		/*   Input (Data,Var,Abs)		*/
	0x81, 0x03,		/*   Input (Cnst,Var,Abs) - reserved	*/
	0x95, 0x05,		/*   Report Count (5)			*/
	0x05, 0x08,		/*   Usage Page (LEDs)			*/
	0x19, 0x01,		/*   Usage Minimum (Num Lock)		*/
	0x29, 0x05,		/*   Usage Maximum (Kana)		*/
	0x91, 0x02,		/*   Output (Data,Var,Abs)		*/
	0x95, 0x01,		/*   Report Count (1)			*/
	0x75, 0x03,		/*   Report Size (3)			*/
	0x91, 0x01,		/*   Output (Cnst,Ary,Abs) - padding	*/
	0x95, 0x06,		/*   Report Count (6)			*/
	0x75, 0x08,		/*   Report Size (8)			*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x26, 0xa4, 0x00,	/*   Logical Maximum (164)		*/
	0x05, 0x07,		/*   Usage Page (Keyboard/Keypad)	*/
	0x19, 0x00,		/*   Usage Minimum (0)			*/
	0x2a, 0xa4, 0x00,	/*   Usage Maximum (164)		*/
	0x81, 0x00,		/*   Input (Data,Ary,Abs)		*/
	0xc0,			/* End Collection			*/
};

static const u8 nanosic_consumer_rdesc[] = {
	0x05, 0x0c,		/* Usage Page (Consumer)		*/
	0x09, 0x01,		/* Usage (Consumer Control)		*/
	0xa1, 0x01,		/* Collection (Application)		*/
	0x85, NANOSIC_ID_CONSUMER, /*   Report ID (6)			*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x26, 0x80, 0x03,	/*   Logical Maximum (896)		*/
	0x19, 0x00,		/*   Usage Minimum (0)			*/
	0x2a, 0x80, 0x03,	/*   Usage Maximum (896)		*/
	0x75, 0x10,		/*   Report Size (16)			*/
	0x95, 0x01,		/*   Report Count (1)			*/
	0x81, 0x00,		/*   Input (Data,Ary,Abs)		*/
	0xc0,			/* End Collection			*/
};

static const u8 nanosic_mouse_rdesc[] = {
	0x05, 0x01,		/* Usage Page (Generic Desktop)		*/
	0x09, 0x02,		/* Usage (Mouse)			*/
	0xa1, 0x01,		/* Collection (Application)		*/
	0x85, NANOSIC_ID_MOUSE,	/*   Report ID (2)			*/
	0x09, 0x01,		/*   Usage (Pointer)			*/
	0xa1, 0x00,		/*   Collection (Physical)		*/
	0x05, 0x09,		/*     Usage Page (Button)		*/
	0x19, 0x01,		/*     Usage Minimum (Button 1)		*/
	0x29, 0x05,		/*     Usage Maximum (Button 5)		*/
	0x15, 0x00,		/*     Logical Minimum (0)		*/
	0x25, 0x01,		/*     Logical Maximum (1)		*/
	0x95, 0x05,		/*     Report Count (5)			*/
	0x75, 0x01,		/*     Report Size (1)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x95, 0x01,		/*     Report Count (1)			*/
	0x75, 0x03,		/*     Report Size (3)			*/
	0x81, 0x01,		/*     Input (Cnst,Ary,Abs) - padding	*/
	0x05, 0x01,		/*     Usage Page (Generic Desktop)	*/
	0x09, 0x30,		/*     Usage (X)			*/
	0x09, 0x31,		/*     Usage (Y)			*/
	0x09, 0x38,		/*     Usage (Wheel)			*/
	0x16, 0x00, 0x80,	/*     Logical Minimum (-32768)		*/
	0x26, 0xff, 0x7f,	/*     Logical Maximum (32767)		*/
	0x75, 0x10,		/*     Report Size (16)			*/
	0x95, 0x03,		/*     Report Count (3)			*/
	0x81, 0x06,		/*     Input (Data,Var,Rel)		*/
	0xc0,			/*   End Collection			*/
	0xc0,			/* End Collection			*/
};

/*
 * Touchpad geometry.
 *
 * The logical range is the bridge's own and is not negotiable. The physical
 * extents are: the vendor descriptor declared 363 x 214 with Unit(Inch) and
 * Unit Exponent -3, i.e. a touchpad 9.2 mm by 5.4 mm. It plainly is not one -
 * those two numbers are the display's size in millimetres, carried over from
 * constants still named MI_DISPLAY_X_SIZE_DESC / MI_DISPLAY_Y_SIZE_DESC and
 * reinterpreted as thousandths of an inch. hid-multitouch would derive a
 * resolution near 7900 units/inch from that, and libinput would be pointer-
 * accelerating, palm-rejecting and gesture-thresholding against a pad it
 * believes is under a centimetre wide, if it accepted it as a touchpad at all.
 *
 * The descriptor is ours, so the numbers below simply replace them. They are
 * PROVISIONAL and want a ruler on the real folio: 110.0 mm is the typical
 * active width of an 11-inch keyboard-folio touchpad, and the height follows
 * from the 2879:1799 logical aspect ratio (1.600), so the resolution comes out
 * uniform on both axes at ~664 units/inch. Both are expressed in the unit the
 * descriptor already declares, thousandths of an inch.
 */
#define NANOSIC_TP_LOGICAL_MAX_X	2879
#define NANOSIC_TP_LOGICAL_MAX_Y	1799
#define NANOSIC_TP_PHYSICAL_MAX_X	4331	/* 110.00 mm, provisional */
#define NANOSIC_TP_PHYSICAL_MAX_Y	2707	/*  68.75 mm, provisional */

/*
 * Contacts the wire format has room for. The 21-byte touchpad report is one
 * button byte plus three six-byte finger records plus the contact count, and
 * the bridge never fills more than the three. It is reported through
 * raw_request() below rather than left to hid-multitouch's fallback, which
 * would take the Contact Count Maximum field's logical maximum of 8 and
 * allocate five slots that can never carry a finger.
 */
#define NANOSIC_TP_CONTACTS		3

static const u8 nanosic_touchpad_rdesc[] = {
	0x05, 0x0d,		/* Usage Page (Digitizer)		*/
	0x09, 0x05,		/* Usage (Touch Pad)			*/
	0xa1, 0x01,		/* Collection (Application)		*/
	0x85, NANOSIC_ID_TOUCHPAD, /*   Report ID (25)			*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x25, 0x01,		/*   Logical Maximum (1)		*/
	0x35, 0x00,		/*   Physical Minimum (0)		*/
	0x45, 0x01,		/*   Physical Maximum (1)		*/
	0x75, 0x01,		/*   Report Size (1)			*/
	0x95, 0x02,		/*   Report Count (2)			*/
	0x05, 0x09,		/*   Usage Page (Button)		*/
	0x09, 0x01,		/*   Usage (Button 1)			*/
	0x09, 0x02,		/*   Usage (Button 2)			*/
	0x81, 0x02,		/*   Input (Data,Var,Abs)		*/
	0x95, 0x06,		/*   Report Count (6)			*/
	0x81, 0x01,		/*   Input (Cnst,Ary,Abs) - padding	*/

	/* Contact 1 */
	0x05, 0x0d,		/*   Usage Page (Digitizer)		*/
	0x09, 0x22,		/*   Usage (Finger)			*/
	0xa1, 0x02,		/*   Collection (Logical)		*/
	0x09, 0x42,		/*     Usage (Tip Switch)		*/
	0x15, 0x00,		/*     Logical Minimum (0)		*/
	0x25, 0x01,		/*     Logical Maximum (1)		*/
	0x75, 0x01,		/*     Report Size (1)			*/
	0x95, 0x01,		/*     Report Count (1)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x09, 0x32,		/*     Usage (In Range)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x09, 0x47,		/*     Usage (Confidence)		*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x95, 0x05,		/*     Report Count (5)			*/
	0x81, 0x03,		/*     Input (Cnst,Var,Abs) - padding	*/
	0x75, 0x08,		/*     Report Size (8)			*/
	0x09, 0x51,		/*     Usage (Contact Identifier)	*/
	0x95, 0x01,		/*     Report Count (1)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x05, 0x01,		/*     Usage Page (Generic Desktop)	*/
	0x15, 0x00,		/*     Logical Minimum (0)		*/
	0x26, NANOSIC_TP_LOGICAL_MAX_X & 0xff,
	      NANOSIC_TP_LOGICAL_MAX_X >> 8,
	0x75, 0x10,		/*     Report Size (16)			*/
	0x55, 0x0d,		/*     Unit Exponent (-3)		*/
	0x65, 0x13,		/*     Unit (Inch, English Linear)	*/
	0x09, 0x30,		/*     Usage (X)			*/
	0x35, 0x00,		/*     Physical Minimum (0)		*/
	0x46, NANOSIC_TP_PHYSICAL_MAX_X & 0xff,
	      NANOSIC_TP_PHYSICAL_MAX_X >> 8,
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x09, 0x31,		/*     Usage (Y)			*/
	0x26, NANOSIC_TP_LOGICAL_MAX_Y & 0xff,
	      NANOSIC_TP_LOGICAL_MAX_Y >> 8,
	0x46, NANOSIC_TP_PHYSICAL_MAX_Y & 0xff,
	      NANOSIC_TP_PHYSICAL_MAX_Y >> 8,
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0xc0,			/*   End Collection			*/

	/* Contact 2 */
	0xa1, 0x02,		/*   Collection (Logical)		*/
	0x05, 0x0d,		/*     Usage Page (Digitizer)		*/
	0x09, 0x42,		/*     Usage (Tip Switch)		*/
	0x15, 0x00,		/*     Logical Minimum (0)		*/
	0x25, 0x01,		/*     Logical Maximum (1)		*/
	0x75, 0x01,		/*     Report Size (1)			*/
	0x95, 0x01,		/*     Report Count (1)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x09, 0x32,		/*     Usage (In Range)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x09, 0x47,		/*     Usage (Confidence)		*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x95, 0x05,		/*     Report Count (5)			*/
	0x81, 0x03,		/*     Input (Cnst,Var,Abs) - padding	*/
	0x75, 0x08,		/*     Report Size (8)			*/
	0x09, 0x51,		/*     Usage (Contact Identifier)	*/
	0x95, 0x01,		/*     Report Count (1)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x05, 0x01,		/*     Usage Page (Generic Desktop)	*/
	0x15, 0x00,		/*     Logical Minimum (0)		*/
	0x26, NANOSIC_TP_LOGICAL_MAX_X & 0xff,
	      NANOSIC_TP_LOGICAL_MAX_X >> 8,
	0x75, 0x10,		/*     Report Size (16)			*/
	0x55, 0x0d,		/*     Unit Exponent (-3)		*/
	0x65, 0x13,		/*     Unit (Inch, English Linear)	*/
	0x09, 0x30,		/*     Usage (X)			*/
	0x35, 0x00,		/*     Physical Minimum (0)		*/
	0x46, NANOSIC_TP_PHYSICAL_MAX_X & 0xff,
	      NANOSIC_TP_PHYSICAL_MAX_X >> 8,
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x09, 0x31,		/*     Usage (Y)			*/
	0x26, NANOSIC_TP_LOGICAL_MAX_Y & 0xff,
	      NANOSIC_TP_LOGICAL_MAX_Y >> 8,
	0x46, NANOSIC_TP_PHYSICAL_MAX_Y & 0xff,
	      NANOSIC_TP_PHYSICAL_MAX_Y >> 8,
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0xc0,			/*   End Collection			*/

	/* Contact 3 */
	0xa1, 0x02,		/*   Collection (Logical)		*/
	0x05, 0x0d,		/*     Usage Page (Digitizer)		*/
	0x09, 0x42,		/*     Usage (Tip Switch)		*/
	0x15, 0x00,		/*     Logical Minimum (0)		*/
	0x25, 0x01,		/*     Logical Maximum (1)		*/
	0x75, 0x01,		/*     Report Size (1)			*/
	0x95, 0x01,		/*     Report Count (1)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x09, 0x32,		/*     Usage (In Range)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x09, 0x47,		/*     Usage (Confidence)		*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x95, 0x05,		/*     Report Count (5)			*/
	0x81, 0x03,		/*     Input (Cnst,Var,Abs) - padding	*/
	0x75, 0x08,		/*     Report Size (8)			*/
	0x09, 0x51,		/*     Usage (Contact Identifier)	*/
	0x95, 0x01,		/*     Report Count (1)			*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x05, 0x01,		/*     Usage Page (Generic Desktop)	*/
	0x15, 0x00,		/*     Logical Minimum (0)		*/
	0x26, NANOSIC_TP_LOGICAL_MAX_X & 0xff,
	      NANOSIC_TP_LOGICAL_MAX_X >> 8,
	0x75, 0x10,		/*     Report Size (16)			*/
	0x55, 0x0d,		/*     Unit Exponent (-3)		*/
	0x65, 0x13,		/*     Unit (Inch, English Linear)	*/
	0x09, 0x30,		/*     Usage (X)			*/
	0x35, 0x00,		/*     Physical Minimum (0)		*/
	0x46, NANOSIC_TP_PHYSICAL_MAX_X & 0xff,
	      NANOSIC_TP_PHYSICAL_MAX_X >> 8,
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0x09, 0x31,		/*     Usage (Y)			*/
	0x26, NANOSIC_TP_LOGICAL_MAX_Y & 0xff,
	      NANOSIC_TP_LOGICAL_MAX_Y >> 8,
	0x46, NANOSIC_TP_PHYSICAL_MAX_Y & 0xff,
	      NANOSIC_TP_PHYSICAL_MAX_Y >> 8,
	0x81, 0x02,		/*     Input (Data,Var,Abs)		*/
	0xc0,			/*   End Collection			*/

	0x05, 0x0d,		/*   Usage Page (Digitizer)		*/
	0x09, 0x54,		/*   Usage (Contact Count)		*/
	0x95, 0x01,		/*   Report Count (1)			*/
	0x75, 0x08,		/*   Report Size (8)			*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x25, 0x08,		/*   Logical Maximum (8)		*/
	0x81, 0x02,		/*   Input (Data,Var,Abs)		*/
	0x09, 0x55,		/*   Usage (Contact Count Maximum)	*/
	0xb1, 0x02,		/*   Feature (Data,Var,Abs)		*/
	0xc0,			/* End Collection			*/
};

enum nanosic_hid_index {
	NANOSIC_HID_KEYBOARD,
	NANOSIC_HID_CONSUMER,
	NANOSIC_HID_MOUSE,
	NANOSIC_HID_TOUCHPAD,
	NANOSIC_HID_COUNT
};

struct nanosic_hid_info {
	const char *name;
	const u8 *rdesc;
	unsigned int rdesc_size;
	u32 product;
};

/*
 * Names and product IDs are the vendor driver's, so that any userspace already
 * keyed on them - libinput quirks, udev hwdb entries - keeps matching.
 */
static const struct nanosic_hid_info nanosic_hid_info[NANOSIC_HID_COUNT] = {
	[NANOSIC_HID_KEYBOARD] = {
		.name = "Xiaomi Keyboard",
		.rdesc = nanosic_keyboard_rdesc,
		.rdesc_size = sizeof(nanosic_keyboard_rdesc),
		.product = 0x00a3,
	},
	[NANOSIC_HID_CONSUMER] = {
		.name = "Xiaomi Consumer",
		.rdesc = nanosic_consumer_rdesc,
		.rdesc_size = sizeof(nanosic_consumer_rdesc),
		.product = 0x00a4,
	},
	[NANOSIC_HID_MOUSE] = {
		.name = "Xiaomi Mouse",
		.rdesc = nanosic_mouse_rdesc,
		.rdesc_size = sizeof(nanosic_mouse_rdesc),
		.product = 0x00a2,
	},
	[NANOSIC_HID_TOUCHPAD] = {
		.name = "Xiaomi Touch",
		.rdesc = nanosic_touchpad_rdesc,
		.rdesc_size = sizeof(nanosic_touchpad_rdesc),
		.product = 0x00a1,
	},
};

/*
 * Subpacket table. wire_len is what the report occupies inside the frame and
 * is fixed per report ID; report_len is what gets injected. The two differ for
 * the consumer report, which the bridge pads to five bytes while the descriptor
 * above declares three - handing the HID core the extra two only earns a
 * truncation warning on every media key. dest is -1 for the vendor-diagnostic
 * reports, which have no consumer here and are skipped over so that a report
 * behind them in the same frame still gets parsed.
 */
struct nanosic_packet {
	u8 id;
	u8 wire_len;
	u8 report_len;
	int dest;
};

static const struct nanosic_packet nanosic_packets[] = {
	{ NANOSIC_ID_MOUSE,	 8,  8, NANOSIC_HID_MOUSE },
	{ NANOSIC_ID_KEYBOARD,	 9,  9, NANOSIC_HID_KEYBOARD },
	{ NANOSIC_ID_CONSUMER,	 5,  3, NANOSIC_HID_CONSUMER },
	{ NANOSIC_ID_TOUCHPAD,	21, 21, NANOSIC_HID_TOUCHPAD },
	{ NANOSIC_ID_VENDOR_16,	16,  0, -1 },
	{ NANOSIC_ID_VENDOR_32,	32,  0, -1 },
};

enum nanosic_i2c_operation {
	NANOSIC_I2C_NONE,
	NANOSIC_I2C_DATA_READ,
	NANOSIC_I2C_VERSION_WRITE,
	NANOSIC_I2C_VERSION_READ,
};

struct nanosic {
	struct i2c_client *client;
	struct regulator *vdd;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *wake_gpio;
	struct hid_device *hid[NANOSIC_HID_COUNT];

	/* Serialises the wake line, the awake flag and the I2C buffers. */
	struct mutex lock;
	struct delayed_work idle_work;
	unsigned long wake_deadline;
	bool awake;
	bool suspended;
	bool data_pending;

	int data_irq;
	bool data_irq_wake_enabled;

	u64 irq_count;
	u64 i2c_transfer_count;
	u64 i2c_error_count;
	u64 last_i2c_error_transfer;
	u64 reset_recovery_count;
	u64 bad_magic_count;
	u64 valid_report_count;
	int last_i2c_errno;
	enum nanosic_i2c_operation last_i2c_operation;

	u8 reg;
	u8 rxbuf[NANOSIC_FRAME_LEN];
};

static const struct nanosic_packet *nanosic_packet_by_id(u8 id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(nanosic_packets); i++)
		if (nanosic_packets[i].id == id)
			return &nanosic_packets[i];

	return NULL;
}

static void nanosic_idle_work(struct work_struct *work)
{
	struct nanosic *nano = container_of(to_delayed_work(work),
					    struct nanosic, idle_work);

	guard(mutex)(&nano->lock);

	if (nano->suspended)
		return;

	/*
	 * mod_delayed_work() cannot stop an instance that is already running
	 * and blocked on this mutex. Re-checking the deadline after taking the
	 * lock prevents that stale instance from undoing a concurrent wake.
	 */
	if (time_before(jiffies, nano->wake_deadline)) {
		mod_delayed_work(system_wq, &nano->idle_work,
				 nano->wake_deadline - jiffies);
		return;
	}

	gpiod_set_value_cansleep(nano->wake_gpio, 0);
	nano->awake = false;
}

/*
 * Raise the wake line if it is down and restart the idle countdown. The vendor
 * driver instead sampled the interrupt line to decide whether the bridge was
 * already running; tracking who lowered the line is the same information from
 * the side that has it, and it keeps the GPIO out of the device tree binding.
 */
static void nanosic_wake(struct nanosic *nano)
{
	unsigned long delay = msecs_to_jiffies(NANOSIC_IDLE_TIMEOUT_MS);

	lockdep_assert_held(&nano->lock);

	if (!nano->awake) {
		gpiod_set_value_cansleep(nano->wake_gpio, 1);
		nano->awake = true;
	}

	nano->wake_deadline = jiffies + delay;
	mod_delayed_work(system_wq, &nano->idle_work, delay);
}

static const char *nanosic_i2c_operation_name(enum nanosic_i2c_operation operation)
{
	switch (operation) {
	case NANOSIC_I2C_DATA_READ:
		return "data-read";
	case NANOSIC_I2C_VERSION_WRITE:
		return "version-write";
	case NANOSIC_I2C_VERSION_READ:
		return "version-read";
	default:
		return "none";
	}
}

static int nanosic_i2c_transfer(struct nanosic *nano, struct i2c_msg *msg,
				int num, enum nanosic_i2c_operation operation)
{
	int ret;

	lockdep_assert_held(&nano->lock);

	nano->i2c_transfer_count++;
	ret = i2c_transfer(nano->client->adapter, msg, num);
	if (ret != num) {
		ret = ret < 0 ? ret : -EIO;
		nano->i2c_error_count++;
		nano->last_i2c_errno = ret;
		nano->last_i2c_operation = operation;
		nano->last_i2c_error_transfer = nano->i2c_transfer_count;
		return ret;
	}

	return 0;
}

static int nanosic_read_frame(struct nanosic *nano,
			      enum nanosic_i2c_operation operation)
{
	struct i2c_client *client = nano->client;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.len = sizeof(nano->reg),
			.buf = &nano->reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(nano->rxbuf),
			.buf = nano->rxbuf,
		},
	};
	lockdep_assert_held(&nano->lock);

	return nanosic_i2c_transfer(nano, msg, ARRAY_SIZE(msg), operation);
}

static int nanosic_write_version_command(struct nanosic *nano)
{
	struct i2c_client *client = nano->client;
	struct i2c_msg msg = {
		.addr = client->addr,
		.len = sizeof(nanosic_version_command),
		.buf = (u8 *)nanosic_version_command,
	};

	lockdep_assert_held(&nano->lock);

	/* The vendor command's register byte is specific to the 0x4c device. */
	if (client->addr != nanosic_version_command[0])
		return -ENXIO;

	return nanosic_i2c_transfer(nano, &msg, 1,
				    NANOSIC_I2C_VERSION_WRITE);
}

static void nanosic_reset_bridge(struct nanosic *nano)
{
	lockdep_assert_held(&nano->lock);

	/* Match the vendor's bounded recovery pulse and application-ready wait. */
	gpiod_set_value_cansleep(nano->reset_gpio, 1);
	msleep(NANOSIC_RESET_ASSERT_MS);
	gpiod_set_value_cansleep(nano->reset_gpio, 0);
	msleep(NANOSIC_RESET_READY_MS);
	nano->reset_recovery_count++;
}

static bool nanosic_version_error_needs_reset(int ret)
{
	/* GPI DMA reports callback failures as -EIO and no completion as timeout. */
	return ret == -ENXIO || ret == -EIO || ret == -ETIMEDOUT;
}

static int nanosic_extract_firmware_version(const u8 *frame, size_t len,
					    char *version)
{
	const struct nanosic_packet *pkt;
	const u8 *p;
	size_t left;
	size_t version_len;
	unsigned int i;

	if (len < NANOSIC_FRAME_HDR_LEN || frame[0] != NANOSIC_FRAME_MAGIC)
		return -ENOMSG;

	switch (frame[2]) {
	case 0x39:
	case 0x4a:
	case 0x5b:
	case 0x6c:
		break;
	default:
		return -ENOMSG;
	}

	p = frame + NANOSIC_FRAME_HDR_LEN;
	left = len - NANOSIC_FRAME_HDR_LEN;

	while (left) {
		pkt = nanosic_packet_by_id(p[0]);
		if (!pkt || !pkt->wire_len || pkt->wire_len > left)
			return -ENOMSG;

		if (p[0] == NANOSIC_ID_VENDOR_32 &&
		    p[2] == 0x18 && p[3] == 0x80 && p[4] == 0x01) {
			version_len = strnlen((const char *)p +
					      NANOSIC_VERSION_OFFSET,
					      NANOSIC_VERSION_LEN);
			if (!version_len)
				return -ENODATA;

			for (i = 0; i < version_len; i++)
				if (!isprint(p[NANOSIC_VERSION_OFFSET + i]))
					return -EBADMSG;

			memcpy(version, p + NANOSIC_VERSION_OFFSET, version_len);
			version[version_len] = '\0';
			return 0;
		}

		p += pkt->wire_len;
		left -= pkt->wire_len;
	}

	return -ENOMSG;
}

static void nanosic_parse_frame(struct nanosic *nano, u8 *frame, size_t len)
{
	struct device *dev = &nano->client->dev;
	u8 *p;
	size_t left;

	if (len < NANOSIC_FRAME_HDR_LEN)
		return;

	p = frame + NANOSIC_FRAME_HDR_LEN;
	left = len - NANOSIC_FRAME_HDR_LEN;

	if (frame[0] != NANOSIC_FRAME_MAGIC) {
		nano->bad_magic_count++;
		dev_dbg(dev, "bad frame magic 0x%02x\n", frame[0]);
		return;
	}

	switch (frame[2]) {
	case NANOSIC_FRAME_MARK_EMPTY:
		return;
	case 0x39:
	case 0x4a:
	case 0x5b:
	case 0x6c:
		break;
	default:
		dev_dbg(dev, "bad frame marker 0x%02x\n", frame[2]);
		return;
	}

	while (left) {
		const struct nanosic_packet *pkt = nanosic_packet_by_id(p[0]);
		int ret;

		/*
		 * Report IDs 0x24 and 0x26 run to the end of the frame and so
		 * carry no length of their own; every other unknown ID leaves
		 * nothing to resynchronise on. Either way the frame ends here.
		 */
		if (!pkt) {
			dev_dbg(dev, "unhandled report 0x%02x\n", p[0]);
			return;
		}

		if (!pkt->wire_len || pkt->report_len > pkt->wire_len ||
		    pkt->wire_len > left) {
			dev_dbg(dev, "report 0x%02x truncated\n", p[0]);
			return;
		}
		nano->valid_report_count++;

		if (pkt->dest >= 0) {
			ret = hid_input_report(nano->hid[pkt->dest],
					       HID_INPUT_REPORT, p,
					       pkt->report_len, 1);
			if (ret && ret != -ENODEV && ret != -EBUSY)
				dev_dbg(dev, "report 0x%02x rejected: %d\n",
					p[0], ret);
		}

		p += pkt->wire_len;
		left -= pkt->wire_len;
	}
}

static void nanosic_receive_frame(struct nanosic *nano)
{
	int ret;

	lockdep_assert_held(&nano->lock);

	ret = nanosic_read_frame(nano, NANOSIC_I2C_DATA_READ);
	if (ret) {
		/* Retry only an observed failure; the normal IRQ path never waits. */
		usleep_range(NANOSIC_I2C_RETRY_US,
			     NANOSIC_I2C_RETRY_US + 500);
		ret = nanosic_read_frame(nano, NANOSIC_I2C_DATA_READ);
	}
	if (ret) {
		dev_err_ratelimited(&nano->client->dev,
				    "frame read failed: %d\n", ret);
		return;
	}

	nanosic_parse_frame(nano, nano->rxbuf, sizeof(nano->rxbuf));
}

static int nanosic_query_firmware_version(struct nanosic *nano, char *version)
{
	bool reset_attempted = false;
	bool was_awake;
	unsigned int attempt;
	int ret = -ENODATA;

	lockdep_assert_held(&nano->lock);

	if (nano->suspended)
		return -EHOSTDOWN;

	was_awake = nano->awake;
	nanosic_wake(nano);
	if (!was_awake)
		usleep_range(NANOSIC_I2C_RETRY_US,
			     NANOSIC_I2C_RETRY_US + 500);

	for (attempt = 0; attempt < NANOSIC_VERSION_RETRIES; attempt++) {
		ret = nanosic_write_version_command(nano);
		if (nanosic_version_error_needs_reset(ret) && !reset_attempted &&
		    attempt + 1 < NANOSIC_VERSION_RETRIES) {
			nanosic_reset_bridge(nano);
			reset_attempted = true;
			continue;
		}
		if (ret)
			goto retry;

		usleep_range(NANOSIC_VERSION_RESPONSE_US,
			     NANOSIC_VERSION_RESPONSE_US + 500);
		ret = nanosic_read_frame(nano, NANOSIC_I2C_VERSION_READ);
		if (ret)
			goto retry;

		ret = nanosic_extract_firmware_version(nano->rxbuf,
						       sizeof(nano->rxbuf),
						       version);
		/* Preserve any input report that happened to arrive first. */
		nanosic_parse_frame(nano, nano->rxbuf, sizeof(nano->rxbuf));
		if (!ret)
			return 0;

retry:
		if (attempt + 1 < NANOSIC_VERSION_RETRIES)
			usleep_range(NANOSIC_I2C_RETRY_US,
				     NANOSIC_I2C_RETRY_US + 500);
	}

	return ret;
}

static irqreturn_t nanosic_data_irq(int irq, void *data)
{
	struct nanosic *nano = data;

	guard(mutex)(&nano->lock);
	nano->irq_count++;

	if (READ_ONCE(nano->suspended)) {
		nano->data_pending = true;
		pm_wakeup_event(&nano->client->dev, 1000);
		return IRQ_HANDLED;
	}

	nanosic_wake(nano);
	nanosic_receive_frame(nano);

	return IRQ_HANDLED;
}

static ssize_t firmware_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct nanosic *nano = dev_get_drvdata(dev);
	char version[NANOSIC_VERSION_LEN + 1];
	int ret;

	guard(mutex)(&nano->lock);

	ret = nanosic_query_firmware_version(nano, version);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%s\n", version);
}
static DEVICE_ATTR_RO(firmware_version);

static ssize_t bridge_health_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct nanosic *nano = dev_get_drvdata(dev);
	ssize_t len = 0;

	guard(mutex)(&nano->lock);

	len += sysfs_emit_at(buf, len, "irq=%llu\n",
			     (unsigned long long)nano->irq_count);
	len += sysfs_emit_at(buf, len, "i2c_transfers=%llu\n",
			     (unsigned long long)nano->i2c_transfer_count);
	len += sysfs_emit_at(buf, len, "i2c_errors=%llu\n",
			     (unsigned long long)nano->i2c_error_count);
	len += sysfs_emit_at(buf, len, "last_i2c_errno=%d\n",
			     nano->last_i2c_errno);
	len += sysfs_emit_at(buf, len, "last_i2c_operation=%s\n",
			     nanosic_i2c_operation_name(nano->last_i2c_operation));
	len += sysfs_emit_at(buf, len, "last_i2c_error_transfer=%llu\n",
			     (unsigned long long)nano->last_i2c_error_transfer);
	len += sysfs_emit_at(buf, len, "reset_recoveries=%llu\n",
			     (unsigned long long)nano->reset_recovery_count);
	len += sysfs_emit_at(buf, len, "bad_magic=%llu\n",
			     (unsigned long long)nano->bad_magic_count);
	len += sysfs_emit_at(buf, len, "valid_reports=%llu\n",
			     (unsigned long long)nano->valid_report_count);

	return len;
}
static DEVICE_ATTR_RO(bridge_health);

static struct attribute *nanosic_attrs[] = {
	&dev_attr_firmware_version.attr,
	&dev_attr_bridge_health.attr,
	NULL
};

static const struct attribute_group nanosic_attr_group = {
	.attrs = nanosic_attrs,
};

static int nanosic_hid_start(struct hid_device *hid)
{
	return 0;
}

static void nanosic_hid_stop(struct hid_device *hid)
{
}

static int nanosic_hid_open(struct hid_device *hid)
{
	return 0;
}

static void nanosic_hid_close(struct hid_device *hid)
{
}

static int nanosic_hid_index(struct hid_device *hid)
{
	struct nanosic *nano = dev_get_drvdata(hid->dev.parent);
	unsigned int i;

	for (i = 0; i < NANOSIC_HID_COUNT; i++)
		if (nano->hid[i] == hid)
			return i;

	return -ENODEV;
}

static int nanosic_hid_parse(struct hid_device *hid)
{
	int index = nanosic_hid_index(hid);

	if (index < 0)
		return index;

	return hid_parse_report(hid, nanosic_hid_info[index].rdesc,
				nanosic_hid_info[index].rdesc_size);
}

static int nanosic_hid_raw_request(struct hid_device *hid,
				   unsigned char reportnum, u8 *buf, size_t len,
				   unsigned char rtype, int reqtype)
{
	struct nanosic *nano = dev_get_drvdata(hid->dev.parent);

	/*
	 * The private framing has no request/response channel for HID at all,
	 * so the one request that can be served is the one hid-multitouch
	 * makes at probe time. Contact Count Maximum is a property of the
	 * descriptor above rather than of the hardware, and answering it here
	 * is what keeps the slot count at three.
	 */
	if (hid == nano->hid[NANOSIC_HID_TOUCHPAD] && len >= 2 &&
	    rtype == HID_FEATURE_REPORT && reqtype == HID_REQ_GET_REPORT &&
	    reportnum == NANOSIC_ID_TOUCHPAD) {
		buf[0] = NANOSIC_ID_TOUCHPAD;
		buf[1] = NANOSIC_TP_CONTACTS;
		return 2;
	}

	return -EOPNOTSUPP;
}

/*
 * No output_report and no writable raw_request: the keyboard descriptor
 * declares the five boot-protocol LEDs, but no command for setting them is
 * known on the downstream side of the bridge, and the vendor driver never sent
 * one either. Caps Lock therefore has no indicator; the alternative would be
 * guessing at bytes to write onto the bus.
 */
static const struct hid_ll_driver nanosic_hid_ll_driver = {
	.start = nanosic_hid_start,
	.stop = nanosic_hid_stop,
	.open = nanosic_hid_open,
	.close = nanosic_hid_close,
	.parse = nanosic_hid_parse,
	.raw_request = nanosic_hid_raw_request,
};

static void nanosic_destroy_hid(void *data)
{
	struct nanosic *nano = data;
	unsigned int i;

	for (i = 0; i < NANOSIC_HID_COUNT; i++)
		if (nano->hid[i])
			hid_destroy_device(nano->hid[i]);
}

static int nanosic_register_hid(struct nanosic *nano)
{
	struct device *dev = &nano->client->dev;
	unsigned int i;
	int ret;

	ret = devm_add_action_or_reset(dev, nanosic_destroy_hid, nano);
	if (ret)
		return ret;

	for (i = 0; i < NANOSIC_HID_COUNT; i++) {
		const struct nanosic_hid_info *info = &nanosic_hid_info[i];
		struct hid_device *hid;

		hid = hid_allocate_device();
		if (IS_ERR(hid))
			return PTR_ERR(hid);

		nano->hid[i] = hid;
		hid->ll_driver = &nanosic_hid_ll_driver;
		hid->dev.parent = dev;
		hid->bus = BUS_I2C;
		hid->vendor = NANOSIC_VENDOR_ID;
		hid->product = info->product;

		strscpy(hid->name, info->name, sizeof(hid->name));
		snprintf(hid->phys, sizeof(hid->phys), "%s/input%u",
			 dev_name(dev), i);

		ret = hid_add_device(hid);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add %s\n", info->name);
	}

	return 0;
}

static void nanosic_power_off(void *data)
{
	struct nanosic *nano = data;

	cancel_delayed_work_sync(&nano->idle_work);
	gpiod_set_value_cansleep(nano->wake_gpio, 0);
	gpiod_set_value_cansleep(nano->reset_gpio, 1);
	regulator_disable(nano->vdd);
}

static int nanosic_power_on(struct nanosic *nano)
{
	unsigned long delay = msecs_to_jiffies(NANOSIC_IDLE_TIMEOUT_MS);
	int ret;

	ret = regulator_enable(nano->vdd);
	if (ret)
		return ret;

	/*
	 * Reset is still asserted from the descriptor request. regulator_enable()
	 * has already honoured the fixed regulator's startup-delay-us, so release
	 * reset here without imposing that delay a second time. Nothing is read
	 * back: the bridge's boot-to-application time is not fixed and probing it
	 * here is what the vendor had to give up on.
	 */
	gpiod_set_value_cansleep(nano->reset_gpio, 0);
	gpiod_set_value_cansleep(nano->wake_gpio, 1);
	nano->awake = true;

	/*
	 * Start the countdown here rather than on the first interrupt. On a
	 * tablet with no folio attached there is never a first interrupt, and
	 * the wake line would otherwise be held high for the life of the
	 * system with nothing on the other end of it.
	 */
	nano->wake_deadline = jiffies + delay;
	mod_delayed_work(system_wq, &nano->idle_work, delay);

	return 0;
}

/*
 * The bridge brings out two interrupts. "data" is the one that matters: the
 * line idles high while the bridge runs and dips for about a millisecond to
 * announce a frame, so the rising edge at the end of that dip is the event.
 * "wakeup" fires from the status line while the system is suspended. The I2C
 * core claims that named IRQ as the device's dedicated wake IRQ before probe;
 * requesting it again here would fail with -EBUSY and roll the whole probe
 * back. The driver owns only the data IRQ and lets the PM core arm the
 * dedicated wake IRQ around suspend.
 */
static int nanosic_irqs_get(struct nanosic *nano)
{
	struct device *dev = &nano->client->dev;
	int ret;

	ret = fwnode_irq_get_byname(dev_fwnode(dev), "data");
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get data irq\n");

	nano->data_irq = ret;

	return 0;
}

static int nanosic_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct nanosic *nano;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	nano = devm_kzalloc(dev, sizeof(*nano), GFP_KERNEL);
	if (!nano)
		return -ENOMEM;

	nano->client = client;
	/* The bridge takes its own slave address as the register address. */
	nano->reg = client->addr;
	i2c_set_clientdata(client, nano);

	ret = devm_mutex_init(dev, &nano->lock);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&nano->idle_work, nanosic_idle_work);

	/*
	 * Only these two lines and the rail. The hall GPIOs stay with
	 * gpio-keys; see the file header before adding any.
	 */
	nano->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(nano->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(nano->reset_gpio),
				     "failed to get reset gpio\n");

	nano->wake_gpio = devm_gpiod_get(dev, "wake", GPIOD_OUT_LOW);
	if (IS_ERR(nano->wake_gpio))
		return dev_err_probe(dev, PTR_ERR(nano->wake_gpio),
				     "failed to get wake gpio\n");

	nano->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(nano->vdd))
		return dev_err_probe(dev, PTR_ERR(nano->vdd),
				     "failed to get vdd supply\n");

	ret = nanosic_irqs_get(nano);
	if (ret)
		return ret;

	ret = nanosic_power_on(nano);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	ret = devm_add_action_or_reset(dev, nanosic_power_off, nano);
	if (ret)
		return ret;

	/* The HID devices must exist before an interrupt can feed them. */
	ret = nanosic_register_hid(nano);
	if (ret)
		return ret;

	/*
	 * The vendor device tree hands request_threaded_irq() a flag word of
	 * IRQF_NO_SUSPEND | IRQF_ONESHOT | IRQF_TRIGGER_RISING verbatim. The
	 * trigger comes from the device tree here, and wakeup is expressed the
	 * way the kernel expects - enable_irq_wake() below - rather than by
	 * keeping the interrupt live across suspend, which is what
	 * IRQF_NO_SUSPEND would do without ever waking the system.
	 */
	ret = devm_request_threaded_irq(dev, nano->data_irq, NULL,
					nanosic_data_irq, IRQF_ONESHOT,
					"nanosic-data", nano);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request data irq\n");

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	/* Attribute reads are the only path which issues a version request. */
	return sysfs_create_group(&dev->kobj, &nanosic_attr_group);
}

static void nanosic_remove(struct i2c_client *client)
{
	/* Stop new callbacks while all devm-managed driver state is still live. */
	sysfs_remove_group(&client->dev.kobj, &nanosic_attr_group);
}

static int nanosic_suspend(struct device *dev)
{
	struct nanosic *nano = dev_get_drvdata(dev);
	int ret;

	/*
	 * Set the state under the same lock as the data IRQ before cancelling
	 * work. Any IRQ racing this transition will request a system wake rather
	 * than re-arm idle_work or raise the bridge wake line behind our back.
	 */
	scoped_guard(mutex, &nano->lock) {
		WRITE_ONCE(nano->suspended, true);
		gpiod_set_value_cansleep(nano->wake_gpio, 0);
		nano->awake = false;
	}
	cancel_delayed_work_sync(&nano->idle_work);

	if (!device_may_wakeup(dev))
		return 0;

	ret = enable_irq_wake(nano->data_irq);
	if (ret)
		goto restore_runtime;
	nano->data_irq_wake_enabled = true;

	return 0;

restore_runtime:
	scoped_guard(mutex, &nano->lock) {
		WRITE_ONCE(nano->suspended, false);
		nanosic_wake(nano);
		if (nano->data_pending) {
			nano->data_pending = false;
			nanosic_receive_frame(nano);
		}
	}

	return ret;
}

static int nanosic_resume(struct device *dev)
{
	struct nanosic *nano = dev_get_drvdata(dev);
	int err, ret = 0;

	if (nano->data_irq_wake_enabled) {
		err = disable_irq_wake(nano->data_irq);
		if (err && !ret)
			ret = err;
		nano->data_irq_wake_enabled = false;
	}

	scoped_guard(mutex, &nano->lock) {
		WRITE_ONCE(nano->suspended, false);
		nanosic_wake(nano);
		if (nano->data_pending) {
			nano->data_pending = false;
			nanosic_receive_frame(nano);
		}
	}

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nanosic_pm_ops, nanosic_suspend,
				nanosic_resume);

static const struct of_device_id nanosic_of_match[] = {
	{ .compatible = "nanosic,wn8030" },
	{ }
};
MODULE_DEVICE_TABLE(of, nanosic_of_match);

static const struct i2c_device_id nanosic_i2c_id[] = {
	{ "wn8030" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nanosic_i2c_id);

static struct i2c_driver nanosic_i2c_driver = {
	.driver = {
		.name = "hid-nanosic",
		.of_match_table = nanosic_of_match,
		.pm = pm_sleep_ptr(&nanosic_pm_ops),
	},
	.probe = nanosic_probe,
	.remove = nanosic_remove,
	.id_table = nanosic_i2c_id,
};
module_i2c_driver(nanosic_i2c_driver);

MODULE_DESCRIPTION("HID transport driver for the Nanosic WN8030 keyboard bridge");
MODULE_LICENSE("GPL");
