/*
  Control out (Co):
    40: Vendor output request:
      (LIBUSB_REQUEST_TYPE_VENDOR (0x40) | LIBUSB_ENDPOINT_OUT (0x00)):

      40 00 0000 0000 0000 0    - Reset port: Reset SIO
      40 00 0001 0000 0000 0    - Reset port: Purge RX buffer
      40 00 0002 0000 0000 0    - Reset port: Purge TX buffer
      40 03 4006 0000 0000 0    - Set baudrate
      40 09 0004 0000 0000 0    - Set latency timer
      40 0b 0000 0000 0000 0    - Set bit mode: standard
      40 0b 0107 0000 0000 0    - Set bit mode: bitbang - 3 LSB pins

  Control in (Ci):
    80: Standard input request:
      (LIBUSB_REQUEST_TYPE_STANDARD (0x00 ) | LIBUSB_ENDPOINT_IN (0x80)):

      80 06 0100 0000 0012 18    - USB: Get device descriptor
      80 06 0200 0000 0020 32    - USB: Get config descriptor
      80 06 0300 0000 00ff 255   - USB: Get string descriptor
      80 06 0302 0409 00ff 255   - USB: Get string descriptor (2?)
      80 06 0303 0409 00ff 255   - USB: Get string descriptor (3?)

    C0: Vendor input request
      (LIBUSB_REQUEST_TYPE_VENDOR (0x40) | LIBUSB_ENDPOINT_IN (0x80)):

      c0 05 0000 0000 0002 2    - Request modem status
      c0 0c 0000 0000 0001 1    - Read pins
      c0 90 0000 0001 0002 2    - 0x90: Read EEPROM at offset 1 (2 Bytes)
                                    VENDOR_ID          = 0
                                    PRODUCT_ID         = 1
                                    SELF_POWERED       = 2
                                    REMOTE_WAKEUP      = 3
                                    IS_NOT_PNP         = 4
                                    SUSPEND_DBUS7      = 5
                                    IN_IS_ISOCHRONOUS  = 6
                                    OUT_IS_ISOCHRONOUS = 7
                                    SUSPEND_PULL_DOWNS = 8
                                    USE_SERIAL         = 9
                                    USB_VERSION        = 10
                                    USE_USB_VERSION    = 11
                                    MAX_POWER          = 12
                                    CHANNEL_A_TYPE     = 13
                                    CHANNEL_B_TYPE     = 14
                                    CHANNEL_A_DRIVER   = 15
                                    CHANNEL_B_DRIVER   = 16
                                    CBUS_FUNCTION_0    = 17
                                    CBUS_FUNCTION_1    = 18
                                    CBUS_FUNCTION_2    = 19
                                    CBUS_FUNCTION_3    = 20
                                    CBUS_FUNCTION_4    = 21
                                    CBUS_FUNCTION_5    = 22
                                    CBUS_FUNCTION_6    = 23
                                    CBUS_FUNCTION_7    = 24
                                    CBUS_FUNCTION_8    = 25
                                    CBUS_FUNCTION_9    = 26
                                    HIGH_CURRENT       = 27
                                    HIGH_CURRENT_A     = 28
                                    HIGH_CURRENT_B     = 29
                                    INVERT             = 30
                                    GROUP0_DRIVE       = 31
                                    GROUP0_SCHMITT     = 32
                                    GROUP0_SLEW        = 33
                                    GROUP1_DRIVE       = 34
                                    GROUP1_SCHMITT     = 35
                                    GROUP1_SLEW        = 36
                                    GROUP2_DRIVE       = 37
                                    GROUP2_SCHMITT     = 38
                                    GROUP2_SLEW        = 39
                                    GROUP3_DRIVE       = 40
                                    GROUP3_SCHMITT     = 41
                                    GROUP3_SLEW        = 42
                                    CHIP_SIZE          = 43
                                    CHIP_TYPE          = 44
                                    POWER_SAVE         = 45
                                    CLOCK_POLARITY     = 46
                                    DATA_ORDER         = 47
                                    FLOW_CONTROL       = 48
                                    CHANNEL_C_DRIVER   = 49
                                    CHANNEL_D_DRIVER   = 50
                                    CHANNEL_A_RS485    = 51
                                    CHANNEL_B_RS485    = 52
                                    CHANNEL_C_RS485    = 53
                                    CHANNEL_D_RS485    = 54
                                    RELEASE_NUMBER     = 55
 */

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <libusb-1.0/libusb.h>

static struct libusb_device_handle *dh;
static uint8_t *data;

#define IMAGEFILE "image.img"

#define LOGICPORT_STARTUP             0

#define LOGICPORT_BITMODE_RESET       0
#define LOGICPORT_BITMODE_BITBANG    (1 << 8)
#define LOGICPORT_BITMODE_3_LSB_PINS  7

#define LOGICPORT_SILENT              0
#define LOGICPORT_PRINT               1

#define LOGICPORT_MODEM_STATUS        0
#define LOGICPORT_READ_PINS           1
#define LOGICPORT_READ_EPROM          2
#define LOGICPORT_RESET_SIO           3
#define LOGICPORT_PURGE_RX_BUFFER     4
#define LOGICPORT_PURGE_TX_BUFFER     5
#define LOGICPORT_SET_BAUDRATE        6
#define LOGICPORT_SET_LATENCY_TIMER   7
#define LOGICPORT_SET_BIT_MODE        8

#define LOGICPORT_ENDPOINT_ONE        1
#define LOGICPORT_ENDPOINT_TWO        2

#define LOGICPORT_MSG_OUT             0
#define LOGICPORT_MSG_IN              1

#define LOGICPORT_BITBANG_3_LSB       LOGICPORT_BITMODE_BITBANG | LOGICPORT_BITMODE_3_LSB_PINS
#define LOGICPORT_VENDOR_IN           LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN
#define LOGICPORT_VENDOR_OUT          LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT
#define LOGICPORT_EP_OUT_TWO          LIBUSB_ENDPOINT_OUT | LOGICPORT_ENDPOINT_TWO
#define LOGICPORT_EP_IN_ONE           LIBUSB_ENDPOINT_IN | LOGICPORT_ENDPOINT_ONE

#define LOGICPORT_VID                 0x0403
#define LOGICPORT_PID                 0xdc48

#define LOGICPORT_MAX_DATA_SIZE       4096

#define LOGICPORT_NA 0

static void logicport_print(uint8_t *msg, int len, int direction)
{
	int i;

	printf("%s ", direction ? "IN" : "OUT");
	for (i = 0; i < len; i++)
		printf("%02x ", msg[i]);
	printf("\n");
}

static uint8_t * logicport_strip_ftdi(int *len)
{
	uint8_t *msg;
#if 0
	if (data[0] == 0x31 && data[1] == 0x60) {
		*len -= 2;
		return data + 2;
	}

	printf("Unexpected FTDI status 0x%x02\n", data[0], data[1]);
#endif
	return data;
}

static int logicport_react(int len, int state)
{
	int ret;
	static int e0 = 0;

	switch(state) {
	case LOGICPORT_STARTUP :
		/* TODO */
		break;

	default :
		printf("Unsupported state: %d\n", ret);
	}
	return 0;
}

static int logicport_parse(int len, int print)
{
	uint8_t *msg;
	static int idle = 0, once = 1, packets = 0;
	int ret, i;

	msg = logicport_strip_ftdi(&len);

	if (len) {
		if (len > 2) {
			if (print)
				logicport_print(msg, len, LOGICPORT_MSG_IN);

			ret = logicport_react(len, LOGICPORT_STARTUP);
			if (ret < 0)
				return -1;
		}
		packets++;

		if (len > 2)
			idle = 0;
	}

	if (idle++ > 64 || packets > 512) {
		printf("Nothing interesting happening - exiting\n");
		return -1;
	}

	return 0;
}

static int logicport_read(int print)
{
	int delivered;
	int ret;

	for (;;) {
		ret = libusb_bulk_transfer(dh, LOGICPORT_EP_IN_ONE,
					   data, LOGICPORT_MAX_DATA_SIZE,
					   &delivered, 10);
		if (ret == LIBUSB_ERROR_TIMEOUT)
			continue;

		if (ret)
			printf("Read failed: Returned: %d errno: %d\n", ret, errno);
		else if (delivered) {
			ret = logicport_parse(delivered, print);
			if (ret < 0)
				return -1;
		}
		break;
	}

	return delivered;
}

static int logicport_write(uint8_t *msg, int len, int print)
{
	int delivered, timeout = 10;
	int ret, i;

	for (i = 0; i < timeout; i++) {
		ret = libusb_bulk_transfer(dh, LOGICPORT_EP_OUT_TWO,
					   msg, len, &delivered, 10);
		if (ret == LIBUSB_ERROR_TIMEOUT)
				continue;

		if (ret) {
			printf("Write failed: Returned: %d errno: %d\n", ret, errno);
			return -1;
		}

		if (len != delivered)
			printf("Not all delivered: (delivered: %d from %d)\n", delivered, len);

		if (print)
			logicport_print(msg, delivered, LOGICPORT_MSG_OUT);

		return delivered;
	}

	printf("Failed to send packet - timed out\n");
	return -1;
}

static int logicport_flash_fpga(void)
{
	int ret, i;
	int alt, len;
	FILE *fp;

	fp = fopen(IMAGEFILE, "rb");
	if (!fp) {
		printf("Failed to open file %s\n", IMAGEFILE);
		return -1;
	}

	do {
		len = fread(data, 1, LOGICPORT_MAX_DATA_SIZE, fp);
		if (ferror(fp)) {
			printf("Failed to read from %s\n", IMAGEFILE);
			ret = -1;
			goto err;
		}

		if (len) {
			ret = logicport_write(data, len, LOGICPORT_SILENT);
			if (ret < 0)
				goto err;
		}
		usleep(500);

	} while(!feof(fp));
	ret = 0;

err:
        fclose(fp);
	return ret;
}

static int __logicport_control(uint8_t bmRequestType, uint8_t bRequest,
			       uint16_t wValue, uint16_t wIndex, int len)
{
	int ret;

	for (;;) {
		ret = libusb_control_transfer(dh, bmRequestType, bRequest,
					      wValue, wIndex, data, len, 10);
		if (ret == LIBUSB_ERROR_TIMEOUT)
			continue;
		if (ret < 0) {
			printf("Control failed: Returned: %d errno: %d\n", ret, errno);
			return ret;
		}
		break;
	}

	return len;
}

static int logicport_control(int msg, int arg)
{
	switch(msg) {
	case LOGICPORT_MODEM_STATUS :
		return __logicport_control(LOGICPORT_VENDOR_IN,  0x05, 0x0000, 0x0000, 2);
	case LOGICPORT_READ_PINS :
		return __logicport_control(LOGICPORT_VENDOR_IN,  0x0C, 0x0000, 0x0000, 1);
	case LOGICPORT_READ_EPROM :
		return __logicport_control(LOGICPORT_VENDOR_IN,  0x90, 0x0000, arg,    2);
	case LOGICPORT_RESET_SIO :
		return __logicport_control(LOGICPORT_VENDOR_OUT, 0x00, 0x0000, 0x0000, 0);
	case LOGICPORT_PURGE_RX_BUFFER :
		return __logicport_control(LOGICPORT_VENDOR_OUT, 0x00, 0x0001, 0x0000, 0);
	case LOGICPORT_PURGE_TX_BUFFER :
		return __logicport_control(LOGICPORT_VENDOR_OUT, 0x00, 0x0002, 0x0000, 0);
	case LOGICPORT_SET_BAUDRATE :
		return __logicport_control(LOGICPORT_VENDOR_OUT, 0x03, arg,    0x0000, 0);
	case LOGICPORT_SET_LATENCY_TIMER :
		return __logicport_control(LOGICPORT_VENDOR_OUT, 0x09, arg,    0x0000, 0);
	case LOGICPORT_SET_BIT_MODE :
		return __logicport_control(LOGICPORT_VENDOR_OUT, 0x0B, arg,    0x0000, 0);
	default :
		printf("Internal error: Unsupported control message: %d\n", msg);
	}
	return -1;
}

static int logicport_purge_buffers(void)
{
	int ret, i;

	ret = logicport_control(LOGICPORT_PURGE_TX_BUFFER, LOGICPORT_NA);
	if (ret < 0)
		return -1;

	for (i = 0; i < 6; i++) {
		ret = logicport_control(LOGICPORT_PURGE_RX_BUFFER, LOGICPORT_NA);
		if (ret < 0)
			return -1;
	}

	return 0;
}

static int logicport_reset_settings(void)
{
	int ret;

	ret = logicport_control(LOGICPORT_RESET_SIO, LOGICPORT_NA);
	if (ret < 0)
		return -1;

	ret = logicport_control(LOGICPORT_MODEM_STATUS, LOGICPORT_NA);
	if (ret < 0)
		return -1;

	ret = logicport_control(LOGICPORT_RESET_SIO, LOGICPORT_NA);
	if (ret < 0)
		return -1;

	logicport_purge_buffers();

	ret = logicport_control(LOGICPORT_SET_LATENCY_TIMER, 4);
	if (ret < 0)
		return -1;

	ret = logicport_control(LOGICPORT_SET_BAUDRATE, 0x4006);
	if (ret < 0)
		return -1;

	return 0;
}

static int logicport_startup(void)
{
	int ret, i, j, k;
	int len, alt = 0;
	int count = 0;

	ret = logicport_reset_settings();
	if (ret < 0)
		return -1;

	//logicport_purge_buffers();
	//if (ret < 0)
	//	return -1;

	ret = logicport_control(LOGICPORT_SET_BIT_MODE,
				LOGICPORT_BITBANG_3_LSB);
	if (ret < 0)
		return -1;

	//data[0] = 0x00;
	//ret = logicport_write(data, 1, LOGICPORT_SILENT);
	//if (ret < 0)
	//	return -1;

	//data[0] = 0x04;
	//ret = logicport_write(data, 1, LOGICPORT_SILENT);
	//if (ret < 0)
	//	return -1;

	ret = logicport_flash_fpga();
	if (ret < 0)
		return -1;

	ret = logicport_control(LOGICPORT_READ_PINS, LOGICPORT_NA);
	if (ret < 0)
		return -1;
	if (data[0] != 0xff)
		printf("Flash probably not successful:\n"
		       "  pins: 0x%x (expected: 0xff)\n", data[0]);

	ret = logicport_control(LOGICPORT_SET_BIT_MODE, LOGICPORT_BITMODE_RESET);
	if (ret < 0)
		return -1;

	//ret = logicport_reset_settings();
	//if (ret < 0)
	//	return -1;

	logicport_purge_buffers();
	if (ret < 0)
		return -1;

	/* Flush the buffers. */
	memset(data, 0, LOGICPORT_MAX_DATA_SIZE);
	for (i = 0; i < 32; i++) {
		ret = logicport_write(data, 32, LOGICPORT_SILENT);
		if (ret < 0)
			return -1;

		len = logicport_read(LOGICPORT_SILENT);
		if (len < 0)
			return -1;

		if (len >= 2 && data[0] == 0x31 && data[1] == 0x60)
			break;
	}

	data[0] = 0x01;
	ret = logicport_write(data, 1, LOGICPORT_SILENT);
	if (ret < 0)
		return -1;

	//ret = logicport_control(LOGICPORT_READ_EPROM, 0x3f);
	//if (ret < 0)
	//	return -1;

	/* Write first 'C' message: C3 00 20 000 (Request packet number) */
	data[0] = 0xC3; data[1] = 0x00; data[2] = 0x20;
	data[3] = 0x00; data[4] = 0x00;
	ret = logicport_write(data, 5, LOGICPORT_PRINT);
	if (ret < 0)
		return -1;

	return 0;
}

static uint8_t * logicport_init(void)
{
	int ret;

	ret = libusb_init(NULL);
	if (ret < 0) {
		fprintf(stderr, "failed to init libusb\n");
		exit(-1);
	}

	dh = libusb_open_device_with_vid_pid(NULL, LOGICPORT_VID, LOGICPORT_PID);
	if (!dh) {
		fprintf(stderr, "can't open %x %x\n", LOGICPORT_VID, LOGICPORT_PID);
		return NULL;
	}

	ret = libusb_claim_interface(dh, 0);
	if (ret < 0) {
		fprintf(stderr, "can't claim interface 0\n");
		return NULL;
	}

	return malloc(LOGICPORT_MAX_DATA_SIZE);
}

int main(int argc, char *argv[]) {
	int r, ret, delivered, i, timeout = 0;
	static int once = 1;

	data = logicport_init();
	if (!data)
		goto done;

	ret = logicport_startup();
	if (ret)
		goto release;

	for (;;) {
		ret = logicport_read(LOGICPORT_PRINT);
		if (ret < 0)
			goto release;
	}

	ret = 0;

release:
	free(data);
	libusb_release_interface(dh, 0);
done:
	if (dh)
		libusb_close(dh);
	libusb_exit(NULL);

	return ret;
}
