#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include <pmd.h>
#include <usb-20X.h>

volatile sig_atomic_t interrupted = 0;

void handle_interrupt(int sig)
{
	(void) sig;  // Clear unused variable warning

	interrupted = 1;

	// A second interrupt will kill the program normally
	signal(SIGINT, SIG_DFL);
}

uint32_t npot(uint32_t v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;

}

int main()
{
	libusb_device_handle *udev = NULL;
	float table_AIN[NCHAN_USB20X][2];
	struct tm calDate;
	uint8_t options = 0x0;
	uint8_t channel = 0;
	double frequency;
	// Analog input data
	uint16_t *sdataIn;

	if (libusb_init(NULL) < 0) {
		perror("Failed to initialize libusb");
		exit(1);
	}

	if ((udev = usb_device_find_USB_MCC(USB201_PID, NULL))) {
		fprintf(stderr, "Success, found a USB 201!\n");
	} else {
		fprintf(stderr, "Failure, did not find a USB 201!\n");
		return 0;
	}

	// Print out the wMaxPacketSize. Should be 64
	fprintf(stderr, "wMaxPacketSize = %d\n", usb_get_max_packet_size(udev, 0));

	usbBuildGainTable_USB20X(udev, table_AIN);
	for (int i = 0; i < NCHAN_USB20X; i++) {
		fprintf(stderr, "Calibration Table: %d	 Slope = %f	 Offset = %f\n", i, table_AIN[i][0], table_AIN[i][1]);
	}

	usbCalDate_USB20X(udev, &calDate);
	fprintf(stderr, "MFG Calibration date = %s\n", asctime(&calDate));

	fprintf(stderr, "USB-201 Analog Input Scan in Continuous mode\n");
	fprintf(stderr, "Enter number of channels [1-8]: ");
	int nchan;
	scanf("%d", &nchan);
	fprintf(stderr, "Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
	fprintf(stderr, "Ctrl-C to exit\n");

	if (frequency < 100.) {
		options = IMMEDIATE_TRANSFER_MODE;
	}
	for (int i = 0; i < nchan; i++) {
		channel |= (0x1 << i);
	}
	usbAInScanStop_USB20X(udev);
	usbAInScanStop_USB20X(udev);
	usbAInScanClearFIFO_USB20X(udev);

	uint16_t count = npot(frequency / 4);
	if (count < 1) {
		count = 1;
	}

	// The total number of bytes returned is 2*nchan*count
	if ((sdataIn = malloc(sizeof(uint16_t) * nchan * count)) == NULL) {
		perror("Error in malloc");
		return 1;
	}
	sleep(1);

	usbAInScanStart_USB20X(udev, 0, frequency, channel, options, 0, 0);

	signal(SIGINT, handle_interrupt);

	time_t last_update = time(NULL);
	uint32_t scan_number = 0;
	do {
		int ret = usbAInScanRead_USB20X(udev, count, nchan, sdataIn, options | CONTINUOUS, 5000);
		if (ret < 0) {
			fprintf(stderr, "aInScanRead failed\n");
			goto cleanup;
		}
		// For each channel in each scan
		for (uint16_t scan = 0; scan < count; scan++) {
			for (int chan = 0; chan < nchan; chan++) {
				uint16_t corrected = sdataIn[scan * nchan + chan] * table_AIN[chan][0] + table_AIN[chan][1] + 0.5;
				printf("%lf", volts_USB20X(corrected) * 1000);
				putchar((chan < nchan - 1) ? ',' : '\n');
			}
		}
		scan_number += count;
		time_t now = time(NULL);
		if (now - last_update > 1) {
			fprintf(stderr, "Scan %d\n", scan_number * count);
		}
		last_update = now;
	} while (!interrupted);

cleanup:
	free(sdataIn);

	usbAInScanStop_USB20X(udev);
}

