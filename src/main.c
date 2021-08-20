#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/adc.h>
#define CLK GPIO0
#define POSTBIT GPIO1
#define RST_IN GPIO2
#define RST_IN_ADC_CHAN 2
#define RST_OUT GPIO3
#define PLL GPIO4
#define RST_HIGH_TSHD 1500

bool post_high_state = false;
bool clk_rising_edge = false;
char rst_state = '0', old_rst_state = 'X';
char pll_state = '0', old_pll_state = 'X';
uint32_t edges_cnt = 0;
uint16_t post_cnt = 0;
bool force_rst = true;
volatile bool serial_tx_complete = true;
bool started = false;
char cmd_buffer[64] = { 0 };
usbd_device * usbd_dev;

static
const struct usb_device_descriptor dev = { .bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 *This notification endpoint isn't implemented. According to CDC spec its
 *optional, but its absence causes a NULL pointer dereference in Linux
 *cdc_acm driver.
 */
static
const struct usb_endpoint_descriptor comm_endp[] = {
		{
		.bLength = USB_DT_ENDPOINT_SIZE,
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 0x83,
			.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
			.wMaxPacketSize = 16,
			.bInterval = 255,
	}
};

static
const struct usb_endpoint_descriptor data_endp[] = {
		{
		.bLength = USB_DT_ENDPOINT_SIZE,
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 0x01,
			.bmAttributes = USB_ENDPOINT_ATTR_BULK,
			.wMaxPacketSize = 64,
			.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x82,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1,
	}
};

static
const struct
{
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
}
__attribute__((packed)) cdcacm_functional_descriptors = { .header = { .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = { .bFunctionLength =
		sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = { .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = { .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	}
};

static
const struct usb_interface_descriptor comm_iface[] = {
		{
		.bLength = USB_DT_INTERFACE_SIZE,
			.bDescriptorType = USB_DT_INTERFACE,
			.bInterfaceNumber = 0,
			.bAlternateSetting = 0,
			.bNumEndpoints = 1,
			.bInterfaceClass = USB_CLASS_CDC,
			.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
			.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
			.iInterface = 0,
			.endpoint = comm_endp,
			.extra = &cdcacm_functional_descriptors,
			.extralen = sizeof(cdcacm_functional_descriptors)
	}
};

static
const struct usb_interface_descriptor data_iface[] = {
		{
		.bLength = USB_DT_INTERFACE_SIZE,
			.bDescriptorType = USB_DT_INTERFACE,
			.bInterfaceNumber = 1,
			.bAlternateSetting = 0,
			.bNumEndpoints = 2,
			.bInterfaceClass = USB_CLASS_DATA,
			.bInterfaceSubClass = 0,
			.bInterfaceProtocol = 0,
			.iInterface = 0,
			.endpoint = data_endp,
	}
};

static
const struct usb_interface ifaces[] = {
		{
		.num_altsetting = 1,
			.altsetting = comm_iface,
	},
	{
		.num_altsetting = 1,
		.altsetting = data_iface,
	}
};

static
const struct usb_config_descriptor config = { .bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,
	.interface = ifaces,
};

static
const char *usb_strings[] = { "DrSchottky",
	"X360 Timing Pirate",
	"DEMO",
};
/*Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req,
	uint8_t **buf,
	uint16_t *len,
	void(**complete)(usbd_device *usbd_dev,
		struct usb_setup_data *req))
{
	(void) complete;
	(void) buf;
	(void) usbd_dev;

	switch (req->bRequest)
	{
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
			{ 	/*
				 *This Linux cdc_acm driver requires this to be implemented
				 *even though it's optional in the CDC spec, and we don't
				 *advertise it in the ACM functional descriptor.
				 */
				char local_buf[10];
				struct usb_cdc_notification *notif = (void*) local_buf;

				/*We echo signals back to host as notification. */
				notif->bmRequestType = 0xA1;
				notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
				notif->wValue = 0;
				notif->wIndex = 0;
				notif->wLength = 2;
				local_buf[8] = req->wValue &3;
				local_buf[9] = 0;
				return USBD_REQ_HANDLED;
			}
		case USB_CDC_REQ_SET_LINE_CODING:
			if (*len < sizeof(struct usb_cdc_line_coding))
			{
				return USBD_REQ_NOTSUPP;
			}
			return USBD_REQ_HANDLED;
	}
	return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void) ep;
	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	if (len)
	{
		if (!cmd_buffer[0])
		{
			strncpy(cmd_buffer, buf, 64);
			cmd_buffer[len - 1] = 0;
		}
	}
}

static void cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	serial_tx_complete = true;
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void) wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_tx_cb);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(usbd_dev,
		USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		cdcacm_control_request);
}

static void configure_gpio(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, CLK | RST_OUT | POSTBIT);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_ANALOG, RST_IN);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_PULL_UPDOWN, PLL);
	gpio_clear(GPIOA, PLL);
	gpio_clear(GPIOA, POSTBIT);
	gpio_clear(GPIOA, RST_OUT);
	for (int i = 0; i < 10; i++)
	{
		gpio_set(GPIOA, CLK);
		gpio_clear(GPIOA, CLK);
	}

	rcc_periph_clock_enable(RCC_ADC1);
	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
	adc_power_on(ADC1);
	for (int i = 0; i < 800000; i++)
		__asm__("nop");
	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

static uint16_t read_adc(uint8_t channel)
{
	uint8_t channel_array[16];
	channel_array[0] = channel;
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_start_conversion_direct(ADC1);
	while (!adc_eoc(ADC1));
	uint16_t reg16 = adc_read_regular(ADC1);
	return reg16;
}

static void blocking_serial_print(char *buffer, int len)
{

	int total_tx_sent = 0;
	while (len)
	{
		int timeout = 100;
		serial_tx_complete = false;
		int tx_size = (len < 64) ? len : 64;
		usbd_ep_write_packet(usbd_dev, 0x82, buffer + total_tx_sent, tx_size);
		total_tx_sent += tx_size;
		len -= tx_size;
		while (!serial_tx_complete && timeout-- > 0)
		{
			usbd_poll(usbd_dev);
		};
	}
}

static void serial_print(const char *buffer)
{
	blocking_serial_print(buffer, strlen(buffer));
}

static char get_pll_state()
{
	if (gpio_get(GPIOA, PLL))
	{
		return '1';
	}
	else
	{
		return '0';
	}
}

static char get_rst_state()
{
	if (read_adc(RST_IN_ADC_CHAN) > RST_HIGH_TSHD)
	{
		return '1';
	}
	else
	{
		gpio_set(GPIOA, RST_OUT);
		if (read_adc(RST_IN_ADC_CHAN) > RST_HIGH_TSHD)
		{
			return 'Z';
		}
		else
		{
			return '0';
		}
	}
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config,
		usb_strings,
		3,
		usbd_control_buffer,
		sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
	configure_gpio();

	pll_state = get_pll_state();
	old_pll_state = pll_state;
	rst_state = get_rst_state();
	old_rst_state = rst_state;
	while (1)
	{
		usbd_poll(usbd_dev);
		gpio_clear(GPIOA, RST_OUT);
		if (!force_rst)
		{
			pll_state = get_pll_state();
			rst_state = get_rst_state();
			if (old_rst_state != rst_state || old_pll_state != pll_state)
			{
				char msg[256] = {};
				sprintf(msg, "\nDetected event on CLK edge %lu (%c)\n", edges_cnt, (clk_rising_edge) ? 'R' : 'F');
				serial_print(msg);
				if (old_rst_state != rst_state)
				{
					sprintf(msg, "CPU_RST state changed: %c->%c\n", old_rst_state, rst_state);
					serial_print(msg);
					old_rst_state = rst_state;
				}
				if (old_pll_state != pll_state)
				{
					sprintf(msg, "PLL_BYPASS state changed: %c->%c\n", old_pll_state, pll_state);
					serial_print(msg);
					old_pll_state = pll_state;
				}
			}
			gpio_set(GPIOA, RST_OUT);
		}
		for (unsigned int i = 0; i < strlen(cmd_buffer); i++)
		{
			char cmd = cmd_buffer[i];
			if (cmd == 'p' || cmd == 'P')
			{
				post_cnt++;
				post_high_state = !post_high_state;
				char msg[64] = { 0 };
				sprintf(msg, "\nSend POST tick %d", post_cnt);
				serial_print(msg);
				if (post_high_state)
				{
					gpio_set(GPIOA, POSTBIT);
				}
				else
				{
					gpio_clear(GPIOA, POSTBIT);
				}
				edges_cnt = 0;
			}
			else if (cmd == 'r')
			{
				serial_print("\nReleasing CPU_RST");
				force_rst = false;
				post_cnt = 0;
				post_high_state = false;
				gpio_clear(GPIOA, POSTBIT);
			}
			else if (cmd == 'R')
			{
				serial_print("\nForcing CPU_RST");
				force_rst = true;
			}
			if (i == strlen(cmd_buffer) - 1)
				cmd_buffer[0] = 0;
		}
		if (clk_rising_edge)
		{
			gpio_clear(GPIOA, CLK);
		}
		else
		{
			gpio_set(GPIOA, CLK);
		}
		clk_rising_edge = !clk_rising_edge;
		edges_cnt++;
	}
}