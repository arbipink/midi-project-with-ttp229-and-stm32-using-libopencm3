#include <stdlib.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* ============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================ */

// TTP229 Hardware Configuration
#define TTP229_SCL_PORT         GPIOA
#define TTP229_SCL_PIN          GPIO6
#define TTP229_SDO_PORT         GPIOA  
#define TTP229_SDO_PIN          GPIO7
#define TTP229_NUM_KEYS         16

// Timing Configuration
#define TTP229_CLOCK_DELAY_US   10
#define TTP229_READ_INTERVAL_MS 5
#define TTP229_DEBOUNCE_COUNT   3
#define MAIN_LOOP_DELAY_US      1000

// MIDI Configuration
#define MIDI_BASE_NOTE          60      // C4 (Middle C)
#define MIDI_VELOCITY_ON        80      // Note on velocity
#define MIDI_VELOCITY_OFF       0       // Note off velocity
#define MIDI_CHANNEL            0       // MIDI Channel 1 (0-indexed)

// USB Configuration
#define USB_VENDOR_ID           0x6666  // Prototype vendor ID
#define USB_PRODUCT_ID          0x5119  // Random product ID
#define USB_CONTROL_BUFFER_SIZE 128

/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================ */

typedef struct {
    uint16_t current_state;
    uint16_t previous_state;
    uint8_t debounce_counters[TTP229_NUM_KEYS];
    uint32_t last_read_time;
} ttp229_state_t;

typedef struct {
    uint32_t ms_counter;
} system_time_t;

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */

static ttp229_state_t ttp229_state = {0};
static system_time_t sys_time = {0};
static char usb_serial_number[25];
static uint8_t usbd_control_buffer[USB_CONTROL_BUFFER_SIZE];

/* ============================================================================
 * USB DESCRIPTORS
 * ============================================================================ */

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = USB_VENDOR_ID,
    .idProduct = USB_PRODUCT_ID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_midi_endpoint_descriptor midi_bulk_endp[] = {{
    .head = {
        .bLength = sizeof(struct usb_midi_endpoint_descriptor),
        .bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
        .bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
        .bNumEmbMIDIJack = 1,
    },
    .jack[0] = {
        .baAssocJackID = 0x01,
    },
}, {
    .head = {
        .bLength = sizeof(struct usb_midi_endpoint_descriptor),
        .bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
        .bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
        .bNumEmbMIDIJack = 1,
    },
    .jack[0] = {
        .baAssocJackID = 0x03,
    },
}};

static const struct usb_endpoint_descriptor bulk_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 0x40,
    .bInterval = 0x00,
    .extra = &midi_bulk_endp[0],
    .extralen = sizeof(midi_bulk_endp[0])
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x81,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 0x40,
    .bInterval = 0x00,
    .extra = &midi_bulk_endp[1],
    .extralen = sizeof(midi_bulk_endp[1])
}};

static const struct {
    struct usb_audio_header_descriptor_head header_head;
    struct usb_audio_header_descriptor_body header_body;
} __attribute__((packed)) audio_control_functional_descriptors = {
    .header_head = {
        .bLength = sizeof(struct usb_audio_header_descriptor_head) +
                   1 * sizeof(struct usb_audio_header_descriptor_body),
        .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
        .bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
        .bcdADC = 0x0100,
        .wTotalLength = sizeof(struct usb_audio_header_descriptor_head) +
                       1 * sizeof(struct usb_audio_header_descriptor_body),
        .binCollection = 1,
    },
    .header_body = {
        .baInterfaceNr = 0x01,
    },
};

static const struct usb_interface_descriptor audio_control_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 0,
    .bInterfaceClass = USB_CLASS_AUDIO,
    .bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .extra = &audio_control_functional_descriptors,
    .extralen = sizeof(audio_control_functional_descriptors)
}};

static const struct {
    struct usb_midi_header_descriptor header;
    struct usb_midi_in_jack_descriptor in_embedded;
    struct usb_midi_in_jack_descriptor in_external;
    struct usb_midi_out_jack_descriptor out_embedded;
    struct usb_midi_out_jack_descriptor out_external;
} __attribute__((packed)) midi_streaming_functional_descriptors = {
    .header = {
        .bLength = sizeof(struct usb_midi_header_descriptor),
        .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
        .bDescriptorSubtype = USB_MIDI_SUBTYPE_MS_HEADER,
        .bcdMSC = 0x0100,
        .wTotalLength = sizeof(midi_streaming_functional_descriptors),
    },
    .in_embedded = {
        .bLength = sizeof(struct usb_midi_in_jack_descriptor),
        .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
        .bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
        .bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
        .bJackID = 0x01,
        .iJack = 0x00,
    },
    .in_external = {
        .bLength = sizeof(struct usb_midi_in_jack_descriptor),
        .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
        .bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
        .bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
        .bJackID = 0x02,
        .iJack = 0x00,
    },
    .out_embedded = {
        .head = {
            .bLength = sizeof(struct usb_midi_out_jack_descriptor),
            .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
            .bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
            .bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
            .bJackID = 0x03,
            .bNrInputPins = 1,
        },
        .source[0] = {
            .baSourceID = 0x02,
            .baSourcePin = 0x01,
        },
        .tail = {
            .iJack = 0x00,
        }
    },
    .out_external = {
        .head = {
            .bLength = sizeof(struct usb_midi_out_jack_descriptor),
            .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
            .bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
            .bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
            .bJackID = 0x04,
            .bNrInputPins = 1,
        },
        .source[0] = {
            .baSourceID = 0x01,
            .baSourcePin = 0x01,
        },
        .tail = {
            .iJack = 0x00,
        },
    },
};

static const struct usb_interface_descriptor midi_streaming_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_AUDIO,
    .bInterfaceSubClass = USB_AUDIO_SUBCLASS_MIDISTREAMING,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = bulk_endp,
    .extra = &midi_streaming_functional_descriptors,
    .extralen = sizeof(midi_streaming_functional_descriptors)
}};

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = audio_control_iface,
}, {
    .num_altsetting = 1,
    .altsetting = midi_streaming_iface,
}};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,
    .interface = ifaces,
};

static const char *usb_strings[] = {
    "Arbipink",
    "Polyphonic MIDI Touch Controller",
    usb_serial_number
};

/* SysEx identity response */
const uint8_t sysex_identity[] = {
    0x04, 0xf0, 0x7e, 0x00,    // SysEx start, non-realtime, channel 0
    0x04, 0x7d, 0x66, 0x66,    // Manufacturer ID, family code
    0x04, 0x51, 0x19, 0x00,    // Model number, version start
    0x04, 0x00, 0x01, 0x00,    // Version continuation
    0x05, 0xf7, 0x00, 0x00,    // SysEx end with padding
};

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

static inline void delay_us(uint32_t us) {
    // Assuming 72MHz clock, each loop iteration takes ~4 cycles
    volatile uint32_t cycles = (us * 72) / 4;
    while (cycles--) {
        __asm__("nop");
    }
}

static inline bool is_key_pressed(uint16_t key_state, uint8_t key_index) {
    return (key_state & (1 << key_index)) != 0;
}

static inline uint8_t get_midi_note_for_key(uint8_t key_index) {
    return MIDI_BASE_NOTE + key_index;
}

/* ============================================================================
 * TTP229 TOUCH SENSOR FUNCTIONS
 * ============================================================================ */

static void ttp229_init_hardware(void) {
    gpio_set_mode(TTP229_SCL_PORT, GPIO_MODE_OUTPUT_2_MHZ, 
                  GPIO_CNF_OUTPUT_PUSHPULL, TTP229_SCL_PIN);
    
    gpio_set_mode(TTP229_SDO_PORT, GPIO_MODE_INPUT, 
                  GPIO_CNF_INPUT_PULL_UPDOWN, TTP229_SDO_PIN);
    gpio_set(TTP229_SDO_PORT, TTP229_SDO_PIN);
    
    gpio_set(TTP229_SCL_PORT, TTP229_SCL_PIN);
}

static uint16_t ttp229_read_raw_keys(void) {
    uint16_t keys = 0;
    
    delay_us(TTP229_CLOCK_DELAY_US * 2);
    for (int i = 0; i < TTP229_NUM_KEYS; i++) {
        gpio_clear(TTP229_SCL_PORT, TTP229_SCL_PIN);
        delay_us(TTP229_CLOCK_DELAY_US);
        
        gpio_set(TTP229_SCL_PORT, TTP229_SCL_PIN);
        delay_us(TTP229_CLOCK_DELAY_US);
        
        if (!gpio_get(TTP229_SDO_PORT, TTP229_SDO_PIN)) {
            keys |= (1 << i);
        }
    }
    
    delay_us(TTP229_CLOCK_DELAY_US * 2);
    
    return keys;
}

static uint16_t ttp229_debounce_keys(uint16_t raw_keys) {
    uint16_t stable_keys = 0;
    
    for (int i = 0; i < TTP229_NUM_KEYS; i++) {
        bool raw_pressed = is_key_pressed(raw_keys, i);
        bool current_pressed = is_key_pressed(ttp229_state.current_state, i);
        
        if (raw_pressed == current_pressed) {
            ttp229_state.debounce_counters[i] = 0;
            if (raw_pressed) {
                stable_keys |= (1 << i);
            }
        } else {
            ttp229_state.debounce_counters[i]++;
            
            if (ttp229_state.debounce_counters[i] >= TTP229_DEBOUNCE_COUNT) {
                ttp229_state.debounce_counters[i] = 0;
                if (raw_pressed) {
                    stable_keys |= (1 << i);
                }
            } else {
                if (current_pressed) {
                    stable_keys |= (1 << i);
                }
            }
        }
    }
    
    return stable_keys;
}

/* ============================================================================
 * MIDI MESSAGE FUNCTIONS
 * ============================================================================ */

static bool send_midi_note_message(usbd_device *usbd_dev, uint8_t note, 
                                  uint8_t velocity) {
    uint8_t midi_msg[4] = {
        0x08,                                           // USB framing
        velocity > 0 ? (0x90 | MIDI_CHANNEL) : (0x80 | MIDI_CHANNEL), // Note on/off
        note,                                           // Note number
        velocity                                        // Velocity
    };
    
    return usbd_ep_write_packet(usbd_dev, 0x81, midi_msg, sizeof(midi_msg)) > 0;
}

static void send_midi_note_on(usbd_device *usbd_dev, uint8_t note) {
    while (!send_midi_note_message(usbd_dev, note, MIDI_VELOCITY_ON)) {
    }
}

static void send_midi_note_off(usbd_device *usbd_dev, uint8_t note) {
    while (!send_midi_note_message(usbd_dev, note, MIDI_VELOCITY_OFF)) {
    }
}

/* ============================================================================
 * POLYPHONIC KEY PROCESSING
 * ============================================================================ */

static void process_key_changes(usbd_device *usbd_dev, uint16_t new_keys) {
    uint16_t changed_keys = new_keys ^ ttp229_state.previous_state;
    
    // Process all changed keys
    for (int i = 0; i < TTP229_NUM_KEYS; i++) {
        if (changed_keys & (1 << i)) {
            uint8_t midi_note = get_midi_note_for_key(i);
            
            if (is_key_pressed(new_keys, i)) {
                send_midi_note_on(usbd_dev, midi_note);
            } else {
                send_midi_note_off(usbd_dev, midi_note);
            }
        }
    }
    
    ttp229_state.previous_state = ttp229_state.current_state;
    ttp229_state.current_state = new_keys;
}

static void ttp229_poll(usbd_device *usbd_dev) {
    // Rate limiting
    if (sys_time.ms_counter - ttp229_state.last_read_time < TTP229_READ_INTERVAL_MS) {
        return;
    }
    
    uint16_t raw_keys = ttp229_read_raw_keys();
    uint16_t stable_keys = ttp229_debounce_keys(raw_keys);
    process_key_changes(usbd_dev, stable_keys);
    
    ttp229_state.last_read_time = sys_time.ms_counter;
}

/* ============================================================================
 * USB CALLBACK FUNCTIONS
 * ============================================================================ */

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
    (void)ep;
    
    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
    
    // Respond to any incoming message with SysEx identity
    if (len) {
        while (usbd_ep_write_packet(usbd_dev, 0x81, sysex_identity,
                                   sizeof(sysex_identity)) == 0) {
            // Retry until successful
        }
    }
}

static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue) {
    (void)wValue;
    
    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, usbmidi_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
}

/* ============================================================================
 * HARDWARE INITIALIZATION
 * ============================================================================ */

static void init_system_clock(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

static void init_gpio_clocks(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USB);
}

static void init_usb_pins(void) {
    // USB pins PA11 (D-) and PA12 (D+)
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11 | GPIO12);
}

static usbd_device *init_usb_device(void) {
    desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));
    
    usbd_device *usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config,
                                     usb_strings, 3, usbd_control_buffer, 
                                     sizeof(usbd_control_buffer));
    
    usbd_register_set_config_callback(usbd_dev, usbmidi_set_config);
    
    return usbd_dev;
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

int main(void) {
    init_system_clock();
    init_gpio_clocks();
    init_usb_pins();
    
    ttp229_init_hardware();
    
    usbd_device *usbd_dev = init_usb_device();
    
	while (1) {
        usbd_poll(usbd_dev);
        ttp229_poll(usbd_dev);
        sys_time.ms_counter++;
        
        // Small delay to prevent excessive CPU usage
        delay_us(MAIN_LOOP_DELAY_US);
    }
    
    return 0;
}