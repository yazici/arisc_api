/**
 * @file    arisc.h
 *
 * @brief   ARISC firmware API header
 *
 * This test program implements an API to ARISC firmware
 */

#ifndef _ARISC_H
#define _ARISC_H

#include <stdint.h>




// public method prototypes

void encoder_pin_setup(uint8_t c, uint8_t phase, uint8_t port, uint8_t pin);
void encoder_setup(uint8_t c, uint8_t using_B, uint8_t using_Z);
void encoder_state_set(uint8_t c, uint8_t state);
void encoder_counts_set(uint8_t c, int32_t counts);
uint8_t encoder_state_get(uint8_t c);
int32_t encoder_counts_get(uint8_t c);

void pulsgen_pin_setup(uint8_t c, uint8_t port, uint8_t pin, uint8_t inverted);
void pulsgen_task_setup(uint32_t c, uint32_t toggles, uint32_t pin_setup_time,
    uint32_t pin_hold_time, uint32_t start_delay);
void pulsgen_task_abort(uint8_t c);
uint8_t pulsgen_task_state(uint8_t c);
uint32_t pulsgen_task_toggles(uint8_t c);

void gpio_pin_setup_for_output(uint32_t port, uint32_t pin);
void gpio_pin_setup_for_input(uint32_t port, uint32_t pin);
uint32_t gpio_pin_get(uint32_t port, uint32_t pin);
void gpio_pin_set(uint32_t port, uint32_t pin);
void gpio_pin_clear(uint32_t port, uint32_t pin);
uint32_t gpio_port_get(uint32_t port);
void gpio_port_set(uint32_t port, uint32_t mask);
void gpio_port_clear(uint32_t port, uint32_t mask);

int8_t msg_read(uint8_t type, uint8_t * msg, uint8_t bswap);
int8_t msg_send(uint8_t type, uint8_t * msg, uint8_t length, uint8_t bswap);

void mem_init(void);
void mem_deinit(void);

int32_t reg_match(const char *source, const char *pattern, uint32_t *match_array, uint32_t array_size);
int32_t parse_and_exec(const char *str);





// public data

#define PHY_MEM_BLOCK_SIZE      4096

#define SRAM_A2_SIZE            (48*1024)
#define SRAM_A2_ADDR            0x00040000 ///< for ARM use 0x00040000
#define ARISC_CONF_SIZE         2048
#define ARISC_CONF_ADDR         (SRAM_A2_ADDR + SRAM_A2_SIZE - ARISC_CONF_SIZE)

#define MSG_BLOCK_SIZE          4096
#define MSG_BLOCK_ADDR          (ARISC_CONF_ADDR - MSG_BLOCK_SIZE)

#define MSG_CPU_BLOCK_SIZE      2048
#define MSG_ARISC_BLOCK_ADDR    (MSG_BLOCK_ADDR + 0)
#define MSG_ARM_BLOCK_ADDR      (MSG_BLOCK_ADDR + MSG_CPU_BLOCK_SIZE)

#define MSG_MAX_CNT             32
#define MSG_MAX_LEN             (MSG_CPU_BLOCK_SIZE / MSG_MAX_CNT)
#define MSG_LEN                 (MSG_MAX_LEN - 4)

#define MSG_RECV_CALLBACK_CNT   32

#pragma pack(push, 1)
struct msg_t
{
    uint8_t length;
    uint8_t type;
    uint8_t locked; // actually not used at this moment
    uint8_t unread;
    uint8_t msg[MSG_LEN];
};
#pragma pack(pop)




#define GPIO_PORTS_CNT          8   ///< number of GPIO ports
#define GPIO_PINS_CNT           32  ///< number of GPIO port pins

/// the GPIO port names
enum { PA, PB, PC, PD, PE, PF, PG, PL };

/// the GPIO pin states
enum { LOW, HIGH };

/// the message types
enum
{
    GPIO_MSG_SETUP_FOR_OUTPUT = 0x10,
    GPIO_MSG_SETUP_FOR_INPUT,

    GPIO_MSG_PIN_GET,
    GPIO_MSG_PIN_SET,
    GPIO_MSG_PIN_CLEAR,

    GPIO_MSG_PORT_GET,
    GPIO_MSG_PORT_SET,
    GPIO_MSG_PORT_CLEAR
};

/// the message data sizes
#define GPIO_MSG_BUF_LEN MSG_LEN

/// the message data access
struct gpio_msg_port_t      { uint32_t port; };
struct gpio_msg_port_pin_t  { uint32_t port; uint32_t pin;  };
struct gpio_msg_port_mask_t { uint32_t port; uint32_t mask; };
struct gpio_msg_state_t     { uint32_t state; };




#define PULSGEN_CH_CNT      64  ///< maximum number of pulse generator channels
#define PULSGEN_MAX_DUTY    100 ///< maximum percent of pulse duty cycle (255 is max)
#define PULSGEN_MAX_PERIOD  (UINT32_MAX/(TIMER_FREQUENCY/1000000))

/// messages types
enum
{
    PULSGEN_MSG_PIN_SETUP = 0x20,
    PULSGEN_MSG_TASK_SETUP,
    PULSGEN_MSG_TASK_ABORT,
    PULSGEN_MSG_TASK_STATE,
    PULSGEN_MSG_TASK_TOGGLES
};

/// the message data sizes
#define PULSGEN_MSG_BUF_LEN MSG_LEN

/// the message data access
struct pulsgen_msg_pin_setup_t { uint32_t ch; uint32_t port; uint32_t pin; uint32_t inverted; };
struct pulsgen_msg_task_setup_t { uint32_t ch; uint32_t toggles;
    uint32_t pin_setup_time; uint32_t pin_hold_time; uint32_t start_delay; };
struct pulsgen_msg_ch_t { uint32_t ch; };
struct pulsgen_msg_state_t { uint32_t state; };
struct pulsgen_msg_toggles_t { uint32_t toggles; };




#define ENCODER_CH_CNT 8  ///< maximum number of encoder counter channels

enum { PHASE_A, PHASE_B, PHASE_Z };
enum { PH_A, PH_B, PH_Z };

/// messages types
enum
{
    ENCODER_MSG_PIN_SETUP = 0x30,
    ENCODER_MSG_SETUP,
    ENCODER_MSG_STATE_SET,
    ENCODER_MSG_STATE_GET,
    ENCODER_MSG_COUNTS_SET,
    ENCODER_MSG_COUNTS_GET
};

/// the message data sizes
#define ENCODER_MSG_BUF_LEN MSG_LEN

/// the message data access
struct encoder_msg_ch_t { uint32_t ch; };
struct encoder_msg_pin_setup_t { uint32_t ch; uint32_t phase; uint32_t port; uint32_t pin; };
struct encoder_msg_setup_t { uint32_t ch; uint32_t using_B; uint32_t using_Z; };
struct encoder_msg_state_set_t { uint32_t ch; uint32_t state; };
struct encoder_msg_counts_set_t { uint32_t ch; int32_t counts; };
struct encoder_msg_state_get_t { uint32_t state; };
struct encoder_msg_counts_get_t { int32_t counts; };




#endif
