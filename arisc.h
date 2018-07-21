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




#define PULSGEN_CH_CNT      32  ///< maximum number of pulse generator channels
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

#define PULSGEN_MSG_BUF_LEN             MSG_LEN
#define PULSGEN_MSG_CH_CNT              12
#define PULSGEN_MSG_PIN_SETUP_LEN       (4*4*PULSGEN_MSG_CH_CNT)
#define PULSGEN_MSG_TASK_SETUP_LEN      (5*4*PULSGEN_MSG_CH_CNT)
#define PULSGEN_MSG_TASK_ABORT_LEN      (4)
#define PULSGEN_MSG_TASK_STATE_LEN      (4)
#define PULSGEN_MSG_TASK_TOGGLES_LEN    (4*PULSGEN_MSG_CH_CNT)

/// the message data access
#define PULSGEN_MSG_BUF_CHANNEL_ID(LINK,SLOT)       (*((uint32_t*)(LINK) + SLOT))
#define PULSGEN_MSG_BUF_PORT(LINK,SLOT)             (*((uint32_t*)(LINK) + SLOT + 1*PULSGEN_MSG_CH_CNT))
#define PULSGEN_MSG_BUF_PIN(LINK,SLOT)              (*((uint32_t*)(LINK) + SLOT + 2*PULSGEN_MSG_CH_CNT))
#define PULSGEN_MSG_BUF_INVERTED(LINK,SLOT)         (*((uint32_t*)(LINK) + SLOT + 3*PULSGEN_MSG_CH_CNT))

#define PULSGEN_MSG_BUF_PERIOD(LINK,SLOT)           (*((uint32_t*)(LINK) + SLOT + 1*PULSGEN_MSG_CH_CNT))
#define PULSGEN_MSG_BUF_DELAY(LINK,SLOT)            (*((uint32_t*)(LINK) + SLOT + 2*PULSGEN_MSG_CH_CNT))
#define PULSGEN_MSG_BUF_TOGGLES(LINK,SLOT)          (*((uint32_t*)(LINK) + SLOT + 3*PULSGEN_MSG_CH_CNT))
#define PULSGEN_MSG_BUF_DUTY(LINK,SLOT)             (*((uint32_t*)(LINK) + SLOT + 4*PULSGEN_MSG_CH_CNT))

#define PULSGEN_MSG_BUF_CHANNELS_MASK(LINK)         (*((uint32_t*)(LINK)))

#define PULSGEN_MSG_BUF_TOGGLES_MADE(LINK,SLOT)     (*((uint32_t*)(LINK) + SLOT))




#define ENCODER_CH_CNT 8  ///< maximum number of encoder counter channels

/// a channel parameters
struct encoder_ch_t
{
    uint8_t     enabled;
    uint8_t     using_B;
    uint8_t     using_Z;

    uint8_t     port[3];
    uint8_t     pin[3];
    uint8_t     state[3];

    int32_t     counts;
    uint8_t     AB_state;
};

enum { PHASE_A, PHASE_B, PHASE_Z };
enum { PH_A, PH_B, PH_Z };

/// messages types
enum
{
    ENCODER_MSG_PINS_SETUP = 0x30,
    ENCODER_MSG_SETUP,
    ENCODER_MSG_COUNTS,
    ENCODER_MSG_ENABLE,
    ENCODER_MSG_RESET
};

#define ENCODER_MSG_BUF_LEN             MSG_LEN
#define ENCODER_MSG_PINS_SETUP_LEN      (4*6*ENCODER_CH_CNT)
#define ENCODER_MSG_SETUP_LEN           (4*3*ENCODER_CH_CNT)
#define ENCODER_MSG_COUNTS_LEN          (4*ENCODER_CH_CNT)
#define ENCODER_MSG_ENABLE_LEN          (4)
#define ENCODER_MSG_RESET_LEN           (4)

/// the message data access
#define ENCODER_MSG_BUF_PORT(LINK,CH,PHASE)     (*((uint32_t*)(LINK) + 6*CH + PHASE))
#define ENCODER_MSG_BUF_PIN(LINK,CH,PHASE)      (*((uint32_t*)(LINK) + 6*CH + PHASE + 3))

#define ENCODER_MSG_BUF_ENABLED(LINK,CH)        (*((uint32_t*)(LINK) + 3*CH))
#define ENCODER_MSG_BUF_USING_B(LINK,CH)        (*((uint32_t*)(LINK) + 3*CH + 1))
#define ENCODER_MSG_BUF_USING_Z(LINK,CH)        (*((uint32_t*)(LINK) + 3*CH + 2))

#define ENCODER_MSG_BUF_COUNTS(LINK,CH)         (*((uint32_t*)(LINK) + CH))

#define ENCODER_MSG_BUF_ENABLE(LINK)            (*((uint32_t*)(LINK)))

#define ENCODER_MSG_BUF_RESET(LINK)             (*((uint32_t*)(LINK)))




#endif
