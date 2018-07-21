/**
 * @file    arisc.c
 *
 * @brief   ARISC firmware API source
 *
 * This test program implements an API to ARISC firmware
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <byteswap.h>
#include "arisc.h"




// private vars

static struct msg_t * msg_arisc[MSG_MAX_CNT] = {0};
static struct msg_t * msg_arm[MSG_MAX_CNT] = {0};
static uint8_t msg_buf[MSG_LEN] = {0};

static uint32_t *vrt_block_addr;




// main entry

int main(void)
{
    uint8_t t = 0, n = 0;

    mem_init();

    // --- GPIO TEST 1 ---------------------------------------------------------

    // setup pins PA15 and PL10
    gpio_pin_setup_for_output(PA,15);
    gpio_pin_setup_for_output(PL,10);

    for(;;)
    {
        // main loop is finite, only 100 pin toggles
        if ( (++n) > 100 ) break;

        // 1sec delay
        usleep(1000000);

        // toggle pins
        if ( t )
        {
            gpio_pin_set(PA,15);
            gpio_pin_clear(PL,10);

            t = 0;
            printf("%d: PA15 = 1, PL10 = 0 \n", n); // debug
        }
        else
        {
            gpio_pin_clear(PA,15);
            gpio_pin_set(PL,10);

            t = 1;
            printf("%d: PA15 = 0, PL10 = 1 \n", n); // debug
        }
    }

    // -------------------------------------------------------------------------

    mem_deinit();

    return 0;
}








// public methods

/**
 * @brief   set pin mode to OUTPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_setup_for_output(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port; tx.pin  = pin;
    msg_send(GPIO_MSG_SETUP_FOR_OUTPUT, (uint8_t*)&tx, 8, 0);
}

/**
 * @brief   set pin mode to INPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_setup_for_input(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port; tx.pin  = pin;
    msg_send(GPIO_MSG_SETUP_FOR_INPUT, (uint8_t*)&tx, 8, 0);
}

/**
 * @brief   get pin state
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  1 (HIGH)
 * @retval  0 (LOW)
 */
uint32_t gpio_pin_get(uint32_t port, uint32_t pin)
{
    uint32_t n = 0;
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);
    struct gpio_msg_state_t rx = *((struct gpio_msg_state_t *) &msg_buf);

    tx.port = port; tx.pin = pin;
    msg_send(GPIO_MSG_PIN_GET, (uint8_t*)&tx, 4, 0);

    // finite loop, only 999999 tries to read an answer
    for ( n = 999999; n--; )
    {
        if ( msg_read(GPIO_MSG_PIN_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        return rx.state;
    }

    return 0;
}

/**
 * @brief   set pin state to HIGH (1)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_set(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port; tx.pin  = pin;
    msg_send(GPIO_MSG_PIN_SET, (uint8_t*)&tx, 8, 0);
}

/**
 * @brief   set pin state to LOW (0)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_clear(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port; tx.pin  = pin;
    msg_send(GPIO_MSG_PIN_CLEAR, (uint8_t*)&tx, 8, 0);
}

/**
 * @brief   get port state
 * @param   port    GPIO port number (0 .. GPIO_PORTS_CNT)
 * @note    each bit value of returned value represents port pin state
 * @retval  0 .. 0xFFFFFFFF
 */
uint32_t gpio_port_get(uint32_t port)
{
    uint32_t n = 0;
    struct gpio_msg_port_t tx = *((struct gpio_msg_port_t *) &msg_buf);
    struct gpio_msg_state_t rx = *((struct gpio_msg_state_t *) &msg_buf);

    tx.port = port;
    msg_send(GPIO_MSG_PORT_GET, (uint8_t*)&tx, 4, 0);

    // finite loop, only 999999 tries to read an answer
    for ( n = 999999; n--; )
    {
        if ( msg_read(GPIO_MSG_PORT_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        return rx.state;
    }

    return 0;
}

/**
 * @brief   set port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT)
 * @param   mask    GPIO pins mask to set   (0 .. 0xFFFFFFFF) \n\n
 *                  mask examples: \n\n
 *                      mask = 0xFFFFFFFF (0b11111111111111111111111111111111) means <b>set all pins state to 1 (HIGH)</b> \n
 *                      mask = 0x00000001 (0b1) means <b>set pin 0 state to 1 (HIGH)</b> \n
 *                      mask = 0x0000000F (0b1111) means <b>set pins 0,1,2,3 states to 1 (HIGH)</b>
 *
 * @retval  none
 */
void gpio_port_set(uint32_t port, uint32_t mask)
{
    struct gpio_msg_port_mask_t tx = *((struct gpio_msg_port_mask_t *) &msg_buf);

    tx.port = port; tx.mask  = mask;
    msg_send(GPIO_MSG_PORT_SET, (uint8_t*)&tx, 8, 0);
}

/**
 * @brief   clear port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT)
 * @param   mask    GPIO pins mask to clear (0 .. 0xFFFFFFFF) \n\n
 *                  mask examples: \n\n
 *                  mask = 0xFFFFFFFF (0b11111111111111111111111111111111) means <b>set all pins state to 0 (LOW)</b> \n
 *                  mask = 0x00000003 (0b11) means <b>set pins 0,1 states to 0 (LOW)</b> \n
 *                  mask = 0x00000008 (0b1000) means <b>set pin 3 state to 0 (LOW)</b>
 *
 * @retval  none
 */
void gpio_port_clear(uint32_t port, uint32_t mask)
{
    struct gpio_msg_port_mask_t tx = *((struct gpio_msg_port_mask_t *) &msg_buf);

    tx.port = port; tx.mask  = mask;
    msg_send(GPIO_MSG_PORT_CLEAR, (uint8_t*)&tx, 8, 0);
}




/**
 * @brief   read a message from the ARISC cpu
 *
 * @param   type    user defined message type (0..0xFF)
 * @param   msg     pointer to the message buffer
 * @param   bswap   0 - if you sending an array of 32bit numbers, 1 - for the text
 *
 * @retval   0 (message read)
 * @retval  -1 (message not read)
 */
int8_t msg_read(uint8_t type, uint8_t * msg, uint8_t bswap)
{
    static uint8_t last = 0;
    static uint8_t m = 0;
    static uint8_t i = 0;
    static uint32_t * link;

    // find next unread message
    for ( i = MSG_MAX_CNT, m = last; i--; )
    {
        // process message only of current type
        if ( msg_arisc[m]->unread && msg_arisc[m]->type == type )
        {
            if ( bswap )
            {
                // swap message data bytes for correct reading by ARM
                link = ((uint32_t*) &msg_arisc[m]->msg);
                for ( i = msg_arisc[m]->length / 4 + 1; i--; link++ )
                {
                    *link = __bswap_32(*link);
                }
            }

            // copy message to the buffer
            memcpy(msg, &msg_arisc[m]->msg, msg_arisc[m]->length);

            // message read
            msg_arisc[m]->unread = 0;
            last = m;
            return msg_arisc[m]->length;
        }

        ++m;
        if ( m >= MSG_MAX_CNT ) m = 0;
    }

    return -1;
}

/**
 * @brief   send a message to the ARISC cpu
 *
 * @param   type    user defined message type (0..0xFF)
 * @param   msg     pointer to the message buffer
 * @param   length  the length of a message (0..MSG_LEN) )
 * @param   bswap   0 - if you sending an array of 32bit numbers, 1 - for the text
 *
 * @retval   0 (message sent)
 * @retval  -1 (message not sent)
 */
int8_t msg_send(uint8_t type, uint8_t * msg, uint8_t length, uint8_t bswap)
{
    static uint8_t last = 0;
    static uint8_t m = 0;
    static uint8_t i = 0;
    static uint32_t * link;

    // find next free message slot
    for ( i = MSG_MAX_CNT, m = last; i--; )
    {
        // sending message
        if ( !msg_arm[m]->unread )
        {
            // copy message to the buffer
            memset( (uint8_t*)((uint8_t*)&msg_arm[m]->msg + length/4*4), 0, 4);
            memcpy(&msg_arm[m]->msg, msg, length);

            if ( bswap )
            {
                // swap message data bytes for correct reading by ARISC
                link = ((uint32_t*) &msg_arm[m]->msg);
                for ( i = length / 4 + 1; i--; link++ )
                {
                    *link = __bswap_32(*link);
                }
            }

            // set message data
            msg_arm[m]->type   = type;
            msg_arm[m]->length = length;
            msg_arm[m]->unread = 1;

            // message sent
            last = m;
            return 0;
        }

        ++m;
        if ( m >= MSG_MAX_CNT ) m = 0;
    }

    // message not sent
    return -1;
}




void mem_init(void)
{
    int32_t     mem_fd;
    uint32_t    vrt_offset = 0;
    off_t       phy_block_addr = 0;
    int32_t     m = 0;

    // open physical memory file
    if ( (mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0 )
    {
       printf("ERROR: can't open /dev/mem file\n");
       return;
    }

    // calculate phy memory block start
    vrt_offset = MSG_ARISC_BLOCK_ADDR % PHY_MEM_BLOCK_SIZE;
    phy_block_addr = MSG_ARISC_BLOCK_ADDR - vrt_offset;

    // make a block of phy memory visible in our user space
    vrt_block_addr = mmap(NULL, 2*MSG_BLOCK_SIZE, PROT_READ | PROT_WRITE,
       MAP_SHARED, mem_fd, phy_block_addr);

    // exit program if mmap is failed
    if (vrt_block_addr == MAP_FAILED)
    {
       printf("ERROR: mmap() failed\n");
       return;
    }

    // adjust offset to correct value
    vrt_block_addr += (vrt_offset/4);

    // assign messages pointers
    for ( m = 0; m < MSG_MAX_CNT; ++m )
    {
        msg_arisc[m] = (struct msg_t *) (vrt_block_addr + (m * MSG_MAX_LEN)/4);
        msg_arm[m]   = (struct msg_t *) (vrt_block_addr + (m * MSG_MAX_LEN + MSG_CPU_BLOCK_SIZE)/4);
    }
}

void mem_deinit(void)
{
    munmap(vrt_block_addr, 2*MSG_BLOCK_SIZE);
}
