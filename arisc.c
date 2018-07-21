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

static uint32_t *vrt_block_addr;




// public methods

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
 * @brief   send a message to the ARM cpu
 *
 * @param   type    user defined message type (0..0xFF)
 * @param   msg     pointer to the message buffer
 * @param   length  the length of a message ( 0 .. (MSG_MAX_LEN-4) )
 * @param   bswap   0 - if you sending a structure of 32bit numbers, 1 - for the others
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




int main(void)
{
    int32_t     mem_fd;
    uint32_t    vrt_offset = 0;
    off_t       phy_block_addr = 0;
    int32_t     m = 0;
    uint8_t     buf[MSG_LEN] = {0};


    // open physical memory file
    if ( (mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0 )
    {
       printf("ERROR: can't open /dev/mem file\n");
       return -1;
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
       return -1;
    }

    // adjust offset to correct value
    vrt_block_addr += (vrt_offset/4);

    // assign messages pointers
    for ( m = 0; m < MSG_MAX_CNT; ++m )
    {
        msg_arisc[m] = (struct msg_t *) (vrt_block_addr + (m * MSG_MAX_LEN)/4);
        msg_arm[m]   = (struct msg_t *) (vrt_block_addr + (m * MSG_MAX_LEN + MSG_CPU_BLOCK_SIZE)/4);
    }

    


    // setup vars
    uint8_t t = 0, n = 0;

    // setup output message buffer view
    struct gpio_msg_port_pin_t out = *((struct gpio_msg_port_pin_t *) &buf);

    // setup pins
    {
        // cleanup buffer
        memset(&buf, 0, MSG_LEN);

        // setup PA15 as output
        out.port = PA; out.pin  = 15;
        msg_send(GPIO_MSG_SETUP_FOR_OUTPUT, (uint8_t*)&buf, 8, 0);

        // setup PL10 as output
        out.port = PL; out.pin  = 10;
        msg_send(GPIO_MSG_SETUP_FOR_OUTPUT, (uint8_t*)&buf, 8, 0);
    }




    for(;;)
    {
        n++;

        // 1sec delay
        usleep(1000000);

        // cleanup buffer
        memset(&buf, 0, MSG_LEN);

        // toogle pins PA15 and PL10
        if ( t )
        {
            // set PA15 state = 1
            out.port = PA; out.pin  = 15;
            msg_send(GPIO_MSG_PIN_SET, (uint8_t*)&buf, 8, 0);

            // set PL10 state = 0
            out.port = PL; out.pin  = 10;
            msg_send(GPIO_MSG_PIN_CLEAR, (uint8_t*)&buf, 8, 0);

            t = 0;
            printf("%d: PA15 = 1, PL10 = 0 \n", n);
        }
        else
        {
            // set PA15 state = 0
            out.port = PA; out.pin  = 15;
            msg_send(GPIO_MSG_PIN_CLEAR, (uint8_t*)&buf, 8, 0);

            // set PL10 state = 1
            out.port = PL; out.pin  = 10;
            msg_send(GPIO_MSG_PIN_SET, (uint8_t*)&buf, 8, 0);

            t = 1;
            printf("%d: PA15 = 0, PL10 = 1 \n", n);
        }
    }




    munmap(vrt_block_addr, 2*MSG_BLOCK_SIZE);

    return 0;
}
