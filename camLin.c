//
//  camLin.c
//  camLin
//
//  Created by Marco on 22.02.14.
//
//

#include <stdio.h>

#include <avr/io.h>
#include <util/delay.h>
#include "usiTwiSlave.h"
#include "camLin.h"


/* I2C slave address */
#define ADDRESS 0x05

/* Commands */
#define RESET               0x1B
#define REG_WRITE           0x40
#define NO_SLEEP            0x00
#define SLEEP               0x10
#define START_INTEGRATION   0x08
#define END_INTEGRATION     0x10
#define READ_PIXEL          0x02

/* addresses */
#define MODE                0x1F
#define LEFT_OFFSET_ADDR    0x00
#define LEFT_GAIN_ADDR      0x01
#define CENTER_OFFSET_ADDR  0x02
#define CENTER_GAIN_ADDR    0x03
#define RIGHT_OFFSET_ADDR   0x04
#define RIGHT_GAIN_ADDR     0x05

/* Global variables */
uint16_t integration_steps;

/* ATTENTIOOOOOON. END PAGE 3 DATASHEET CAMERA
 but a minimum of 5 clocks after the stop bit is required after any command has been issued to ensure that the corresponding internal logic actions have been completed.
 */

/* ATTENTIOOOOOON. DATASHEET CAMERA, SEE MAXIMAL CLOCK FREQUENCY
    make clock slower with nop
 */


int main(void)
{
    uint16_t j;
    
    asm("cli");
    DDRB |= 0x2;    // clock pin, as output
    DDRB &= 0xF7;    // SDOUT pin, as input
    DDRB |= 0x10;   // SDIN pin, as output
    usiTwiSlaveInit(ADDRESS);
    asm("sei");
    lcam_setup();
    
    
    while(1)
    {
        integration_steps = 0;
        integration_steps = rxbuffer[INTEG_TIME_LOW] | (((uint16_t)rxbuffer[INTEG_TIME_HIGH]) << 8);
        lcam_startintegration();
        
        /* integration time delay */
        for(j=(integration_steps >> 2);j>0;j--)
            _delay_us(4);
        
        lcam_endintegration();
        lcam_read();
        txbuffer[102] = lcam_getpic();
        
    }
    return 0;   
}

// Initialization Sequence
void lcam_reset(void)
{
    _delay_ms(1);
    
    
    // 10 clock impulsions with SDIN held high to clear the receiver logic
    PORTB |= 0x10;           // set SDIN pin
    lcam_pulse_clock(10);
    // 3 reset instructions to clear the control logic
    lcam_send(RESET);
    lcam_send(RESET);
    lcam_send(RESET);
    // 30 clock impulsions to assure the state of SDOUT
    lcam_pulse_clock(30);
    
    // register write mode
    lcam_reg_write(MODE, NO_SLEEP);
    
    // reset settings register values
    rxbuffer[LEFT_OFFSET]  = 0;
    rxbuffer[CENTER_OFFSET] = 0;
    rxbuffer[RIGHT_OFFSET] = 0;
    rxbuffer[LEFT_GAIN] = 15;
    rxbuffer[CENTER_GAIN] = 15;
    rxbuffer[RIGHT_GAIN] = 15;
    rxbuffer[INTEG_TIME_LOW] = 150;
    rxbuffer[INTEG_TIME_HIGH] = 0;
}


void lcam_setup(void)
{
    lcam_reset();
    

    // make sure the clock is cleared
    PORTB &= 0xFD;   // clock pin = 0
    
    // Left offset
    lcam_reg_write(LEFT_OFFSET_ADDR, rxbuffer[LEFT_OFFSET]);
    // Left gain
    lcam_reg_write(LEFT_GAIN_ADDR, rxbuffer[LEFT_GAIN]);
    // Center offset
    lcam_reg_write(CENTER_OFFSET_ADDR, rxbuffer[CENTER_OFFSET]);
    // Center gain
    lcam_reg_write(CENTER_GAIN_ADDR, rxbuffer[CENTER_GAIN]);
    // Right offset
    lcam_reg_write(RIGHT_OFFSET_ADDR, rxbuffer[RIGHT_OFFSET]);
    // Right gain
    lcam_reg_write(RIGHT_GAIN_ADDR, rxbuffer[RIGHT_GAIN]);
}

void lcam_reg_write(uint8_t mode, uint8_t option)
{
    // register write mode
    lcam_send(REG_WRITE+mode);
    // Init, no sleep
    lcam_send(option);
}

// send a byte to the camera
void lcam_send(uint8_t value)
{
    uint8_t i;
    
    // Start bit
    PORTB &= 0xEF;           // clear SDIN pin
    lcam_pulse();
   
    // Send 8 bits to camera
    for(i = 0; i < 8; i++)
    {
        if ((value >> i) & 1)                   // bit is 1
            PORTB |= 0x10;   // set SDIN pin
        else                                    // bit is 0
            PORTB &= 0xEF;   // clear SDIN pin
        
        lcam_pulse();
    }
    
    // Stop bit
    PORTB |= 0x10;           // set SDIN pin
    lcam_pulse();
}

void lcam_pulse_clock(uint8_t times)
{
    uint8_t i;
    for(i = 0; i < times; i++)
    {
        lcam_pulse();
    }
}

void lcam_pulse()
{
    PORTB |= 0x2;    // clock pin = 1
    PORTB &= 0xFD;   // clock pin = 0
}

void lcam_startintegration(void)
{
    // Send start integration command
    lcam_send(START_INTEGRATION);
    // delayed until the pixel reset cycle has been completed (22-clock delay)
    lcam_pulse_clock(22);
}

void lcam_endintegration(void)
{
    // Sample int command
    lcam_send(END_INTEGRATION);
    // pixel reset sequence is initiated, requires 22 clocks
    lcam_pulse_clock(22);
}


// Tell the camera to be ready to send data
void lcam_read(void)
{
    uint8_t i, pixel_bit, pixel;
    // Read pixel command
    lcam_send(READ_PIXEL);
    // 44-clock cycle delay until the first pixel data is output
    lcam_pulse_clock(44);
    // Read the 102 pixels from the camera
    for(i = 0; i < 102; i++)
    {
        pixel = 0;
        // pulse the pixel start bit (SDOUT = 0)
        lcam_pulse();
        // read a byte, bit by bit
        for(pixel_bit = 0; pixel_bit < 8; pixel_bit++)
        {
            PORTB |= 0x2;    // clock pin = 1
            // read pin while clock is at 1
            pixel |= (((PINB >> 3) & 1) << pixel_bit);
            PORTB &= 0xFD;   // clock pin = 0
        }
        // store byte to buffer
        txbuffer[i] = pixel;
        // pulse the pixel stop bit (SDOUT = 1)
        lcam_pulse();
    }
}

// Divide the 100 first pixels into 25 4-byte averages and return the highest average index
uint8_t lcam_getpic(void)
{
    uint8_t i, max_region = 0;      // why was it char?
    uint16_t value=0, average = 0, highest = 0;
    for(i = 0; i < 25; i++)
    {
        // take 4-byte average and divide by 4 (shift to right by 2)
        value = (((uint16_t)txbuffer[i*4] + (uint16_t)txbuffer[i*4+1] + (uint16_t)txbuffer[i*4+2] + (uint16_t)txbuffer[i*4+3]) >> 2);
        if(value > highest)
        {
            highest = value;
            max_region = i;
        }
        average += value;
    }
    
    average /= 25;
    
    if(highest > average + 30)
    {
        return max_region;
    }
    else
    {
        return 0;
    }
}


