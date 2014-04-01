//
//  camLin.h
//  camLin
//
//  Created by Marco on 22.02.14.
//
//

#ifndef camLin_camLin_h
#define camLin_camLin_h


void lcam_reset(void);
void lcam_setup(void);
void lcam_reg_write(uint8_t mode, uint8_t option);
void lcam_send(uint8_t value);
void lcam_pulse_clock(uint8_t times);
void lcam_pulse();
void lcam_startintegration(void);
void lcam_endintegration(void);
void lcam_read(void);
uint8_t lcam_getpic(void);

#endif
