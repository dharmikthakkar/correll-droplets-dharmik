#pragma once

#include "droplet_init.h"

#define MOTORS_MSG_FLAG 'M'

#define SLAVE 0x4177

typedef struct motors_msg_struct{
	char flag;
	uint8_t dir;
	int16_t settings[3];
}MotorsMsg;

void		init(void);
void		loop(void);
void		handleMsg(irMsg* msgStruct);

void		auto_calibration(void);

void		print_rnb_data(void);
void		auto_calibration_dir_6(void);
void		follow_droplet(void);