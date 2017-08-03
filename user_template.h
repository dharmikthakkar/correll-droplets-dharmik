#pragma once

#include "droplet_init.h"

#define MOTORS_MSG_FLAG 'M'

typedef struct motors_msg_struct{
	char flag;
	uint8_t dir;
	int16_t settings[3];
}MotorsMsg;

void		init(void);
void		loop(void);
void		handle_msg(ir_msg* msg_struct);

void		auto_calibration(void);

void		print_rnb_data(void);
void		auto_calibration_dir_6(void);