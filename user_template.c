#include "user_template.h"

//#define walk_on
//#define rnb_broadcast_sw
#define master_calib
uint32_t last_rnb_time = 0;
uint32_t last_rnb_print_time = 0;

/*
 * any code in this function will be run once, when the robot starts.
 */
void init(){
	set_rgb(0, 255, 255);
	delay_ms(8000);
	//walk(0,100);
	//while(is_moving() != -1);
	//walk(3,100);
	//while(is_moving() != -1);
	//walk(6,80);
	//while(is_moving() != -1);
	//walk(7,80);
	//while(is_moving() != -1);
	//set_rgb(0, 0, 0);
	last_rnb_time = get_time();
//	ir_cmd(ALL_DIRS, "set_motors 0 0 200 200", 22);
#ifdef master_calib
	auto_calibration();
#endif
}


//ir_cmd(ALL_DIRS, "set_motors 0 0 200 200", );
/*
 * the code in this function will be called repeatedly, as fast as it can execute.
 */
void loop(){ 
#ifdef rnb_broadcast_sw
	if(get_time() - last_rnb_time > 8000)
	{
		broadcast_rnb_data();
		last_rnb_time = get_time();
		//set_rgb(0, 255, 0);
		//delay_ms(250);
		//set_rgb(0, 0, 0);
	}
#endif
	if(rnb_updated)
	{
		if(get_time() - last_rnb_print_time > 5000){
			printf("id=%04X\n\r", last_good_rnb.id);
			printf("range=%u\n\r", last_good_rnb.range);
			printf("bearing=%d\n\r", last_good_rnb.bearing);
			printf("heading=%d\n\r", last_good_rnb.heading);
			last_rnb_print_time = get_time();

		}
#ifdef walk_on
		stop_move();
		if(is_moving() == -1)
		{
			if(last_good_rnb.bearing <= 30 && last_good_rnb.bearing >= -30)
			{
				walk(0, 30);
				//set_rgb(255, 255, 0);
				//delay_ms(250);
				//set_rgb(0, 0, 0);
			}
			else if((last_good_rnb.bearing <= -150 && last_good_rnb.bearing >= -180) || (last_good_rnb.bearing <= 180 && last_good_rnb.bearing >= 150))
			{
				walk(3, 30);
				//set_rgb(0, 255, 255);
				//delay_ms(250);
				//set_rgb(0, 0, 0);
			}
			else if(last_good_rnb.bearing > 30 && last_good_rnb.bearing < 150)
			{
				walk(7, 20);
				//set_rgb(255, 0, 255);
				//delay_ms(250);
				//set_rgb(0, 0, 0);
			}
			else if(last_good_rnb.bearing < -30 && last_good_rnb.bearing > -150)
			{
				walk(6, 20);

			}
		}
#endif		
		rnb_updated = 0;
		//set_rgb(0, 0, 255);
		//delay_ms(250);
		//set_rgb(0, 0, 0);		
	}

	delay_ms(10);
}

/*
 * after each pass through loop(), the robot checks for all messages it has 
 * received, and calls this function once for each message.
 */
void handle_msg(ir_msg* msg_struct){
 if(msg_struct->length==sizeof(MotorsMsg)){
	 MotorsMsg* msg = (MotorsMsg*)(msg_struct->msg);
	 if(msg->flag==MOTORS_MSG_FLAG){
		 for(uint8_t i=0;i<3;i++){
			motor_adjusts[msg->dir][i] = msg->settings[i];	 
		 }
	 }
 }
}

/*
 *	the function below is optional - commenting it in can be useful for debugging if you want to query
 *	user variables over a serial connection. it should return '1' if command_word was a valid command,
 *  '0' otherwise.
 */
//uint8_t user_handle_command(char* command_word, char* command_args){
	//if(strcmp_P(command_word,PSTR("sm")){
		//dir = command_args[0];
		//val0 = command_args[1];
		//return 1;	
	//}
	//return 0;
//}

void sendMotorsMsg(uint8_t dir, int16_t mot0, int16_t mot1, int16_t mot2){
	uint8_t rv = 0;
	MotorsMsg msg;
	msg.flag = MOTORS_MSG_FLAG;
	msg.dir = dir;
	msg.settings[0] = mot0;
	msg.settings[1] = mot1;
	msg.settings[2] = mot2;
	do{
		rv = ir_send(ALL_DIRS, (char*)&msg, sizeof(MotorsMsg));
	}while(rv == 0);
}

void print_rnb_data(void){
	printf("id=%04X\n\r", last_good_rnb.id);
	printf("range=%u\n\r", last_good_rnb.range);
	printf("bearing=%d\n\r", last_good_rnb.bearing);
	printf("heading=%d\n\r", last_good_rnb.heading);
}

void auto_calibration(){
	uint8_t rv = 0;
	uint8_t motor1_0 = 0;
	uint8_t motor2_0 = 0;
	int16_t motor1_0_drift = 0;
	int16_t motor2_0_drift = 0;
	int8_t drift_check_i=0;
	rnb cal_temp_rnb;
	int16_t dir0_val[3] = {0, 500, -500};
	uint32_t reset_init_time;
	sendMotorsMsg(0, dir0_val[0], dir0_val[1], dir0_val[2]);
	rnb_updated = 0;
	reset_init_time = get_time();
	while(!rnb_updated){
		if((get_time() - reset_init_time) > 20000){
//			do{
			rv = ir_targeted_cmd(ALL_DIRS, "reset", 5, 0x5D61);					//if slave does not send rnb data within 20 seconds, reset it and write last valid motor settings
//			}while(rv == 0);
			delay_ms(20000);
			sendMotorsMsg(0, dir0_val[0], dir0_val[1], dir0_val[2]);
			reset_init_time = get_time();
		}
	}
	rnb_updated = 0;
	if(last_good_rnb.heading <= 0) last_good_rnb.heading = 360 + last_good_rnb.heading;
	print_rnb_data();
	cal_temp_rnb = last_good_rnb;
	do 
	{
			motor1_0 = 0;
			motor2_0 = 0;
			motor1_0_drift = 0;
			motor2_0_drift = 0;
			for(drift_check_i = 0; drift_check_i < 3; drift_check_i++){
				//do{
					//rv = ir_targeted_cmd(ALL_DIRS, "move_steps 0 100", 16, 0x5D61);
				//}while(rv == 0);
				printf("\n\rmoving droplet\n\r");
				delay_ms(16000);
				rnb_updated = 0;
				reset_init_time = get_time();
				while(!rnb_updated){
					if((get_time() - reset_init_time) > 20000){
//						do{
							rv = ir_targeted_cmd(ALL_DIRS, "reset", 5, 0x5D61);				//if slave does not send rnb data within 20 seconds, reset it and write last valid motor settings
//						}while(rv == 0);
						delay_ms(20000);
						sendMotorsMsg(0, dir0_val[0], dir0_val[1], dir0_val[2]);
						reset_init_time = get_time();
					}
				}
				rnb_updated = 0;
				print_rnb_data();
				if(last_good_rnb.heading <= 0) last_good_rnb.heading = 360 + last_good_rnb.heading;
				if(last_good_rnb.heading > cal_temp_rnb.heading && (last_good_rnb.heading - cal_temp_rnb.heading) > 20){
					motor1_0++;
					motor1_0_drift = (motor1_0_drift + (last_good_rnb.heading - cal_temp_rnb.heading)) / motor1_0;
					printf("\n\rAverage right motor drift is %d\n\r", motor1_0_drift);
				}
				else if(last_good_rnb.heading < cal_temp_rnb.heading && (cal_temp_rnb.heading - last_good_rnb.heading) > 20){
					motor2_0++;
					motor2_0_drift = (motor2_0_drift + (cal_temp_rnb.heading - last_good_rnb.heading))  / motor2_0;
					printf("\n\rAverage left motor drift is %d\n\r", motor2_0_drift);
				}
				cal_temp_rnb = last_good_rnb;
				//if left or right drift count equals 2 break for loop no need of third iteration
				if(motor1_0 == 2 || motor2_0 == 2) break;
				
				//also check the difference in individual drift values. if the difference is unusually large (more than 100-125) the result is erroneous. this is due to the heading angle varies over a range of 15-20 degrees when the droplet is far
				//one possible solution could be to not increment the drift count for difference less than  15-20 degrees (to be tested for confirmation)
			}
			
			printf("\n\rmotor1_0 = %hu motor2_0 = %hu\n\r", motor1_0, motor2_0);
			
			//using the averaged drift_value, reduce the weight of the drifting motor by the averaged value. For eg: if the drift value (left motor i.e motor 2) is 111, set motors to 0 500 -389 from 0 500 -500
			//180 is a magic number as of now. The check is to ensure that the change in weights is not implemented due to erroneous results
			if(motor1_0 > motor2_0 && motor1_0_drift < 180){
				dir0_val[1] = dir0_val[1] - motor1_0_drift;
			}
			else if(motor2_0 > motor1_0 && motor2_0_drift < 180){
				dir0_val[2] = dir0_val[2] + motor2_0_drift;
			}
			sendMotorsMsg(0, dir0_val[0], dir0_val[1], dir0_val[2]);
			printf("\n\rUpdating motor settings with %d %d %d\n\r", dir0_val[0], dir0_val[1], dir0_val[2]);

	} while (motor1_0 != 0 || motor2_0 != 0);
}

