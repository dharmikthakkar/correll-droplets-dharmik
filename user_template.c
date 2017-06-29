#include "user_template.h"
uint32_t last_rnb_time = 0;
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
}

/*
 * the code in this function will be called repeatedly, as fast as it can execute.
 */
void loop(){ 
	if(get_time() - last_rnb_time > 5000)
	{
		broadcast_rnb_data();
		last_rnb_time = get_time();
		set_rgb(0, 255, 0);
		delay_ms(250);
		set_rgb(0, 0, 0);
	}
	if(rnb_updated)
	{
		printf("id=%04X\n\r", last_good_rnb.id);
		printf("range=%u\n\r", last_good_rnb.range);
		printf("bearing=%d\n\r", last_good_rnb.bearing);
		printf("heading=%d\n\r", last_good_rnb.heading);
		rnb_updated = 0;
		set_rgb(0, 0, 255);
		delay_ms(250);
		set_rgb(0, 0, 0);		
	}
	if(is_moving() == -1)
	{
		if(last_good_rnb.bearing <= 30 && last_good_rnb.bearing >= -30)
		{
			walk(0, 30);
			set_rgb(255, 255, 0);
			delay_ms(250);
			set_rgb(0, 0, 0);
		}
		else if((last_good_rnb.bearing <= -150 && last_good_rnb.bearing >= -180) || (last_good_rnb.bearing <= 180 && last_good_rnb.bearing >= 150))
		{
			walk(3, 30);
			set_rgb(0, 255, 255);
			delay_ms(250);
			set_rgb(0, 0, 0);
		}
		else if(last_good_rnb.bearing > 30 && last_good_rnb.bearing < 150)
		{
			walk(7, 20);
			set_rgb(255, 0, 255);
			delay_ms(250);
			set_rgb(0, 0, 0);			
		}
		else if(last_good_rnb.bearing < -30 && last_good_rnb.bearing > -150)
		{
			walk(6, 20);

		}	
	}
}

/*
 * after each pass through loop(), the robot checks for all messages it has 
 * received, and calls this function once for each message.
 */
void handle_msg(ir_msg* msg_struct){

}

///*
 //*	the function below is optional - commenting it in can be useful for debugging if you want to query
 //*	user variables over a serial connection. it should return '1' if command_word was a valid command,
 //*  '0' otherwise.
 //*/
//uint8_t user_handle_command(char* command_word, char* command_args){
	//return 0;
//}