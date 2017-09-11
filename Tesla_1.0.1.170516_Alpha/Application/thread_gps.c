
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:			thread_gps.c
  Author:			   	rui.fang
  Date:			    	2017.5.16
  Version:        
  Description:    // 用于详细说明此程序文件完成的主要功能，与其他模块
                  // 或函数的接口，输出值、取值范围、含义及参数间的控
                  // 制、顺序、独立或依赖等关系
  History:        // 修改历史记录列表，每条修改记录应包括修改日期、修改
                  // 者及修改内容简述  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/


#include "stm32f4xx.h"
#include <rtthread.h>
#include <stdio.h>
#include "insgps.h"
#include "time_control.h"
#include "usart.h"
#include "encoder.h"
#include "../communication/include/debug.h"
#include "lcd12864_io_spi.h"
#include "mower_common.h"

/*在分配堆栈空间时，必须要对其*/
ALIGN(RT_ALIGN_SIZE)
char thread_gps_stack[3072];
struct rt_thread thread_gps;
local_time ltime = {0};
float pos_ned_m[3];
float vel_ned_m_s[3];

/*170803 VARS AND FUNCTIONS*/
gpsx GPS_DATA;
rt_size_t result_size = 0;
u8 USART_RX_BUFFER[200]; 

void ubx_nav_pvt(gpsx *gpsx,u8 *buf)
{ 
	s32 temp;
	
	//u2
	temp = buf[5];
	temp = temp << 8;
	temp = temp + buf[4];
	gpsx->year = temp;
	
	//u1
	gpsx->month = buf[6];
	gpsx->day 	= buf[7];
	gpsx->hour 	= buf[8];
	gpsx->min 	= buf[9];
	gpsx->sec 	= buf[10];
	
	//x1
	gpsx->valid = buf[11];
	
	//u4
	temp = buf[15];
	temp = temp << 8;
	temp = temp + buf[14];
	temp = temp << 8;
	temp = temp + buf[13];
	temp = temp << 8;
	temp = temp + buf[12];
	gpsx->tAcc_ns = (float)temp;
	
	//i4
	temp = buf[19];
	temp = temp << 8;
	temp = temp + buf[18];
	temp = temp << 8;
	temp = temp + buf[17];
	temp = temp << 8;
	temp = temp + buf[16];
	gpsx->ns = temp;
	
	//i4
	temp = buf[27];
	temp = temp << 8;
	temp = temp + buf[26];
	temp = temp << 8;
	temp = temp + buf[25];
	temp = temp << 8;
	temp = temp + buf[24];
	gpsx->longitude_rad = (float)temp*1e-7*PI/180;
	
	temp = buf[31];
	temp = temp << 8;
	temp = temp + buf[30];
	temp = temp << 8;
	temp = temp + buf[29];
	temp = temp << 8;
	temp = temp + buf[28];
	gpsx->latitude_rad = (float)temp*1e-7*PI/180;
	
	temp = buf[35];
	temp = temp << 8;
	temp = temp + buf[34];
	temp = temp << 8;
	temp = temp + buf[33];
	temp = temp << 8;
	temp = temp + buf[32];
	gpsx->height_m = (float)temp*0.001;

	temp = buf[39];
	temp = temp << 8;
	temp = temp + buf[38];
	temp = temp << 8;
	temp = temp + buf[37];
	temp = temp << 8;
	temp = temp + buf[36];
	gpsx->hMSL_m = (float)temp*0.001;
		
	//u4
	temp = buf[43];
	temp = temp << 8;
	temp = temp + buf[42];
	temp = temp << 8;
	temp = temp + buf[41];
	temp = temp << 8;
	temp = temp + buf[40];
	gpsx->hAcc_m = (float)temp*0.001;
	
	temp = buf[47];
	temp = temp << 8;
	temp = temp + buf[46];
	temp = temp << 8;
	temp = temp + buf[45];
	temp = temp << 8;
	temp = temp + buf[44];
	gpsx->vAcc_m = (float)temp*0.001;
	
	//i4
	temp = buf[51];
	temp = temp << 8;
	temp = temp + buf[50];
	temp = temp << 8;
	temp = temp + buf[49];
	temp = temp << 8;
	temp = temp + buf[48];
	gpsx->velN_m_s = (float)temp*0.001;
	
	temp = buf[55];
	temp = temp << 8;
	temp = temp + buf[54];
	temp = temp << 8;
	temp = temp + buf[53];
	temp = temp << 8;
	temp = temp + buf[52];
	gpsx->velE_m_s = (float)temp*0.001;
	
	temp = buf[59];
	temp = temp << 8;
	temp = temp + buf[58];
	temp = temp << 8;
	temp = temp + buf[57];
	temp = temp << 8;
	temp = temp + buf[56];
	gpsx->velD_m_s = (float)temp*0.001;
	
	//u4
	temp = buf[71];
	temp = temp << 8;
	temp = temp + buf[70];
	temp = temp << 8;
	temp = temp + buf[69];
	temp = temp << 8;
	temp = temp + buf[68];	
	gpsx->sAcc_m_s = (float)temp*0.001;
	
	temp = buf[75];
	temp = temp << 8;
	temp = temp + buf[74];
	temp = temp << 8;
	temp = temp + buf[73];
	temp = temp << 8;
	temp = temp + buf[72];	
	gpsx->headAcc_deg = (float)temp*1e-5;
	
	temp = buf[67];
	temp = temp << 8;
	temp = temp + buf[66];
	temp = temp << 8;
	temp = temp + buf[65];
	temp = temp << 8;
	temp = temp + buf[64];	
	gpsx->heading_deg = (float)temp*1e-5;
	
	gpsx->numSV = buf[23];
	
	//local time 赋值
	ltime.year  = gpsx->year;
	ltime.month = gpsx->month;
	ltime.day   = gpsx->day;
	ltime.hour  = gpsx->hour+8;            //for CHINA in UTC-8
	ltime.min   = gpsx->min;
	ltime.sec   = gpsx->sec;
}

//void local_time_init(gpsx *gps)
//{
//	ltime.year  = gps->year;
//	ltime.month = gps->month;
//	ltime.day   = gps->day;
//	ltime.hour  = gps->hour+8;            //for CHINA in UTC-8
//	ltime.min   = gps->min;
//	ltime.sec   = gps->sec;
//}

void pos_init(gpsx *gps)
{
	pos_lla_init[0] = (double)gps->latitude_rad;
	pos_lla_init[1] = (double)gps->longitude_rad;
	pos_lla_init[2] = (double)gps->hMSL_m;
}

void get_local_time(local_time *time)
{
	time->year  = ltime.year;
	time->month = ltime.month;
	time->day   = ltime.day;
	time->hour  = ltime.hour;
	time->min   = ltime.min;
	time->sec   = ltime.sec;
	
}

void get_pos_ned(float pos[3])
{
	pos[0] = pos_ned_m[0];
	pos[1] = pos_ned_m[1];
	pos[2] = pos_ned_m[2];
}

void get_vel_ned(float vel[3])
{
	vel[0] = vel_ned_m_s[0];
	vel[1] = vel_ned_m_s[1];
	vel[2] = vel_ned_m_s[2];
}

u8 get_gps_data(u8 *init_flag)
{
	s16 i;
	u8 gpsdata_index = 0;
	u8 gpsdata_flag = 0;
	
	if((USART_RX_BUFFER[0] == 0xB5) &&(USART_RX_BUFFER[1] == 0x62)){
		if((USART_RX_BUFFER[2]==0x01)&&(USART_RX_BUFFER[3]==0x07)&&(USART_RX_BUFFER[4]==0x5C)&&(USART_RX_BUFFER[5]==0x00)){
				gpsdata_flag = 1;
				USART_Cmd(USART_COM, DISABLE); 
		}
	}
	
	if(gpsdata_flag == 1){
		ubx_nav_pvt(&GPS_DATA, (u8*)USART_RX_BUFFER+6+gpsdata_index);   //ubx协议解析，本地时间赋值
		USART_Cmd(USART_COM, ENABLE);
		if(*init_flag == 1){
			//local_time_init(&GPS_TESTDATA);
			pos_init(&GPS_DATA);
			*init_flag = 0;
	  }
	}

	return gpsdata_flag;
	
}

/******************integrated navigation filter main function ******************************************/
void mower_gps_thread(void* parameter)
{
	static unsigned short int i = 0;
	float acc_imu_m_s2[3] ;
	static float P[49] = {100,0,0,0,0,0,0,0,100,0,0,0,0,0,0,0,100,0,0,0,0,0,0,0,100,0,0,0,0,0,0,0,100,0,0,0,0,0,0,0,100,0,0,0,0,0,0,0,100};
	float Q[9] = {0.800000000000000,0,0,0,0.800000000000000,0,0,0,0.800000000000000};
	float stateVec[7] = {0};
  float R_odom[9] = {0.00001*0.4,0,0,0,0.00001*0.4,0,0,0,0.01*0.4};	
	u8 gpsdata_flag = 0;
	u8 init_flag = 1;
	u8 x;
	float odom_valid,delta_odo_l,delta_odo_r;
	static float pos_prev[3] = {0,0,0},vel_prev[3]={0,0,0};
	float pos_temp[3],vel_temp[3];
	
	rt_uint32_t recved;
	
	while (1)
	{
		//rt_enter_critical();
		//LCD_PWM = 0;
		
		// 等待50ms的时间事件
		rt_event_recv(&sys_event, SYS_EVN_GPS, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		
		x = fabs(g_sensor_scaled_body.acc_m_s2[2])>4.5f;
		if(x&&is_odo_valid == 1){
			/**************************prepare acceleration data **********************/
			memcpy(acc_imu_m_s2,g_sensor_scaled_body.acc_m_s2,sizeof(acc_imu_m_s2));
			
			/***************integrated navigation: time update*************************/
			g_timediff_s_gps = (float)get_time_diff2()*1e-6;		
			ekf7_tu(acc_imu_m_s2, pos_prev,vel_prev,quat,g_timediff_s_gps, P, Q, pos_temp, vel_temp,stateVec);
	
			/***************integrated navigation: GPS measurement update*************************/
			//get gps data
			gpsdata_flag = get_gps_data(&init_flag);	
			if((init_flag==0)&&(GPS_DATA.hAcc_m<10)&&(GPS_DATA.valid ==1)){   //初始化完成后
				//ekf7_gps6_mu(pos_temp, vel_temp, &eul_rad[0],stateVec,P, pos_lla_init,&GPS_DATA);
			}
			
		  /***************integrated navigation: Odometer measurement update********************/
			odom_preprocessing(&odom_valid,&delta_odo_l,&delta_odo_r);
			if(odom_valid==1){
				ekf7_odom_mu( pos_prev,vel_prev,&eul_rad[0], stateVec, delta_odo_r,delta_odo_l,P, R_odom, 0.3195,pos_temp,  vel_temp);
				is_odo_ready = 1;

			}

      //copy pos/vel to pos/vel previous for next iteration
			memcpy(pos_prev,pos_temp,sizeof(pos_prev));
			memcpy(vel_prev,vel_temp,sizeof(vel_prev));
			memcpy(pos_ned_m,pos_temp,sizeof(pos_temp));
			memcpy(vel_ned_m_s,vel_temp,sizeof(vel_ned_m_s));
			
      //update eul and fusion updated quat into rotation vector   //!!!!!!!!!!!11shoule move into update functions later
//			quat2eulf(quat, eul_rad);
//			rot_update(quat,rot_vec);

			//temperory data send function
			send_data(g_timediff_s_gps,delta_odo_l,delta_odo_r,GPS_DATA);
		}
		//LCD_PWM = 1;
		//rt_exit_critical();
		//rt_thread_delay(200);// 200ms 钟执行一次此线程
    }
}
