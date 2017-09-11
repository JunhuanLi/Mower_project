
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:			thread_imu.c
  Author:				  rui.fang
  Date:				    2017.7.7
  Version:        

    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/
#include "stm32f4xx.h"
#include <rtthread.h>
#include <stdio.h>
#include "imu.h"
#include "time_control.h"
#include "typedef.h"
#include "lcd12864_io_spi.h"
#include "mower_common.h"
#include "../communication/include/debug.h"


ALIGN(RT_ALIGN_SIZE)
char thread_imu_stack[2048];
struct rt_thread thread_imu;

/*********Variables**************/
static float gyro_group_x[11] = {0};
static float gyro_group_y[11] = {0};
static float gyro_group_z[11] = {0};
static float acc_group_x[11] = {0};
static float acc_group_y[11] = {0};
static float acc_group_z[11] = {0};
static float mag2_group_x[11] = {0};
static float mag2_group_y[11] = {0};
static float mag2_group_z[11] = {0};
float mag_declination_rad = -0.0421;
volatile int is_att_valid = 0;
volatile int is_odo_ready = 0;
/*********Functions***************/
float grav_acc(float lat_rad)
{	
  return 9.806F - 0.0260000229F * (float)cos(2.0F * lat_rad);
}


void quat_norm(float quat[4])
{
	
	float norm_q;
	norm_q = sqrt(quat[0]*quat[0]+quat[1]*quat[1]+quat[2]*quat[2]+quat[3]*quat[3]);
	quat[0] = quat[0]/norm_q;
	quat[1] = quat[1]/norm_q;
	quat[2] = quat[2]/norm_q;
	quat[3] = quat[3]/norm_q;
	
}


void  get_rot_vector(float vector[3])               //get rotation vector
{
	vector[0] = rot_vec[0];
	vector[1] = rot_vec[1];
	vector[2] = rot_vec[2];
}


void  get_eul_rad(float angle[3])
{
	angle[0] = eul_rad[0];
	angle[1] = eul_rad[1];
	angle[2] = eul_rad[2];
}

void mpu_preprocessing(float grav,imu_scaled_body *imu,float *yaw1_after_cali,float *yaw2_after_cali)
{
	
	T_mpu mpu;
	T_mag mag1,mag2;

	//get sensor data
	memcpy(&mpu, &g_sensor_info.mpu,sizeof(mpu));
	memcpy(&mag1,&g_sensor_info.mag1,sizeof(mag1));
	memcpy(&mag2,&g_sensor_info.mag2,sizeof(mag2));

	mag_ellipse_mapping(&mag_cali_ellipse, &mag1,imu->mag1);
	mag_ellipse_mapping(&mag_cali_ellipse2, &mag2,imu->mag2);
	imu->imu_temp    = (float)mpu.temp*0.01;
	imu->acc_m_s2[0] = (float)mpu.ax/16384*grav;
	imu->acc_m_s2[1] = (float)mpu.ay/16384*grav;
	imu->acc_m_s2[2] = (float)mpu.az/16384*grav;
	imu->gyro_rps[0] = (float)mpu.gx/GYRO_FACTOR*PI/180 - gyro_bias[0];
	imu->gyro_rps[1] = (float)mpu.gy/GYRO_FACTOR*PI/180 - gyro_bias[1];
	imu->gyro_rps[2] = (float)mpu.gz/GYRO_FACTOR*PI/180 - gyro_bias[2];
	
	//磁罗盘方向指示，向东转为正，向西转为负，本体坐标系中yaw顺时针为正，此处完成磁罗盘航向到本体航向转换
	*yaw1_after_cali  = -(atan2f((float)imu->mag1[1],(float)imu->mag1[0]) - mag_declination_rad);	
	*yaw2_after_cali  = -(atan2f((float)imu->mag2[1],(float)imu->mag2[0]) - mag_declination_rad);	
	
}


void frame_alignment(imu_scaled_body *imu)
{
	//for current installation
	imu_scaled_body imu_mem;
	memcpy(&imu_mem,imu,sizeof(imu_scaled_body));
	imu->acc_m_s2[0] = (imu_mem.acc_m_s2[0]-0);
	imu->acc_m_s2[1] = -imu_mem.acc_m_s2[1]-0.16;
	imu->acc_m_s2[2] = -imu_mem.acc_m_s2[2]-0.2;//!!!!!!!!!!!!!!!!!!!temporary
	imu->gyro_rps[0] = imu_mem.gyro_rps[1];
	imu->gyro_rps[1] = -imu_mem.gyro_rps[0];
	imu->gyro_rps[2] = -imu_mem.gyro_rps[2];
	
}


void hampel_imu(imu_scaled_body *imu)
{
	u16 i;
	for(i=0;i<10;i++) {
		gyro_group_x[i] = gyro_group_x[i+1]; gyro_group_y[i] = gyro_group_y[i+1]; gyro_group_z[i] = gyro_group_z[i+1];
		acc_group_x[i] = acc_group_x[i+1];   acc_group_y[i] = acc_group_y[i+1];   acc_group_z[i] = acc_group_z[i+1];
		mag2_group_x[i] = mag2_group_x[i+1]; mag2_group_y[i] = mag2_group_y[i+1]; mag2_group_z[i] = mag2_group_z[i+1];
	}
			
	gyro_group_x[10] = imu->gyro_rps[0];   gyro_group_y[10] = imu->gyro_rps[1];   gyro_group_z[10] = imu->gyro_rps[2];
	acc_group_x[10]  = imu->acc_m_s2[0];   acc_group_y[10]  = imu->acc_m_s2[1];   acc_group_z[10]  = imu->acc_m_s2[2];
	mag2_group_x[10] = imu->mag2[0];       mag2_group_y[10] = imu->mag2[1];       mag2_group_z[10] = imu->mag2[2];
				
	if((gyro_group_x[0]!=0)&&(acc_group_x[0]!=0)){
		imu->gyro_rps[0] = hampel(gyro_group_x);
		imu->gyro_rps[1] = hampel(gyro_group_y);
		imu->gyro_rps[2] = hampel(gyro_group_z);
		imu->acc_m_s2[0] = hampel(acc_group_x);
		imu->acc_m_s2[1] = hampel(acc_group_y);
		imu->acc_m_s2[2] = hampel(acc_group_z);
	}
	
	if(mag2_group_x[0]!=0){
		imu->mag2[0] = hampel(mag2_group_x);
		imu->mag2[1] = hampel(mag2_group_y);
		imu->mag2[0] = hampel(mag2_group_z);
	}


}

void rot_update(float quat[4],float rotVec[3])
{
	
  float b_quat[9];
  int i0;
  int i1;
  static const signed char b[3] = { 1, 0, 0 };
	
  b_quat[0] = quat[0] * quat[0] + quat[1] * quat[1] - quat[2] * quat[2] - quat[3]* quat[3];
  b_quat[3] = 2.0F * (quat[1] * quat[2] - quat[0] * quat[3]);
  b_quat[6] = 2.0F * (quat[1] * quat[3] + quat[0] * quat[2]);
  b_quat[1] = 2.0F * (quat[1] * quat[2] + quat[0] * quat[3]);
  b_quat[4] = quat[0] * quat[0] - quat[1] * quat[1] + quat[2] * quat[2] - quat[3]* quat[3];
  b_quat[7] = 2.0F * (quat[2] * quat[3] - quat[0] * quat[1]);
  b_quat[2] = 2.0F * (quat[1] * quat[3] - quat[0] * quat[2]);
  b_quat[5] = 2.0F * (quat[2] * quat[3] + quat[0] * quat[1]);
  b_quat[8] = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3]* quat[3];
  for (i0 = 0; i0 < 3; i0++) {
	  rot_vec[i0] = 0.0F;
	  for (i1 = 0; i1 < 3; i1++) {
		  rotVec[i0] += b_quat[i0 + 3 * i1] * (float)b[i1];
    }
  }
}




/**************** main function in attitude calculation thread **********************/
void mower_imu_thread(void* parameter)
{
	float grav;
	float QAtt[9] = {0.000111111111111111,0,0,0,0.000111111111111111,0,0,0,0.000217777777777778};
	float RAtt[9] = {0.00111111111111111,0,0,0,0.00111111111111111,0,0,0,0.00111111111111111};
	static float PAtt[16] = {0.3,0,0,0,0,0.3,0,0,0,0,0.3,0,0,0,0,0.3};
  static float quatl[4] = {1,0,0,0};       
	u16 i;		
	imu_scaled_body imu;
	s8 init_flag = 1;
	static float eul_temp[3];
	//yaw measurement update
	float P1 = 1;
	float Q_mag = 0.1;
	float R_mag = 0.01;
	static float yaw1_mag_prev=0;
	static float yaw2_mag_prev=0;
	float yaw1_after_cali_rad=0,yaw2_after_cali_rad=0;//分别是磁罗盘1和2经过校准后输出的航向角
	float trace_P;
	volatile float trace_P_var;
	rt_uint32_t recved;
	float delta_yaw=0;
	grav = grav_acc((float)pos_lla_init[0]);                 //get local gravity accleration
	while (1)
	{
		//rt_enter_critical();
		//
		if(is_imu_valid==1){
		rt_event_recv(&sys_event, SYS_EVN_IMU, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		
		mpu_preprocessing(grav,&imu,&yaw1_after_cali_rad,&yaw2_after_cali_rad);//22ms

		//yaw initialization
		if(init_flag==1){
			yaw1_mag_prev = yaw1_after_cali_rad;  
			yaw2_mag_prev = yaw2_after_cali_rad;  //暂时用磁罗盘1
			init_flag = 0;
		}
			
		//outlier removal
		hampel_imu(&imu); //2ms

		//frame alignment
		frame_alignment(&imu);//us
 		memcpy(&g_sensor_scaled_body,&imu,sizeof(imu_scaled_body));
		
		//get time difference dt
		g_timediff_s_imu = (float)get_time_diff1()*1e-6;		

		//attitude calculation:time update 
		att_tu(PAtt,quatl,&imu,(float)g_timediff_s_imu, QAtt);
				
		//measurement update by accelerometer
		trace_P = att_mu_acc(PAtt,&imu,quatl,RAtt);
//    trace_P_var = movvarf(trace_P);

//		if((trace_P_var!=0)&&(trace_P_var<0.005)&&(is_att_valid==0)){
//			quatl[0]=1;
//			for(i=1;i<4;i++) quatl[i] = 0;
//			is_att_valid = 1;
//			delta_yaw = eul_temp[0];
//		}
		
		// quaternion renormalization
		quat_norm(quatl);		
		quat2eulf(quatl, eul_temp);
		eul_temp[0] = eul_temp[0]+delta_yaw;
		eul2quatf(eul_temp, quatl);


		//measurement update by magneter1
//		eul_temp[0] = att_mu_mag( &P1, yaw1_mag_prev, yaw1_after_cali_rad, R_mag, Q_mag);
//		yaw1_mag_prev = eul_temp[0];
//		
//		/*fusion mag yaw into calculated quaternion then rotation vector*/
//		eul2quatf(eul_temp,quatl);
//		quat_norm(quatl);
		
		//rotation vector update
		memcpy(quat,quatl,sizeof(quat));		
		memcpy(eul_rad,eul_temp,sizeof(eul_rad));

		rot_update(quatl,rot_vec);
		//rt_kprintf("\r\n%d",(int)(eul_temp[0]*1000));

		
		//send_data_imu(g_timediff_s,imu.acc_m_s2,imu.gyro_rps);
//		if(isnan(quatl[0])&&isnan(quatl[1])&&isnan(quatl[2])&&isnan(quatl[3]))
//			rt_kprintf("\r\ngx = %d",g_sensor_info.mpu.gx);

		//	rt_exit_critical();
	}
    //rt_thread_delay(30);// 20ms
	}
}
