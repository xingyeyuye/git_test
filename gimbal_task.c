#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "user_lib.h"
#include "INS_task.h"
#include "main.h"

//gimbal control data
//云台控制所有相关数据
gimbal_control_t gimbal_control;
gimbal_control_t shoot_control;
gimbal_control_t chassis_control;
//yaw编码器相对角度
const fp32 gimbal_yaw_relative_pid_constants[3]={YAW_ENCODE_RELATIVE_PID_KP,YAW_ENCODE_RELATIVE_PID_KI,YAW_ENCODE_RELATIVE_PID_KD};
const fp32 gimbal_yaw_speed_pid_constants[3]={YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD};
//yaw陀螺仪绝对角度
const fp32 gimbal_yaw_absolute_angle_pid_constants[3]={YAW_ABSOLUTE_ANGLE_PID_KP,YAW_ABSOLUTE_ANGLE_PID_KI,YAW_ABSOLUTE_ANGLE_PID_KD};
const fp32 gimbal_yaw_absolute_speed_pid_constants[3]={YAW_ABSOLUTE_SPEED_PID_KP,YAW_ABSOLUTE_SPEED_PID_KI,YAW_ABSOLUTE_SPEED_PID_KD};
//yaw陀螺仪角速度
const fp32 gimbal_yaw_absolute_gyro_pid_constants[3]={YAW_GYRO_ABSOLUTE_PID_KP,YAW_GYRO_ABSOLUTE_PID_KI,YAW_GYRO_ABSOLUTE_PID_KD};

const fp32 chassis_yaw_relative_pid_constants[3]={CHASSIS_ENCODE_RELATIVE_PID_KP,CHASSIS_ENCODE_RELATIVE_PID_KI,CHASSIS_ENCODE_RELATIVE_PID_KD};

const fp32 shoot_yaw_relative_pid_constants[3]={SHOOT_ENCODE_RELATIVE_PID_KP,SHOOT_ENCODE_RELATIVE_PID_KI,SHOOT_ENCODE_RELATIVE_PID_KD};
const fp32 shoot_speed_pid_constants[3]={SHOOT_SPEED_PID_KP,SHOOT_SPEED_PID_KI,SHOOT_SPEED_PID_KD};

const fp32 gimbal_pitch_relative_pid_constants[3]={PITCH_ENCODE_RELATIVE_PID_KP,PITCH_ENCODE_RELATIVE_PID_KI,PITCH_ENCODE_RELATIVE_PID_KD};
const fp32 gimbal_pitch_speed_pid_constants[3]={PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD};
 
//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
double msp(double x, double in_min, double in_max, double out_min, double out_max);
float lowPassFilter(float alpha, float prevOutput, float input);

fp32 my_fabs(fp32 x);
//fp32 get_yaw_target_angle(void);
fp32 get_pitch_relative_target_angle(void);
fp32 get_yaw_relative_target_angle(void);
fp32 get_yaw_target_INS_angle(fp32 INS_angle);
//拨盘3508电机
fp32 get_shoot_relative_target_angle(void);
fp32 shoot_relative_set_angle;
fp32 shoot_target_angle;
//yaw轴编码器角度
fp32 yaw_relative_target_angle;
fp32 yaw_relative_set_angle;
fp32 yaw_relative_last_set_angle;
fp32 yaw_relative_delta_angle;
fp32 yaw_relative_current_angle;
//pitch轴编码器角度
fp32 pitch_relative_target_angle;
fp32 pitch_relative_set_angle;
fp32 pitch_relative_last_set_angle;
fp32 pitch_relative_delta_angle;
fp32 pitch_relative_current_angle;
//yaw轴陀螺仪角速度
fp32 yaw_target_BMI088_gyro;
fp32 yaw_last_target_BMI088_gyro;
fp32 yaw_set_BMI088_gyro;
fp32 yaw_last_set_BMI088_gyro;
fp32 yaw_delta_BMI088_gyro;
fp32 yaw_current_BMI088_gyro[3];
//yaw轴陀螺仪角度
fp32 yaw_target_INS_angle;
fp32 yaw_set_INS_angle;
fp32 yaw_last_set_INS_angle;
fp32 yaw_delta_INS_angle;
fp32 yaw_current_INS_angle[3];
fp32 yaw_now_INS_angle;

fp32 relative_wz;
//
int target1,target2,actual1,actual2,out1,out2;
//
fp32 last_yaw_gyro_out;
fp32 last_yaw_BMI088_gyro_out;

void Gimbal_task(void const *pvParameters)
{
	gimbal_control.gimbal_rc_ctrl=get_remote_control_point();
	gimbal_control.gimbal_yaw_motor.gimbal_motor_measure=get_yaw_gimbal_motor_measure_point();
	gimbal_control.gimbal_pitch_motor.gimbal_motor_measure=get_pitch_gimbal_motor_measure_point();
	shoot_control.shoot_motor.gimbal_motor_measure=get_trigger_motor_measure_point();
	//获取角速度
	gimbal_control.gimbal_INT_gyro_point=get_gyro_data_point();
	//获取角度
	gimbal_control.gimbal_INT_angle_point=get_INS_angle_point();
	
	//拨盘电机
	PID_init(&shoot_control.shoot_motor.gimbal_motor_relative_angle_pid,PID_POSITION,shoot_yaw_relative_pid_constants,SHOOT_ENCODE_RELATIVE_PID_MAX_OUT,SHOOT_ENCODE_RELATIVE_PID_MAX_IOUT);
	PID_init(&shoot_control.shoot_motor.gimbal_motor_gyro_pid,PID_POSITION,shoot_speed_pid_constants,SHOOT_SPEED_PID_MAX_OUT,SHOOT_SPEED_PID_MAX_IOUT);
	//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
	PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid,PID_POSITION,gimbal_yaw_relative_pid_constants,YAW_ENCODE_RELATIVE_PID_MAX_OUT,YAW_ENCODE_RELATIVE_PID_MAX_IOUT);
	PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid,PID_POSITION,gimbal_pitch_speed_pid_constants,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
	//底盘电机
	PID_init(&chassis_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid,PID_POSITION,chassis_yaw_relative_pid_constants,CHASSIS_ENCODE_RELATIVE_PID_MAX_OUT,CHASSIS_ENCODE_RELATIVE_PID_MAX_IOUT);
	//yaw 角速度环 角度由陀螺仪 PID参数以及 PID最大输出，积分输出
	PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid,PID_POSITION,gimbal_yaw_absolute_angle_pid_constants,YAW_ABSOLUTE_ANGLE_PID_MAX_OUT,YAW_ABSOLUTE_ANGLE_PID_MAX_IOUT);
	PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_BMI088_gyro_pid,PID_POSITION,gimbal_yaw_absolute_gyro_pid_constants,YAW_GYRO_ABSOLUTE_PID_MAX_OUT,YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
		
	//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
	PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid,PID_POSITION,gimbal_pitch_relative_pid_constants,PITCH_ENCODE_RELATIVE_PID_MAX_OUT,PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);
	PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid,PID_POSITION,gimbal_yaw_speed_pid_constants,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT);
		
	while(1)
	{	
//		gimbal_control.gimbal_yaw_motor.absolute_angle=*(INS_angle_gimbal+INS_YAW_ADDRESS_OFFSET)*180/PI;
//		gimbal_control.gimbal_pitch_motor.absolute_angle=*(INS_angle_gimbal+INS_ROLL_ADDRESS_OFFSET)*180/PI;
	
	 	//底盘跟随云台，设置底盘的旋转量
		if(gimbal_control.gimbal_rc_ctrl->rc.s[0]==3)
			{
				PID_calc(&chassis_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid,gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd,7100);
				relative_wz=chassis_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.out*7;
			}
			
			//发送串口显示滤波
		target1=yaw_target_INS_angle;
		actual1=yaw_current_INS_angle[0];
		out1=yaw_target_BMI088_gyro;
		
		yaw_relative_current_angle=gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
		//计算云台的目标值
		if(gimbal_control.gimbal_rc_ctrl->rc.s[1]==3&&gimbal_control.gimbal_rc_ctrl->rc.s[0]!=1){
			//计算云台pitch轴的目标值
			pitch_relative_target_angle=get_pitch_relative_target_angle();
			
			pitch_relative_current_angle=gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
			PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid,pitch_relative_current_angle,pitch_relative_target_angle);
			PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid,gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->speed_rpm,gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.out);
			
			//计算云台yaw轴的目标值
			yaw_relative_target_angle=get_yaw_relative_target_angle();
			
				//计算最短旋转路径
			if(yaw_relative_target_angle-gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd<(-4096.0f))
			yaw_relative_current_angle=gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd-8191.0f;//6020电机一圈是0~8191
			if(yaw_relative_target_angle-gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd>(4096.0f))
			yaw_relative_current_angle=gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd+8191.0f;//6020电机一圈是0~8191
	
			PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid,yaw_relative_current_angle,yaw_relative_target_angle);
			PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid,gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->speed_rpm,gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.out);	
			//一阶低通滤波
			gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.out=lowPassFilter(0.1,last_yaw_gyro_out,gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.out);
			CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.out,gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.out, 0, 0);
			last_yaw_gyro_out=gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.out;
			yaw_now_INS_angle=yaw_current_INS_angle[0];
		}
		//拨盘电机
		if(gimbal_control.gimbal_rc_ctrl->rc.s[1]==1&&gimbal_control.gimbal_rc_ctrl->rc.s[0]!=1)
		{
			//计算云台pitch轴的目标值
			pitch_relative_target_angle=get_pitch_relative_target_angle();
			
			pitch_relative_current_angle=gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
			PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid,pitch_relative_current_angle,pitch_relative_target_angle);
			PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid,gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->speed_rpm,gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.out);
			
			//计算云台yaw轴的目标值
			yaw_relative_target_angle=get_yaw_relative_target_angle();
			
				//计算最短旋转路径
			if(yaw_relative_target_angle-gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd<(-4096.0f))
			yaw_relative_current_angle=gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd-8191.0f;//6020电机一圈是0~8191
			if(yaw_relative_target_angle-gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd>(4096.0f))
			yaw_relative_current_angle=gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd+8191.0f;//6020电机一圈是0~8191
	
			PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid,yaw_relative_current_angle,yaw_relative_target_angle);
			PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid,gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->speed_rpm,gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.out);	
			//一阶低通滤波
			gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.out=lowPassFilter(0.1,last_yaw_gyro_out,gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.out);
			
			//拨盘3508电机
			shoot_target_angle=get_shoot_relative_target_angle();
			PID_calc(&shoot_control.shoot_motor.gimbal_motor_relative_angle_pid,shoot_control.shoot_motor.gimbal_motor_measure->ecd,shoot_target_angle);
			PID_calc(&shoot_control.shoot_motor.gimbal_motor_gyro_pid,shoot_control.shoot_motor.gimbal_motor_measure->speed_rpm,shoot_control.shoot_motor.gimbal_motor_relative_angle_pid.out);

			CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.out,gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.out, shoot_control.shoot_motor.gimbal_motor_gyro_pid.out, 0);
			last_yaw_gyro_out=gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.out;
		}
		//获取云台角度当前值
		yaw_current_INS_angle[0]=*(gimbal_control.gimbal_INT_angle_point+INS_YAW_ADDRESS_OFFSET)*180/PI;
		if(yaw_current_INS_angle[0]<0)
		{
			yaw_current_INS_angle[0]=msp(yaw_current_INS_angle[0],-180,-1,180,359);//陀螺仪反馈角度一圈是0~180~-180~0，此行代码将-180~-1映射到181~360.
		}
		yaw_current_INS_angle[1]=*(gimbal_control.gimbal_INT_angle_point+INS_PITCH_ADDRESS_OFFSET)*180/PI;
		yaw_current_INS_angle[2]=*(gimbal_control.gimbal_INT_angle_point+INS_ROLL_ADDRESS_OFFSET)*180/PI;
		//获取云台角速度当前值
		yaw_current_BMI088_gyro[0]=*(gimbal_control.gimbal_INT_gyro_point+INS_GYRO_X_ADDRESS_OFFSET);
		yaw_current_BMI088_gyro[1]=*(gimbal_control.gimbal_INT_gyro_point+INS_GYRO_Y_ADDRESS_OFFSET);
		yaw_current_BMI088_gyro[2]=*(gimbal_control.gimbal_INT_gyro_point+INS_GYRO_Z_ADDRESS_OFFSET);//yaw轴角速度
		//小陀螺下的云台计算
		if(gimbal_control.gimbal_rc_ctrl->rc.s[0]==1)
		{
			//计算云台pitch轴的目标值
			pitch_relative_target_angle=get_pitch_relative_target_angle();
			
			pitch_relative_current_angle=gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
			PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid,pitch_relative_current_angle,pitch_relative_target_angle);
			PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid,gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->speed_rpm,gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.out);
			
			//计算云台yaw轴的目标值
			yaw_target_INS_angle=get_yaw_target_INS_angle(yaw_now_INS_angle);
			if(yaw_target_INS_angle>360)
			{
				yaw_target_INS_angle=0.0f;
			}
			if(yaw_target_INS_angle<0)
			{
				yaw_target_INS_angle=360.0f;
			}
			//计算最短旋转路径
			if(yaw_target_INS_angle-yaw_current_INS_angle[0]<(-180.0f))
			yaw_current_INS_angle[0]=yaw_current_INS_angle[0]-360.0f;//陀螺仪一圈是0~360
			if(yaw_target_INS_angle-yaw_current_INS_angle[0]>(180.0f))
			yaw_current_INS_angle[0]=yaw_current_INS_angle[0]+360.0f;//陀螺仪一圈是0~360
			
			PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid,yaw_current_INS_angle[0],yaw_target_INS_angle);
			yaw_target_BMI088_gyro=gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.out+1000*(0-yaw_current_BMI088_gyro[2]);
			
//			PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_BMI088_gyro_pid,yaw_current_BMI088_gyro[2],yaw_target_BMI088_gyro);
//			yaw_last_target_BMI088_gyro=yaw_target_BMI088_gyro;
			//一阶低通滤波
//			gimbal_control.gimbal_yaw_motor.gimbal_motor_BMI088_gyro_pid.out=lowPassFilter(0.9,last_yaw_BMI088_gyro_out,gimbal_control.gimbal_yaw_motor.gimbal_motor_BMI088_gyro_pid.out);
//			CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.gimbal_motor_BMI088_gyro_pid.out,gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.out, 0, 0);
			CAN_cmd_gimbal(yaw_target_BMI088_gyro,gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.out, 0, 0);
//			last_yaw_BMI088_gyro_out=gimbal_control.gimbal_yaw_motor.gimbal_motor_BMI088_gyro_pid.out;
			yaw_now_INS_angle=yaw_target_INS_angle;
		}
		//清除PID
		else if(gimbal_control.gimbal_rc_ctrl->rc.s[1]==2&&gimbal_control.gimbal_rc_ctrl->rc.s[0]!=1)
		{
			PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid);
			PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
			PID_clear(&chassis_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid);
			PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
			PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);
			PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_INS_speed_pid);
			PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_BMI088_gyro_pid);
			PID_clear(&shoot_control.shoot_motor.gimbal_motor_relative_angle_pid);
			PID_clear(&shoot_control.shoot_motor.gimbal_motor_gyro_pid);
			CAN_cmd_gimbal(0,0,0,0);
		}
		osDelay(20);
	}
	
	
}

// 一阶低通滤波器更新函数
float lowPassFilter(float alpha, float prevOutput, float input) {
    return alpha * input + (1.0f - alpha) * prevOutput;
}

fp32 my_fabs(fp32 x) {
    return (x < 0) ? -x : x;
}

//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
double msp(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

//fp32 get_yaw_target_angle(void)
//{
//	yaw_set_angle = 180*(msp(gimbal_control.gimbal_rc_ctrl->rc.ch[2],-660,660,-PI,PI))/PI;
//		yaw_last_set_angle=yaw_set_angle;
//		if((my_fabs(yaw_last_set_angle)-my_fabs(yaw_set_angle)>0)||yaw_set_angle==0)
//		{
//			yaw_delta_angle=0;
//		
//		}
//		else if(yaw_set_angle>0)
//		{
//			yaw_delta_angle=10;
//		}
//		else if(yaw_set_angle<0)
//		{
//			yaw_delta_angle=-10;
//		}
//	return	(yaw_delta_angle+gimbal_control.gimbal_yaw_motor.absolute_angle);
//}
//根据云台编码器计算云台目标值
fp32 get_yaw_relative_target_angle(void)
{
	yaw_relative_set_angle = msp(gimbal_control.gimbal_rc_ctrl->rc.ch[2],-660,660,-4096,4096);
		yaw_relative_last_set_angle=yaw_relative_set_angle;
		if((my_fabs(yaw_relative_last_set_angle)-my_fabs(yaw_relative_set_angle)>0)||yaw_relative_set_angle==0)
		{
			yaw_relative_delta_angle=0;
		
		}
		else if(yaw_relative_set_angle>0)
		{
			yaw_relative_delta_angle-=100;
		}
		else if(yaw_relative_set_angle<0)
		{
			yaw_relative_delta_angle+=100;
		}
		if(yaw_relative_delta_angle>800)yaw_relative_delta_angle=800;
		if(yaw_relative_delta_angle<-800)yaw_relative_delta_angle=-800;
	return	(yaw_relative_delta_angle+7100);
}

fp32 get_pitch_relative_target_angle(void)
{
	pitch_relative_set_angle = msp(gimbal_control.gimbal_rc_ctrl->rc.ch[3],-660,660,-4096,4096);
		pitch_relative_last_set_angle=pitch_relative_set_angle;
		if((my_fabs(pitch_relative_last_set_angle)-my_fabs(pitch_relative_set_angle)>0)||pitch_relative_set_angle==0)
		{
			pitch_relative_delta_angle=0;
		
		}
		else if(pitch_relative_set_angle>0)
		{
			pitch_relative_delta_angle+=50;
		}
		else if(pitch_relative_set_angle<0)
		{
			pitch_relative_delta_angle-=50;
		}
		if(pitch_relative_delta_angle>270)pitch_relative_delta_angle=270;
		if(pitch_relative_delta_angle<-270)pitch_relative_delta_angle=-270;
	return	(pitch_relative_delta_angle+6810);//6810
}
//拨盘3508电机
fp32 get_shoot_relative_target_angle(void)
{
	if(gimbal_control.gimbal_rc_ctrl->rc.ch[4]>0)
	{
		shoot_relative_set_angle=1365.0f;
	}
	else if(gimbal_control.gimbal_rc_ctrl->rc.ch[4]<0)
	{
		shoot_relative_set_angle=-1365.0f;
	}
	else
	{
		shoot_relative_set_angle=0.0f;
	}
	return shoot_relative_set_angle+shoot_control.shoot_motor.gimbal_motor_measure->ecd;
}

fp32 get_yaw_target_INS_angle(fp32 INS_angle)
{
	if(gimbal_control.gimbal_rc_ctrl->rc.ch[2]>0)
	{
		yaw_set_INS_angle+=1.0f;
	}
	else if(gimbal_control.gimbal_rc_ctrl->rc.ch[2]<0)
	{
		yaw_set_INS_angle-=1.0f;
	}
	else
	{
		yaw_set_INS_angle=0.0f;
	}
	return (yaw_set_INS_angle+INS_angle);
}
