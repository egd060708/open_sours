# 基于stm32f407zgt6的机甲车辆

### 总体介绍

​		华南理工大学2022无限机甲杯23-大佬说得队比赛车辆

​		电控负责人：张至睿  张华铨

​		本车以stm32f407为主控，PS2手柄为遥控，结合直流减速电机、数字舵机、树莓派实现全向运动以及自动瞄准射击两大功能

### 电控硬件布置

​		车上有三种主要的用电大户分别是：直流减速电机、直流摩擦轮电机、舵机

​		为了合理分配两个支路的功耗，并且方便布线，支路一主要给车身运动机构供电，包括底盘电机、云台舵机、爬楼梯舵机，并且考虑到舵机控制需要共地的问题，把主板的供电也加入支路一。而支路二主要负责云台上自瞄发射机构的供电，包括摩擦轮电机、拨盘电机以及树莓派。

[open_sours/机甲硬件布置.png at main · egd060708/open_sours (github.com)](https://github.com/egd060708/open_sours/blob/main/机甲硬件布置.png)

​		同时，自主设计的降压模块也为芯片提供了稳定的供电。

### 电控软件

​		**本工程主要使用了cubemx+hal库对单片机进行底层配置，用keil5进行代码的编写和程序的生成**

#### 1、代码的封装与规范 

​		本次工程运用面向对象的思想，用结构体对变量进行整合封装，用结构体指针对函数进行封装，提高代码的简洁度和可移植性。

例如：

```c
typedef struct
{
	int PS2_x;
	int PS2_y;
	
	int PS2_left;
	int PS2_right;
	int PS2_up;
	int PS2_down;
} PS2_Rocker;//与PS2摇杆有关的结构体变量

extern PS2_Rocker Rocker_Right_side;
extern PS2_Rocker Rocker_Left_side;
```

​		此外，本次工程也大量使用宏定义，让固定参数修改更方便快捷，让代码可读性更强。

​		在**driver_init**相关文件中对工程底层配置进行了初始化，大量宏定义的作用得到体现。

#### 2、PS2手柄通信

​		PS2手柄通过SPI通信协议实现与stm32的数据传输

​		此部分主要参考了博客http://t.csdn.cn/uCsi0

​		需要注意的是，由于PS2手柄存在两种模式，车辆用到的主要是红灯模拟量模式，而手柄在与接收器连接阶段默认为绿灯模式，若此时单片机接收数据不做处理，则会出现底盘乱跑的情况，因此，对数据接收的部分改进如下：

```c
void PS2_Receive (void)
{
		PS2_LX=PS2_AnologData(PSS_LX);//模拟量
		PS2_LY=PS2_AnologData(PSS_LY);
		PS2_RX=PS2_AnologData(PSS_RX);
		PS2_RY=PS2_AnologData(PSS_RY);
		PS2_KEY=PS2_DataKey();//检测按键值
	if(Data[1]==0x73)//红灯模式判定
	{
		if(PS2_KEY>=5&&PS2_KEY<=8)//此程序中部分按键与遥杆功能重复，需要隔离判断
		{
			if(PS2_KEY==5)//对部分键值进行特别限制
				Rocker_Left_side.PS2_up =5;
			else if(PS2_KEY==6)
				Rocker_Left_side.PS2_right=6;
			else if(PS2_KEY==7)
				Rocker_Left_side.PS2_down =7;
			else if(PS2_KEY==8)
				Rocker_Left_side.PS2_left =8;
		}
		else
		{
			Rocker_Left_side.PS2_up =0;
			Rocker_Left_side.PS2_right =0;
			Rocker_Left_side.PS2_down =0;
			Rocker_Left_side.PS2_left =0;
		}
		Rocker_Left_side.PS2_x=PS2_LX;//读取摇杆模拟量
		Rocker_Left_side.PS2_y=PS2_LY;
		Rocker_Right_side.PS2_x=PS2_RX;
		Rocker_Right_side.PS2_y=PS2_RY;
	}
}
```

​		由此，最大程度地减少了红绿灯模式以及按键和摇杆之间的数据冲突。

#### 3、PID算法的运用

​		本次工程使用了速度单环PID作为底盘电机的控制，使用速度位置双环PID作为拨盘电机的控制，均取得不错的效果。

​		实现PID有几个过程，**首先要实现对电机速度和位置的检测**

```c
int Read_Encoder(int Moto_Select)//读取编码器数值
{
	int Encoder_TIM;
	switch(Moto_Select)
	{
		case Wheel_LT: 
						Encoder_TIM=TIM8->CNT; //读取计数
						if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; 
            			//转化计数值为有方向的值，大于0正转，小于0反转。
						TIM8->CNT=0; //读取完后计数清零
						break;
		case Wheel_RT: 
						Encoder_TIM=TIM3->CNT; //读取计数
						if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; 
            			//转化计数值为有方向的值，大于0正转，小于0反转。
						TIM3->CNT=0; //读取完后计数清零
						break;
		case Wheel_LB: 
						Encoder_TIM=TIM4->CNT; //读取计数
						if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; 
            			//转化计数值为有方向的值，大于0正转，小于0反转。
						TIM4->CNT=0; //读取完后计数清零
						break;
		case Wheel_RB: 
						Encoder_TIM=TIM5->CNT; //读取计数
						if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; 
            			//转化计数值为有方向的值，大于0正转，小于0反转。
						TIM5->CNT=0; //读取完后计数清零
						break;
		case Turntable:
//					if(TIM2->CNT<10000)
//					{
//						Update_times--;
//						__HAL_TIM_SetCounter(&htim2,50000);
//					}
//					else if(TIM2->CNT>50000)
//					{
//						Update_times++;
//						__HAL_TIM_SetCounter(&htim2,10000);
//					}
//					
//					LastPosition2=CurrentPosition2;//先取上一次得出的位置
//		
//					CurrentPosition2=(TIM2->CNT-10000)+Update_times*50000;
//			
//					Encoder_TIM=CurrentPosition2-LastPosition2; //读取计数
						Encoder_TIM=TIM2->CNT; //读取计数
						if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; 
            			//转化计数值为有方向的值，大于0正转，小于0反转。
						TIM2->CNT=0; //读取完后计数清零
						break;
		//default: Encoder_TIM=0;
	}
	return Encoder_TIM; //返回值
}
```

​		**检测有两种方式**

​		第一种是在速度检测的基础上累计得到位置值，这种方法优点是逻辑较简单，稳定性高，但不足的是容易出现编码器读数遗漏的情况，导致位置控制精度下降。

​		第二种是在位置检测的基础上得到速度，这种方法理论上不会漏读可以实现更高精度的控制，但也更易受到不稳定因素的影响，比如定时器溢出中断无法进入导致计数混乱等问题。为了系统稳定性，暂且使用第一种方法，日后会继续改进。



​		实现速度位置检测之后，就是**反馈给PID控制器进行运算**了

```c
//速度PID计算函数//
int Velocity_FeedbackControl(Moto_pid *moto,int Moto_Select)//速度PID控制
{
		int Fact_Velocity=0;
		if(Rocker_Left_side.PS2_x ==128&&Rocker_Left_side.PS2_y ==127&&PS2_KEY==0&&Moto_Select!=Turntable)
		{
			moto->Control_Velocity=0;
		}//消除电机死区积分
		Fact_Velocity=moto->Target_Velocity*(moto->Moto_Characteristic)/6000;//输入目标值速度为rpm值,每10ms访问一次，一分钟访问6000次，一次要达到的转速为rpm/6000，每圈累计读数为68
		moto->Bias=Fact_Velocity-moto->Encoder; //求速度偏差
		                                                                                                                                                                                                     
		moto->Control_Velocity+=moto->Moto_Velocity_Kp*(moto->Bias-moto->Last1_Bias)
								+moto->Moto_Velocity_Ki* moto->Bias;
								//+(moto->Moto_Velocity_Kd)*(moto->Bias-2*(moto->Last1_Bias)+(moto->Last2_Bias));  //增量式PI控制器
                //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	            //Velcity_Ki*Bias速度控制值由Bias不断积分得到 偏差越大加速度越大
		//moto->Last2_Bias=moto->Last1_Bias;
		moto->Last1_Bias=moto->Bias;
//		if(moto->Control_Velocity>8400)//输出限制
//			moto->Control_Velocity=8400;
//		if(moto->Control_Velocity<-8400)
//			moto->Control_Velocity=-8400;
		return moto->Control_Velocity; //返回速度控制值
}

//位置PID计算函数//
int Position_FeedbackControl(Moto_pid *moto)//位置PID控制
{
		float FactPosition;//定义相关变量
		const float P=1.0;
		
		FactPosition=(moto->Target_Position/360.) * moto->Moto_Characteristic * P; //目标位置=目标圈数*1040
	                            //10ms读取一次编码器(即100HZ)，电机减速比为30，霍尔编码器精度13，AB双相组合得到4倍频，
	                            //则转1圈编码器读数为30*13*4=1560，电机转速=Encoder*100/1560r/s 使用定时器2
	                            //1.04是误差系数，电机本身存在误差，可根据实际情况调整该系数以提高控制精度
		moto->Bias=FactPosition - moto->Current_Position; //求位置偏差
		moto->Integral_Bias += moto->Bias;
		if(moto->Integral_Bias> 970) moto->Integral_Bias= 970;	//积分限幅 防止到达目标位置后过冲
		if(moto->Integral_Bias<-970) moto->Integral_Bias=-970;	//积分限幅 防止到达目标位置后过冲
	
		moto->Control_Velocity=moto->Moto_Posision_Kp *moto->Bias+moto->Moto_Posision_Ki/100*moto->Integral_Bias+moto->Moto_Posision_Kd*(moto->Bias-moto->Last1_Bias);  //位置式PI控制器
	                                                                                            //Position_Kp*Bias 偏差越大速度越大
	                                                                                            //Position_Ki*Integral_Bias 减小稳态误差
	                                                                                            //Position_Kd*(Bias-Last_bias) 限制速度
		moto->Last1_Bias=moto->Bias;	
		return moto->Control_Velocity;    //返回速度控制值 
}

//速度限幅函数，对位置输出的PWM加以速度限幅,用于双环速度与位置PID的转换//
int Velocity_Restrict(int PWM_P, int TargetVelocity)
{
		if     (PWM_P>+TargetVelocity*76) PWM_P=+TargetVelocity*76;
		else if(PWM_P<-TargetVelocity*76) PWM_P=-TargetVelocity*76;
		else PWM_P=PWM_P;
		return PWM_P;
}
```

​		不同模式的PID控制器主要是通过这几个函数之间的组合搭配实现PID控制

​		PID控制器输出的PWM可以直接写入定时器寄存器中，对电机动力进行调整

​		把PID控制器放入定时器中定时执行，不断改变动力输出，达到要求的效果

​		

#### 4、底盘运动解算

​		底盘使用麦克纳母轮实现全向运动

~~~c
#define wheel_length 0.114//（m）车长
#define wheel_width 0.08//（m）车宽
#define wheel_r 0.04//（m）车轮半径
#define PI 3.14//圆周率
#define vtx_max 0.5   //左右最大速度
#define vty_max 1.    //前后最大速度
#define w_max PI / 1  //自转最大速度

typedef struct
{
		float vtx;//左右速度
		float vty;//前后速度
		float w;//自转角速度
		
		float vRT;//四轮速度
	  float vLT;
	  float vLB;
	  float vRB;
} wheel_target_v;

wheel_target_v Target_V;
~~~

​		将PS2传输的信号分别解算为小车的前后平移、左右平移、自转速度

~~~c
//从ps2得到小车目标速度
void PS2_to_car_v(PS2_Rocker *PS2_LeftSide, wheel_target_v *target_v)
{
	if (PS2_LeftSide->PS2_left == 0 && PS2_LeftSide->PS2_right == 0)//按键自转
	{
		target_v->w = 0;
	}
	else if (PS2_LeftSide->PS2_left == 8)
	{
		target_v->w = w_max;
	}
	else if (PS2_LeftSide->PS2_right == 6)
	{
		target_v->w = -w_max;
	}

	if (PS2_LeftSide->PS2_up == 0 && PS2_LeftSide->PS2_down == 0)//按键前后
	{
		target_v->vtx = (PS2_LeftSide->PS2_x - 128)*vtx_max / 128;
		target_v->vty = (127 - PS2_LeftSide->PS2_y)*vty_max / 127;
	}
	else if (PS2_LeftSide->PS2_up == 5)//按键前后
	{
		target_v->vty = vty_max;
	}
	else if (PS2_LeftSide->PS2_down == 7)//按键左右
	{
		target_v->vty = -vty_max;
	}
}
~~~

​		将小车的目标运动速度解算为四个麦轮各自的速度

~~~c
//由小车速度解算为轮子速度
void car_v_to_wheel_v(wheel_target_v *target_v)
{
	target_v->vRT = -target_v->vtx + target_v->vty + target_v->w*(wheel_length + wheel_width);
	target_v->vLT = +target_v->vtx + target_v->vty - target_v->w*(wheel_length + wheel_width);
	target_v->vLB = -target_v->vtx + target_v->vty - target_v->w*(wheel_length + wheel_width);
	target_v->vRB = +target_v->vtx + target_v->vty + target_v->w*(wheel_length + wheel_width);
}
~~~

​		将四个麦轮的速度分别解算为各自电机的转速

~~~c
//由轮子速度解算为电机转速
void wheel_v_to_Moto_target_v(wheel_target_v *target_v)
{
	Moto_wheel_RT.Target_Velocity = target_v->vRT * 60 / (2 * PI*wheel_r);
	Moto_wheel_LT.Target_Velocity = target_v->vLT * 60 / (2 * PI*wheel_r);
	Moto_wheel_LB.Target_Velocity = target_v->vLB * 60 / (2 * PI*wheel_r);
	Moto_wheel_RB.Target_Velocity = target_v->vRB * 60 / (2 * PI*wheel_r);
}
~~~

​		直接调用执行函数

~~~c
//执行函数，输入PS2指令，输出电机目标转速（r/min）
void PS2_to_Moto_target_v(PS2_Rocker *PS2_LeftSide , wheel_target_v *target_v)
{
	PS2_to_car_v(PS2_LeftSide,target_v);
	car_v_to_wheel_v(target_v);
	wheel_v_to_Moto_target_v(target_v);
}
~~~

​		将解算得到的电机目标转速存入对应结构变量中，由PID控制器进行调用输出

#### 5、舵机运动解算

​		本次工程使用舵机进行云台的转向平台，一方面是简化了控制逻辑，另一方面也是简化了机械设计难度，虽控制方式比较粗糙，但也不失为自瞄入门的一个很好的平台。

​		同时，舵机还作为爬楼梯装置的动力来源。肩负车辆中两个重要功能的实现，舵机的控制不可谓不重要。

​		首先是云台舵机的控制：

```c
#define Timer_Frequency 100.0//频率为100hz
#define Control_Arr 9600//可用pwm极差
#define SE_X 1
#define SE_Y 2
#define PitchAngle 30//俯仰角最大值
#define YawAngle 135//航向角最大值
#define X_Lock_PWM 7200//设定云台初始位置，以便快速复位
#define Y_Lock_PWM 7400
#define SE_X_Angle_Range 270.0//舵机极限角度
#define SE_Y_Angle_Range 180.0
typedef struct
{
	int 	PWM;
	int 	Max_Angle_Velocity;//每秒最大角速度
	int 	Target;//视觉传输横滚角
	int 	Integral_Target;
	int 	Last_Target;
	double 	SE_Kp;
	double 	SE_Ki;
	double 	SE_Kd;
}SE_xy_Drive;//云台电机结构体
```

​		第一个函数用于手柄控制云台的全方向转动

​		通过手柄传入的模拟量，控制PWM增量的变化，在10ms一次循环的定时器中，实现**任意角速度**的舵机转动

```C
void PS2_to_Cradle(int SE_Select,PS2_Rocker *Rocker_value)//手柄控制云台旋转
{
	double PWM_Change=0;//角速度下每秒钟pwm的增量
	switch(SE_Select)
	{
		case SE_X:
				PWM_Change = (SE_x.Max_Angle_Velocity/Timer_Frequency)*(Control_Arr/SE_X_Angle_Range );
            	//解算每秒钟PWM的增量第一个括号是每个周期要转的角度，第二个括号是每转一度对应的PWM增量
				SE_x.PWM += -(PWM_Change /128)*(Rocker_value->PS2_x-128);
            	//把最大转速对应PWM增量映射到遥杆对应数据上
				if(SE_x.PWM>(int)(X_Lock_PWM+YawAngle*Control_Arr/SE_X_Angle_Range))
						SE_x.PWM = (int)(X_Lock_PWM+YawAngle*Control_Arr/SE_X_Angle_Range);
				else if(SE_x.PWM<(int)(X_Lock_PWM-YawAngle*Control_Arr/SE_X_Angle_Range))
						SE_x.PWM = (int)(X_Lock_PWM-YawAngle*Control_Arr/SE_X_Angle_Range);
				//云台角度限幅
				__HAL_TIM_SET_COMPARE(&htim9 ,TIM_CHANNEL_1 ,SE_x.PWM);
				break;
            
		case SE_Y:
				PWM_Change = (SE_y.Max_Angle_Velocity/Timer_Frequency)*(Control_Arr/SE_Y_Angle_Range );
            	//解算每秒钟PWM的增量第一个括号是每个周期要转的角度，第二个括号是每转一度对应的PWM增量
				SE_y.PWM += -(PWM_Change /128)*(Rocker_value->PS2_y-127);
            	//把最大转速对应PWM增量映射到遥杆对应数据上
				if(SE_y.PWM>(int)(Y_Lock_PWM+PitchAngle*Control_Arr/SE_Y_Angle_Range))
						SE_y.PWM = (int)(Y_Lock_PWM+PitchAngle*Control_Arr/SE_Y_Angle_Range);
				else if(SE_y.PWM<(int)(Y_Lock_PWM-PitchAngle*Control_Arr/SE_Y_Angle_Range))
						SE_y.PWM = (int)(Y_Lock_PWM-PitchAngle*Control_Arr/SE_Y_Angle_Range);
				//云台角度限幅
				__HAL_TIM_SET_COMPARE(&htim9 ,TIM_CHANNEL_2 ,SE_y.PWM);
				break;
	}
}
```

​		第二个函数用于云台舵机进行自动瞄准

​		入口参数为视觉方面通过树莓派+摄像头解算出的误差角度，使用增量式PID的思想，对云台舵机进行控制，简称**视觉PID**

```c
void Linux_to_Cradle(int SE_Select)//自瞄函数
{
	double PWM_Change;//固定转角下PWM的变化
	switch(SE_Select)
	{
		case SE_X:
					SE_x.Target=vision_target.yaw_ref;//偏航角，视觉变量直接视为误差值
		
					SE_x.Integral_Target+=SE_x.Target;//误差积分
		
					PWM_Change=SE_x.SE_Kp*SE_x.Target*Control_Arr/SE_X_Angle_Range+SE_x.SE_Ki 
								*SE_x.Integral_Target*Control_Arr/SE_X_Angle_Range +SE_x.SE_Kd 
								*(SE_x.Target-SE_x.Last_Target)*Control_Arr/SE_X_Angle_Range ;
					//PID控制器
					SE_x.PWM += (int)PWM_Change;//增量输出
		
					if(SE_x.PWM>(int)(X_Lock_PWM+YawAngle*Control_Arr/SE_X_Angle_Range))
						SE_x.PWM = (int)(X_Lock_PWM+YawAngle*Control_Arr/SE_X_Angle_Range);
					else if(SE_x.PWM<(int)(X_Lock_PWM-YawAngle*Control_Arr/SE_X_Angle_Range))
						SE_x.PWM = (int)(X_Lock_PWM-YawAngle*Control_Arr/SE_X_Angle_Range);
					//位置限幅
					__HAL_TIM_SET_COMPARE(&htim9 ,TIM_CHANNEL_1 ,SE_x.PWM);
		
					break;
		case SE_Y:
					SE_y.Target=vision_target.pit_ref;//俯仰角
		
					SE_y.Integral_Target+=SE_y.Target;
		
					PWM_Change=SE_y.SE_Kp*SE_y.Target*Control_Arr/SE_Y_Angle_Range+SE_y.SE_Ki 
								*SE_y.Integral_Target*Control_Arr/SE_Y_Angle_Range +SE_y.SE_Kd 
								*(SE_y.Target-SE_y.Last_Target)*Control_Arr/SE_Y_Angle_Range;
					
					SE_y.PWM += (int)PWM_Change;
					
					if(SE_y.PWM>(int)(Y_Lock_PWM+PitchAngle*Control_Arr/SE_Y_Angle_Range))
						SE_y.PWM = (int)(Y_Lock_PWM+PitchAngle*Control_Arr/SE_Y_Angle_Range);
					else if(SE_y.PWM<(int)(Y_Lock_PWM-PitchAngle*Control_Arr/SE_Y_Angle_Range))
						SE_y.PWM = (int)(Y_Lock_PWM-PitchAngle*Control_Arr/SE_Y_Angle_Range);
					//位置限幅
					__HAL_TIM_SET_COMPARE(&htim9 ,TIM_CHANNEL_2 ,SE_y.PWM);
		
					break;
	}

}

```

​		爬楼梯舵机只要实现稳定正反转即可，控制较简单，就不再过多赘述



#### 6、stm32与树莓派的串口通信数据传输

​		STM32通过串口通信的方式以**16进制方式收发数组**与树莓派实现通信

​		相关变量

~~~c
//串口宏定义
#define huart_nonPC huart3
#define UART_nonPC USART3
#define huart_PC huart1
#define UART_PC USART1

//串口发送数据定义
uint8_t Tx_art0[1]={0xA1};

//串口接收数据定义
#define uart_send_num 8
uint8_t Rx_dat[uart_send_num];//0为开始，1为偏航角符号，2为偏航角整数，3为偏航角小数，4为俯仰角符号，5为俯仰角整数，6为俯仰角小数，7为结束
uint8_t is_into_usart_callback=0;//判断是否在回调函数执行过程中
double 	yaw_ref_calc=0;//偏航角计算中间变量
double 	pit_ref_calc=0;//俯仰角计算中间变量

//结构体定义
typedef struct
{
	double yaw_ref;//偏航角
  double pit_ref;//俯仰角
} Vision_Target;
Vision_Target vision_target;
~~~

通过PS2按键向树莓派发送开始发送数据的指令

~~~c
switch(PS2_KEY)
			{
				case PSB_GREEN://向树莓派发开始指令
					if(key_up )
					{
						key_up =0;
						HAL_UART_Transmit(&huart_nonPC,Tx_art0,sizeof(Tx_art0),10000);//发送指令
						is_into_usart_callback=0;
						vision_target.yaw_ref=0;
						vision_target.pit_ref =0;
					}
					break;
~~~

通过定时中断的方式使串口保持接收数据的状态，将接收到的数据存入数组

~~~c
if(is_into_usart_callback == 0)
		{
			HAL_UART_Receive_IT(&huart_nonPC,Rx_dat,uart_send_num);//开始下一次接收
		}
~~~

对数组中的数据进行解算，解算后得到的俯仰角与偏航角放入结构变量中等待云台执行

~~~c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART_nonPC)//如果是串口3
	{
		is_into_usart_callback=1;//防止被下一个回调函数打断
		
		if(Rx_dat[0]==0xBF&&Rx_dat[uart_send_num-1] == 0xFB)//校验
		{
			double Rx_dat2=Rx_dat[3],Rx_dat4=Rx_dat[6];//小数位转换为double类型变量
		
			while(Rx_dat2>=1)
			{
					Rx_dat2*=0.1;//偏航角小数转换
			}
			yaw_ref_calc=Rx_dat[2]+Rx_dat2;//偏航角赋值
			if(Rx_dat[1]==1)//偏航角正负判断
			{
				yaw_ref_calc*=-1;
			}
						
			while(Rx_dat4>=1)
			{
					Rx_dat4*=0.1;//俯仰角小数转换
			}
			pit_ref_calc=Rx_dat[5]+Rx_dat4;//俯仰角赋值
			if(Rx_dat[4]==1)//俯仰角正负判断
			{
				pit_ref_calc*=-1;
			}
		}
	}
	
	vision_target.yaw_ref=yaw_ref_calc;
	vision_target.pit_ref=pit_ref_calc;//传入结构变量中
	is_into_usart_callback=0;//回调函数执行完毕
	HAL_UART_Receive_IT(&huart_PC,Rx_dat,uart_send_num);//开始下一次接收
}
~~~

**遇到的问题与解决方案：**

**1、问题：**通信一开始是在串口接收回调函数的最后让串口开始下一次接收，后来发现有时会出现根本没有进入回调函数的现象，导致串口不会再接收下一次的数据；

**判断：**可能是被更高优先级的中断打断；

**解决：**通过定时中断的方式让串口保持接收数据的状态，并通过一个判断变量使回调函数在执行的过程中不会被下一个回调函数打断；





