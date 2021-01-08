#include "Arm.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

bool Arm::Connect()
{
	if (port->SerialPort_init(port->usart_number))
	{
		cout << "Successfully connected." << endl;
		SetAngles(angles,0);
		return true;
	}
	else
	{
		cout << "Failed connection" << endl;
		return false;
	}
}

//initialize the arm
Arm::Arm(string port_number, double h, double xyz_init[3], double angle_limits_max[3], double angle_limits_min[3], double a_bias[3], double link[3])
{
	this->port_number = port_number;
	this->h = h;
	angles[0] = 30;
	angles[1] = 100;
	angles[2] = 90;
	this->a_bias[0] = a_bias[0];
	this->a_bias[1] = a_bias[1];
	this->a_bias[2] = a_bias[2];
	pos.x = xyz_init[0];
	pos.y = xyz_init[1];
	pos.z = xyz_init[2];
	this->link[0] = link[0];
	this->link[1] = link[1];
	this->link[2] = link[2];

	port->usart_number = port_number;

	for (int i = 0; i < 3; i++)
	{
		this->angle_limits_max[i] = angle_limits_max[i];
		this->angle_limits_min[i] = angle_limits_min[i];
	}
	// GetAnglesFromXYZ(pos);
	while (!Connect()) //未连接，自旋等待
	{
	}
}

Arm::~Arm()
{
}

void Arm::DisConnect()
{
	port->close();
}

/**
 * @brief move the motors to specific angles
 * 
 * @return true 
 * @return false  if the angles are out of the workspace
 */
bool Arm::IsInWorkspace()
{
	// if angles are out of range
	double angles0 = angles[0] * 180 / 3.1415927;
	double angles1 = angles[1] * 180 / 3.1415927;
	double angles2 = angles[2] * 180 / 3.1415927;
	cout << ' ' << angles0 << ' ' << angles1 << ' ' << angles2 << endl;
	cout << angle_limits_max[0] << ' ' << angle_limits_max[1] << ' ' << angle_limits_max[2] << angle_limits_min[0] << ' ' << angle_limits_min[1] << ' ' << angle_limits_min[2] << endl;
	if (
		(angles0 <= angle_limits_max[0]) && (angles0 >= angle_limits_min[0]) &&
		(angles1 <= angle_limits_max[1]) && (angles1 >= angle_limits_min[1]) &&
		(angles2 <= angle_limits_max[2]) && (angles2 >= angle_limits_min[2]))
	{
		return true;
	}
	else
	{
		return false;
	}
}
/**
 * @brief 得到转轴角度参数(几何解)
 * 
 * @param aim 目标空间点
 * @return true 该动作可行
 * @return false 该动作不可行
 */
bool Arm::GetAnglesFromXYZ(MyPoint3d aim)
{
	angles[2] = atan2(aim.y, aim.x); //alpha

	double AC2 = pow(aim.x, 2) + pow(aim.y, 2) + pow(aim.z, 2) + pow(link[0], 2) - 2 * link[0] * aim.z;
	angles[1] = acos((pow(link[2], 2) + pow(link[1], 2) - AC2) / (2 * link[1] * link[2])); //gama

	double T1 = pow(link[2], 2) + pow(link[1], 2) - 2 * link[1] * link[2] * cos(angles[1]);
	double T2 = 2 * link[2] * sqrt(pow(aim.x, 2) + pow(aim.y, 2)) * sin(angles[1]);
	double T3 = pow(link[1] - link[2] * cos(angles[1]), 2) - (pow(aim.x, 2) + pow(aim.y, 2));

	angles[0] = acos((T2 - sqrt(T2 * T2 + 4 * T1 * T3)) / (2 * T1));
	//cout << "angles:" << angles[0] << ' ';

	// cout<<asin(sqrt(pow(aim.x, 2) + pow(aim.y, 2)) / sqrt(AC2))<<endl;
	// angles[0] = (asin(sqrt(pow(aim.x, 2) + pow(aim.y, 2)) / sqrt(AC2)) < 1.57) ? abs(asin(sqrt(pow(aim.x, 2) + pow(aim.y, 2)) / sqrt(AC2)) - atan((link[2] * sin(angles[1])) / (link[1] - link[2] * cos(angles[1])))) : (3.14 - asin(sqrt(pow(aim.x, 2) + pow(aim.y, 2)) / sqrt(AC2)) - atan((link[2] * sin(angles[1])) / (link[1] - link[2] * cos(angles[1]))));
	angles[0] = abs(asin(sqrt(pow(aim.x, 2) + pow(aim.y, 2)) / sqrt(AC2)) - atan((link[2] * sin(angles[1])) / (link[1] - link[2] * cos(angles[1]))));

	// double T = pow(link[1],2)+pow(link[2]*cos(angles[]))

	cout << "angles:" << angles[0] << ' ' << angles[1] << ' ' << angles[2] << endl;

	if (IsInWorkspace())
	{
		angles[0] *= 180 / 3.1415927;
		angles[1] *= 180 / 3.1415927;
		angles[2] *= 180 / 3.1415927;
		pos.x = aim.x;
		pos.y = aim.y;
		pos.z = aim.z;
		cout << "here" << angles[0] << ' ' << angles[1] << ' ' << angles[2] << endl;
		return true;
	}
	else
	{
		return false;
	}
}

void Arm::MoveTo(MyPoint3d p)
{

	if (GetAnglesFromXYZ(p))
	{
		SetAngles(angles,0);
	}
}

/**
 * @brief set the angles, pay attention that the function is used behind the GetAngelsFromXYZ function
 * 
 * @return true 
 * @return false 
 */
bool Arm::SetAngles(double angles[3], bool flag = 0)
{
	//culculate pwm pulse width
	short pwm[3] = {0, 0, 0};

	//pwm pulse width(us)
	// angles[0] = 0;//上边正，下边负
	// angles[1] = 0;//上边负，下边正
	// angles[2] = 0;//左边负，右边正

	// pwm[0] = 1500 - ((angles[0] - a_bias[0]) * 2000 / 180);//beta
	// pwm[1] = 2500 - ((angles[1] - a_bias[1]) * 2000 / 180);//gamma
	// pwm[2] = 500 + ((angles[2] - a_bias[2]) * 2000 / 180);//alpha

	// int max_num = 90;
	// // int alpha=72,beta=33,gamma=57;
	// int beta=30,gamma=100,alpha=90;

	// angles[0] = beta;
	// angles[1] = gamma;
	// angles[2] = alpha;

	pwm[2] = 900 + ((angles[2] - a_bias[2]) * 1200 / 180);				//alpha
	pwm[0] = 1400 - ((angles[0] - a_bias[0]) * 2000 / 180);				//beta
	pwm[1] = 2300 - ((angles[1] - a_bias[1] - angles[0]) * 2000 / 180); //gamma

	cout << "pwm" << pwm[0] << ' ' << pwm[1] << ' ' << pwm[2] << endl;
	// pwm[0] = 1400; //  1400->0 900->45
	// pwm[1] = 1300;   //1800->45 1300->90
	// pwm[2] = 1500;   //900->0 1500->90 2100->180
	// cout << "pwm" << pwm[0] << ' ' << pwm[1] << ' ' << pwm[2] << endl;

	//convert datatype
	uint8_t tx_l[4] = {0, 0, 0};
	uint8_t tx_h[4] = {0, 0, 0};
	uint8_t a = 0, b = 0;
	for (int i = 0; i < 3; i++)
	{
		tx_h[i] = (pwm[i] >> 8) & 0xff;
		tx_l[i] = pwm[i] & 0xff;
	}

	//transmit to serial port
	// cout<<unsigned(tx_l[2])<<' '<<unsigned(tx_h[2])<<endl;

	if (flag == 0)
	{
		SendCmd(ARM_SET_ANGLE, ARM_CH_A2, tx_l[2], tx_h[2]);
		sleep(1);
		SendCmd(ARM_SET_ANGLE, ARM_CH_A0, tx_l[0], tx_h[0]);
		sleep(1);
		SendCmd(ARM_SET_ANGLE, ARM_CH_A1, tx_l[1], tx_h[1]);
		sleep(1);
	}
	else 
	{
		SendCmd(ARM_SET_ANGLE, ARM_CH_A1, tx_l[1], tx_h[1]);
		sleep(1);
		SendCmd(ARM_SET_ANGLE, ARM_CH_A0, tx_l[0], tx_h[0]);
		sleep(1);
		SendCmd(ARM_SET_ANGLE, ARM_CH_A2, tx_l[2], tx_h[2]);
		sleep(1);
	}

	// SendCmd(ARM_SET_ANGLE, ARM_CH_A0, tx_l[1], tx_h[1]);
	// sleep(1);

	// SendCmd(ARM_SET_ANGLE, ARM_SET_WAIT, a, b); ///////////////////////////////////////////////////
}

void Arm::MoveTo(int num)
{
	switch (num)
	{
	case 1:
		angles[2] = 80;
		angles[0] = 62;
		angles[1] = 116;
		break;
	case 2:
		angles[2] = 92;
		angles[0] = 58;
		angles[1] = 109;
		break;
	case 3:
		angles[2] = 106;
		angles[0] = 65;
		angles[1] = 112;
		break;
	case 4:
		angles[2] = 76;
		angles[0] = 45;
		angles[1] = 81;
		break;
	case 5:
		angles[2] = 91;
		angles[0] = 45;
		angles[1] = 80;
		break;
	case 6:
		angles[2] = 105;
		angles[0] = 45;
		angles[1] = 80;
		break;
	case 7:
		angles[2] = 72;
		angles[0] = 33;
		angles[1] = 57;
		break;
	case 8:
		angles[2] = 90;
		angles[0] = 30;
		angles[1] = 51;
		break;
	case 9:
		angles[2] = 111;
		angles[0] = 33;
		angles[1] = 56;
		break;
	case 0:
		angles[2] = 90;
		angles[0] = 30;
		angles[1] = 100;
		break;
	default:
		printf("error order!!!");
		break;
	}
	SetAngles(angles,0);

	angles[0]=30,angles[1]=100,angles[2]=90;
	SetAngles(angles,1);
}

void Arm::SendCmd(uint8_t cmd, uint8_t ch, uint8_t datal, uint8_t datah)
{
	// char ctx[5] = {0xff,cmd,ch,datal,datah};
	port->data.cmd = cmd;
	port->data.channel = ch;
	port->data.data_H = datah;
	port->data.data_L = datal;

	port->write(port->fd, port->data);
}
