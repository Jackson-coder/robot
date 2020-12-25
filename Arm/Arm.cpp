#include "Arm.h"
using namespace std;

bool Arm::Connect()
{
	if (port->SerialPort_init(port->usart_number))
	{
		cout << "Successfully connected." << endl;
		SetAngles(angles);
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
	angles[0] = 0;
	angles[1] = 0;
	angles[2] = 0;
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
	GetAnglesFromXYZ(pos);
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
	cout<<' '<<angles0<<' '<<angles1<<' '<<angles2<<endl;
	cout<<angle_limits_max[0]<<' '<<angle_limits_max[1]<<' '<<angle_limits_max[2]<<
		angle_limits_min[0]<<' '<<angle_limits_min[1]<<' '<<angle_limits_min[2]<<endl;
	if (
		(angles0 <= angle_limits_max[0]) && (angles0 >= angle_limits_min[0]) &&
		(angles1 <= angle_limits_max[1]) && (angles1 >= angle_limits_min[1]) &&
		(angles2 <= angle_limits_max[2]) && (angles2 >= angle_limits_min[2])
		)
	{
		cout<<endl;
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

	cout << angles[0] << ' ' << angles[1] << ' ' << angles[2] << endl;

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
		SetAngles(angles);
	}
}

/**
 * @brief set the angles, pay attention that the function is used behind the GetAngelsFromXYZ function
 * 
 * @return true 
 * @return false 
 */
bool Arm::SetAngles(double angles[3])
{
	//culculate pwm pulse width
	short pwm[3] = {0, 0, 0};

	//pwm pulse width(us)
	cout<<angles[0]<<endl;
	// angles[0] = 0;//上边正，下边负
	// angles[1] = 0;//上边负，下边正
	// angles[2] = 0;//左边负，右边正
	
	pwm[0] = 1500 - ((angles[0] - a_bias[0]) * 2000 / 180);//beta
	pwm[1] = 2500 - ((angles[1] - a_bias[1]) * 2000 / 180);//gamma
	pwm[2] = 500 + ((angles[2] - a_bias[2]) * 2000 / 180);//alpha
	cout << pwm[0] << ' ' << pwm[1] << ' ' << pwm[2] << endl;

	//convert datatype
	uint8_t tx_l[4] = {0, 0, 0};
	uint8_t tx_h[4] = {0, 0, 0};
	uint8_t a=0,b=0;
	for (int i = 0; i < 3; i++)
	{
		tx_h[i] = (pwm[i] >> 8) & 0xff;
		tx_l[i] = pwm[i] & 0xff;
	}

	//transmit to serial port
	// cout<<unsigned(tx_l[2])<<' '<<unsigned(tx_h[2])<<endl;
	SendCmd(ARM_SET_ANGLE, ARM_CH_A0, tx_l[0], tx_h[0]);
	SendCmd(ARM_SET_ANGLE, ARM_CH_A1, tx_l[1], tx_h[1]);
	SendCmd(ARM_SET_ANGLE, ARM_CH_A2, tx_l[2], tx_h[2]);
	SendCmd(ARM_SET_ANGLE, ARM_SET_WAIT, a, b);///////////////////////////////////////////////////
}



//Set three angular Velocity
void Arm::SetAngularVel(uint8_t w[3])
{
	for (int i = 0; i < 3; i++)
	{
		if (w[i] > 10)
		{
			w[i] = 10;
		}
		else if (w[i] <= 0)
		{
			w[i] = 1;
		}
	}
	cout << "vel=" << w[0] << endl;
	SendCmd(ARM_SET_SPEED, ARM_CH_A0, w[0], 0x00);
	sleep(100);
	SendCmd(ARM_SET_SPEED, ARM_CH_A1, w[1], 0x00);
	sleep(100);
	SendCmd(ARM_SET_SPEED, ARM_CH_A2, w[2], 0x00);
	sleep(100);
}

//description: get a path from current position to thet destination.
//params: path(result), destination, quantity of points
//return: the path
bool Arm::GetPath(vector<MyPoint3d> &path, MyPoint3d dest, int num)
{
	//interpolation
	path.clear();
	path.resize(num + 1);
	double t = 1.0 / num;
	for (int i = 0; i <= num; i++)
	{
		double a[3] = {0, 0, 0};
		path[i].x = pos.x + t * i * (dest.x - pos.x);
		path[i].y = pos.y + t * i * (dest.y - pos.y);
		path[i].z = pos.z + t * i * (dest.z - pos.z);
		//if this point on the path is out of workspace
		if (!GetAnglesFromXYZ(path[i]))
		{
			return false;
		}
	}
	return true;
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