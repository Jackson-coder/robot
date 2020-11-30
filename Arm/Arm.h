#pragma once
#include <vector>
// #include <conio.h>
#include <cmath>
#include <iostream>
#include <time.h>

#include "/home/robot/serial/SerialPort.h"
using namespace std;

struct MyPoint3d
{
	double x;
	double y;
	double z;
};


//9 grids of chessboard


//where the chesspieces are stored
//Point3d chesspiece = { 0,0,0 };

//Point3d wait = { 0,0,0 };
//Point3d stop = { 0,0,0 };

#define ARM_SET_SPEED 0X01
#define ARM_SET_ANGLE 0X02
#define ARM_MOMENT_SERIES 0X09
#define ARM_EMERGENCY_STOP 0X0B
#define ARM_CH_A0 0X09
#define ARM_CH_A1 0X01
#define ARM_CH_A2 0X02
#define ARM_CH_PUMP 0X03
#define ARM_CH_VALVE 0X04

class Arm
{
private:
	// double l[5];
	double link[5];//0:OA,1:AB,2:BC
	double h;
	double angles[3];//目标角度:0:alpha,1:belta,2:gama
	double angle_limits_max[3];
	double angle_limits_min[3];	
	double theta;
	string port_number;
	SerialPort* port;
public:
	double a_bias[3];
	MyPoint3d pos;//当前位置
	
	//Initialize
	Arm(string port_number,double l[5], double h, double xyz_init[3], double angle_limits_max[3], double angle_limits_min[3],double a_bias[3]);
	~Arm();
	bool Connect();
	void DisConnect();

	//compute angles from coordinate of the manipulator and move it
	bool GetAnglesFromXYZ(MyPoint3d aim);
	// bool GetAnglesFromXYZ(double* result, double x, double y, double z);

	//move the motors to specific angles
	bool SetAngles(MyPoint3d aim);

	//
	void SetAngularVel(char w[3]);

	//Get a path from linear interpolation
	bool GetPath(vector<MyPoint3d>& path, MyPoint3d dest, int num);
	
	//judge whether an angle series is in workspace 
	bool IsInWorkspace();

	void SendCmd(char cmd, char ch, char datal, char datah);

	//Send Command to the Controller and move the Arm
	void MoveTo(MyPoint3d p);

};

