
#include "serial/SerialPort.h"
#include "Arm/Arm.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define BLACK -1
#define WHITE 1


int main()
{
	FileStorage fs("/home/robot/parameter.yaml", cv::FileStorage::READ);
	SerialPort port;
	string usart_number;
	double h;
	double xyz_init[3], angle_limits_max[3], angle_limits_min[3], a_bias[3];
	fs["usart_number"] >> usart_number;
	fs["h"] >> h;
	fs["pos_x_init"] >> xyz_init[0];
	fs["pos_y_init"] >> xyz_init[1];
	fs["pos_z_init"] >> xyz_init[2];
	fs["alpha_limits_max"] >> angle_limits_max[0];
	fs["belta_limits_max"] >> angle_limits_max[1];
	fs["gama_limits_max"] >> angle_limits_max[2];
	fs["alpha_limits_min"] >> angle_limits_min[0];
	fs["belta_limits_min"] >> angle_limits_min[1];
	fs["gama_limits_min"] >> angle_limits_min[2];
	fs["a_bias_0"] >> a_bias[0];
	fs["a_bias_1"] >> a_bias[1];
	fs["a_bias_2"] >> a_bias[2];

	while (!port.SerialPort_init(usart_number))
	{
		cout<< a_bias[0] <<endl;
		cout << "SerialPort_init_failed, please retry!" << endl;
	}

	// Arm arm(usart_number, h, xyz_init, angle_limits_max, angle_limits_min, a_bias);

	// while (1)
	// {
	// 	arm.MoveTo(p);
	// 	arm.SetAngularVel(w[3]);
	// }
	
	
	// arm.DisConnect();

	fs.release();
return 0;
}
