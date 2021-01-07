
// #include "serial/SerialPort.h"
// #include "Arm/Arm.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera/camera.h"
// #include <unistd.h>

using namespace cv;
using namespace std;

int main()
{
	FileStorage fs("/home/robot/parameter.yaml", cv::FileStorage::READ);
	// SerialPort port;
	string usart_number;
	double h;
	double xyz_init[3], angle_limits_max[3], angle_limits_min[3], a_bias[3], link[3];

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
	fs["link0"] >> link[0];
	fs["link1"] >> link[1];
	fs["link2"] >> link[2];

	VideoCapture capture(0);
	namedWindow("img");
	while (1)
	{
		camera cam;
		capture >> cam.picture;
		if (cam.picture.empty())
		{
			printf("打开失败\n");
			continue;
		}
		imshow("img", cam.picture);
		cam.find_light();

		for (int i = 0; i < 9; i++)
		{
			printf("%d",cam.flag[i]);
				
		}

		waitKey(30);
	}

	// Arm arm(usart_number, h, xyz_init, angle_limits_max, angle_limits_min, a_bias, link);

	// MyPoint3d p;
	// p.x=3;
	// p.y=20;
	// p.z=10;

	// arm.MoveTo(p);

	fs.release();
	return 0;
}
