#include "camera.h"
#include <fstream>
#include <string>

/**
 * @brief 相机读入图片找灯
 * 
 * @param capture 
 * @return int* 
 */
int *camera::find_light(VideoCapture capture)
{
    cv::Mat picture;
    capture >> picture;
    cvtColor(picture, picture, COLOR_BGR2HSV);
    inRange(picture, Scalar(80, 50, 140), Scalar(130, 255, 255), picture);

    cv::Mat drawing = Mat::zeros(picture.size(), CV_8UC3);
    vector<Vec4i> hierarcy;
    Point2f circle;

    vector<vector<Point>> contours;

    findContours(picture, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    vector<RotatedRect> rectanglement(contours.size());
    int flag[9];
    for (auto f : flag)
    {
        f = 0;
    }

    int area = 10; //最小面积

    for (int i = 0; i < contours.size(); i++)
    {
        rectanglement[i] = minAreaRect(Mat(contours[i]));
        if (rectanglement[i].size.width * rectanglement[i].size.height < area)
            circle = rectanglement[i].center;
        int f = judge(circle);
        if (f != -1)
            flag[f] = 1;
    }

    return flag;
}

//灯序如下：
//0  1  2
//3  4  5
//6  7  8

/**
 * @brief 判断灯的位置
 * 
 * @param circle 
 * @return int 
 */
int camera::judge(Point2f circle)
{
    int border_x[4] = {1, 2, 3, 4}; //高低指的是像素值
    int border_y[4] = {1, 2, 3, 4};
    float x = circle.x;
    float y = circle.y;
    if (x < border_x[0] || y < border_y[0] || x > border_x[3] || y > border_y[3])
        return -1;
    else if (x < border_x[1] && y < border_y[1])
        return 0;
    else if (x > border_x[1] && x < border_x[2] && y < border_y[1])
        return 1;
    else if (x > border_x[2] && x < border_x[3] && y < border_y[1])
        return 2;
    else if (x < border_x[1] && y > border_y[1] && y < border_y[2])
        return 3;
    else if (x > border_x[1] && x < border_x[2] && y > border_y[1] && y < border_y[2])
        return 4;
    else if (x > border_x[2] && y > border_y[1] && y < border_y[2])
        return 5;
    else if (x < border_x[1] && y > border_y[2])
        return 6;
    else if (x > border_x[1] && x < border_x[2] && y > border_y[2])
        return 7;
    else if (x > border_x[2] && y > border_y[2])
        return 8;
}

/**
 * @brief 记录识别结果
 * 
 * @param flag 
 */
void camera::record(int flag[9])
{
    ofstream file("file.txt");

    if (file.fail())
    {
        cout << "warning:write file 'file.txt' failed!" << endl;
    }

    for (int i = 0; i < 9; i++)
    {
        if (flag[i] == 1)
            file << '1';
        else
            file << '0';

        if (i != 8)
            file << ',';
    }

    file.close();
}
