#include "camera.h"
#include <fstream>
#include <string>

int x1 = 0, x2 = 0, x3 = 0, x4 = 0, yy1 = 0, yy2 = 0, yy3 = 0, yy4 = 0, alpha = 10;
int max_num = 255;
void trackbar1(int, void *)
{
}
void trackbar2(int, void *)
{
}
void trackbar3(int, void *)
{
}
void trackbar4(int, void *)
{
}
void trackbar5(int, void *)
{
}
void trackbar6(int, void *)
{
}
void trackbar7(int, void *)
{
}
void trackbar8(int, void *)
{
}
void trackbar(int, void *)
{
}
/**
 * @brief 相机读入图片找灯
 * 
 * @param capture 
 * @return int* 
 */
void camera::find_light()
{
    Mat img = picture;
    cvtColor(picture, picture, COLOR_BGR2HSV);
    namedWindow("win");

    // createTrackbar("r1","win",&r1,max_num,trackbar1);
    // createTrackbar("r2","win",&r2,max_num,trackbar2);
    // createTrackbar("g1","win",&g1,max_num,trackbar3);
    // createTrackbar("g2","win",&g2,max_num,trackbar4);
    // createTrackbar("b1","win",&b1,max_num,trackbar5);
    // createTrackbar("b2","win",&b2,max_num,trackbar6);

    inRange(picture, Scalar(0, 0, 244), Scalar(255, 255, 255), picture);

    cv::Mat drawing = Mat::zeros(picture.size(), CV_8UC3);
    vector<Vec4i> hierarcy;
    Point2f center;

    vector<vector<Point>> contours;

    findContours(picture, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    vector<RotatedRect> rectanglement(contours.size());

    // createTrackbar("alpha", "win", &alpha, 250, trackbar);
    int area = 100; //最小面积
    // cout<<contours.size()<<endl;

    for (int i = 0; i < contours.size(); i++)
    {
        rectanglement[i] = minAreaRect(Mat(contours[i]));
        // cout<<rectanglement[i].size.width * rectanglement[i].size.height<<' '<<area<<endl;
        if (rectanglement[i].size.width * rectanglement[i].size.height > area)
            center = rectanglement[i].center;
        else
            continue;
        int f = judge(center);
        if (f != -1)
        {
            flag[f] = 1;
            cv::circle(img, center, 3, Scalar(50, 180, 180), 3);
            cout << center << endl;
        }
    }
    imshow("find_aim", img);
    return;
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
int camera::judge(Point2f center)
{
    // createTrackbar("x1","win",&x1,640,trackbar1);
    // createTrackbar("x2","win",&x2,640,trackbar2);
    // createTrackbar("x3","win",&x3,640,trackbar3);
    // createTrackbar("x4","win",&x4,640,trackbar4);
    // createTrackbar("y1","win",&yy1,480,trackbar5);
    // createTrackbar("y2","win",&yy2,480,trackbar6);
    // createTrackbar("y3","win",&yy3,480,trackbar7);
    // createTrackbar("y4","win",&yy4,480,trackbar8);
    int border_x[4] = {13, 218, 433, 625}; //高低指的是像素值
    int border_y[4] = {9, 164, 319, 471};

    // border_x[0] = x1;
    // border_x[1] = x2;
    // border_x[2] = x3;
    // border_x[3] = x4;
    // border_y[0] = yy1;
    // border_y[1] = yy2;
    // border_y[2] = yy3;
    // border_y[3] = yy4;

    float x = center.x;
    float y = center.y;
    if (x < border_x[0] || y < border_y[0] || x > border_x[3] || y > border_y[3])
    {
        //printf("-1");
        return -1;
    }
    else if (x < border_x[1] && y < border_y[1])
    {
        //printf("0");
        return 0;
    }
    else if (x > border_x[1] && x < border_x[2] && y < border_y[1])
    {
        //printf("1");
        return 1;
    }
    else if (x > border_x[2] && x < border_x[3] && y < border_y[1])
    {
        //printf("2");
        return 2;
    }
    else if (x < border_x[1] && y > border_y[1] && y < border_y[2])
    {
        //printf("3");
        return 3;
    }
    else if (x > border_x[1] && x < border_x[2] && y > border_y[1] && y < border_y[2])
    {
        //printf("4");
        return 4;
    }
    else if (x > border_x[2] && y > border_y[1] && y < border_y[2])
    {
        //printf("5");
        return 5;
    }
    else if (x < border_x[1] && y > border_y[2])
    {
        //printf("6");
        return 6;
    }
    else if (x > border_x[1] && x < border_x[2] && y > border_y[2])
    {
        //printf("7");
        return 7;
    }
    else if (x > border_x[2] && y > border_y[2])
    {
        //printf("8");
        return 8;
    }
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
