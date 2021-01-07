#pragma once

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class camera
{
public:
    Mat picture;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    int flag[9] = {0};

    void find_light();
    int judge(Point2f circle);
    void record(int flag[9]);
};