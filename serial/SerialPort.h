#pragma once

#include <iostream>
#include <stdio.h>     /*标准输入输出定义*/
#include <stdlib.h>    /*标准函数库定义*/
#include <unistd.h>    /*Unix 标准函数定义*/
#include <sys/types.h> /*数据类型，比如一些XXX_t的那种*/
#include <sys/stat.h>  /*定义了一些返回值的结构，没看明白*/
#include <fcntl.h>     /*文件控制定义*/
#include <termios.h>   /*PPSIX 终端控制定义*/
#include <string.h>
#include <memory>
#include <string>
using namespace std;

/**
 * @brief the serial datastruct
 * 
 */
struct datastruct
{
    uint8_t flag = 0xff;
    uint8_t cmd; //0x01:设置舵机速度; 0x02:设置舵机位置； 0x09:启动动作组；0x0b:急停/恢复
    uint8_t channel;
    uint8_t data_L;
    uint8_t data_H;
};

//eg
//急停
// data.cmd = 0x0b;
// data.channel = 0x00;
// data.data_L = 0x01;
// data.data_H = 0x00;

class SerialPort
{
public:
    string usart_number;
    SerialPort()
    {

    };
    ~SerialPort()
    {
        
    };

    bool SerialPort_init(string usart_number);
    ssize_t write(int fd, datastruct data);
    ssize_t read(int fd, datastruct data);
    void close();

    datastruct data;
    int fd;
};
