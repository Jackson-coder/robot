
#include "SerialPort.h"

/**
 * @brief write the data to the arm
 * 
 * @param fd the flag to check whether we recerive success
 * @param data the data to write
 * @return ssize_t the length of the data
 */
ssize_t SerialPort::write(int fd, datastruct data)
{
    ssize_t write_unm;
    tcflush(fd, TCOFLUSH); //清空，防止数据累积在缓存区
    write_unm = ::write(fd, &data, sizeof(&data));
    if (write_unm > 0)
    {
        printf("write success! \n");
    }
    else printf("write failed! \n");
    return write_unm;
}

/**
 * @brief read the data to the arm
 * 
 * @param fd the flag to check whether we recerive success
 * @param data the data to read
 * @return ssize_t the length of the data
 */
ssize_t SerialPort::read(int fd, datastruct data)
{
    ssize_t read_unm;
    read_unm = ::read(fd, &data, sizeof(&data));
    tcflush(fd, TCOFLUSH); //清空，防止数据累积在缓存区
    if (read_unm > 0)
    {
        printf("read success! \n ");
    }
    return read_unm;
}

/**
 * @brief the initial of the serial_port
 * 
 * @param fd 
 * @param usart_number
 * @return bool
 */
bool SerialPort::SerialPort_init(string usart_number)
{
    //打开串口
    fd = open(usart_number.c_str(), O_RDWR); //"/dev/ttyACM1"

    if (fd == -1)
    {
        printf("Wrong!\n"); /*perror(" 提示错误！");*/ /* 不能打开串口*/
        return false;
    }
    else
        printf("Open COM success!\n");

    datastruct data ;
    data.channel = 0x01;
    data.data_H = 0x02;
    data.data_L = 0x03;
    data.cmd = 0x04;
    
    SerialPort::write(fd, data);

    //设置串口
    struct termios option;
    tcgetattr(fd, &option);
    option.c_iflag = 0;                 //原始输入模式
    option.c_oflag = 0;                 //原始输出模式
    option.c_lflag = 0;                 //关闭终端模式
    option.c_cflag |= (CLOCAL | CREAD); //设置控制模式状态，本地连接，接收使能
    option.c_cflag &= ~CSIZE;           //字符长度，设置数据位之前一定要屏掉这个位
    option.c_cflag |= CS8;              //8位数据长度
    option.c_cflag &= ~CSTOPB;          //1位停止位
    option.c_cc[VTIME] = 0;             //最少字符和等待时间设置
    option.c_cc[VMIN] = 0;
    cfsetispeed(&option, B9600);     //设置波特率
    cfsetospeed(&option, B9600);     //设置波特率
    tcsetattr(fd, TCSANOW, &option); //立即生效

    return true;
}

void SerialPort::close()
{
    if (fd != -1)
        ::close(fd);
}