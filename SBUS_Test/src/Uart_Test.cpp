//============================================================================
// Name        : Uart_Test.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

using namespace std;

unsigned char reverse(unsigned char b)
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        //if (tcsetattr (fd, TCSANOW, &tty) != 0)
                //error_message ("error %d setting term attributes", errno);
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                cout << "Error from tcgetattr: " << errno << endl;
                return -1;
        }

        //cfsetospeed (&tty, speed);
        //cfsetispeed (&tty, speed);
        tty.c_ispeed = speed;
        tty.c_ospeed = speed;

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= (PARENB);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                cout << "error from tcsetattr: " << errno << endl;
                return -1;
        }
        return 0;
}


int main()
{
	//char portname = "/dev/ttyACM0";
	char buf[25];
	char temp;
	int fd = open ("/dev/ttyACM0", O_RDWR);
	int count = 0;
	int prev_count = 0;
	int index = 0;
	int _channels[18];
	bool start_byte = false;
	bool end_byte = false;

	if (fd < 0)
	{
		cout << "Error opening port: " << errno << endl;
		return -1;
	}
	set_interface_attribs (fd, 100000, 1);
	set_blocking(fd,0);
	while(1)
	{
		int n = read (fd, &temp, 1);
		count++;
		if(temp == 0x0f && ((count - prev_count) == 1))
		{
			start_byte = true;
		}
		if(temp == 0x00)
		{
			end_byte = true;
			prev_count = count;
		}
		if(start_byte && end_byte)
		{
			buf[index] = temp;
			temp = 0;
			index++;
		}
		if(index == 25)
		{
			for(int i = 0; i < 25 ; i++)
			{
				//buf[i] = reverse(buf[i]);
				printf("%02x ", buf[i] & 0xff);
			}
			//write (fd, buf, 25);
//			usleep(80000);
			printf("\n");
			index = 0;
			count = 0;
			start_byte = false;
			end_byte = false;
			_channels[0]  = ((buf[1]    |buf[2]<<8)                 & 0x07FF);
			_channels[1]  = ((buf[2]>>3 |buf[3]<<5)                 & 0x07FF);
			_channels[2]  = ((buf[3]>>6 |buf[4]<<2 |buf[5]<<10)  & 0x07FF);
			_channels[3]  = ((buf[5]>>1 |buf[6]<<7)                 & 0x07FF);
			_channels[4]  = ((buf[6]>>4 |buf[7]<<4)                 & 0x07FF);
			_channels[5]  = ((buf[7]>>7 |buf[8]<<1 |buf[9]<<9)   & 0x07FF);
			_channels[6]  = ((buf[9]>>2 |buf[10]<<6)                & 0x07FF);
			_channels[7]  = ((buf[10]>>5|buf[11]<<3)                & 0x07FF);
			_channels[8]  = ((buf[12]   |buf[13]<<8)                & 0x07FF);
			_channels[9]  = ((buf[13]>>3|buf[14]<<5)                & 0x07FF);
			_channels[10] = ((buf[14]>>6|buf[15]<<2|buf[16]<<10) & 0x07FF);
			_channels[11] = ((buf[16]>>1|buf[17]<<7)                & 0x07FF);
			_channels[12] = ((buf[17]>>4|buf[18]<<4)                & 0x07FF);
			_channels[13] = ((buf[18]>>7|buf[19]<<1|buf[20]<<9)  & 0x07FF);
			_channels[14] = ((buf[20]>>2|buf[21]<<6)                & 0x07FF);
			_channels[15] = ((buf[21]>>5|buf[22]<<3)                & 0x07FF);
			for (int i = 0 ; i < 16 ; i++)
			{
				printf("%5d ",_channels[i]);
			}
			printf("\n");
		}
	}
}
