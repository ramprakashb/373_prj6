
//C program to call the driver file
#include<stdio.h>
#include<stdint.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<errno.h>
#include<stdlib.h>
#include<string.h>

#define LED0_ON 0xe
#define LED0_OFF 0x4e
#define LED1_ON 0xe00
#define LED1_OFF 0x4e00
#define LED2_ON 0xe0000
#define LED2_OFF 0x4e0000
#define LED3_ON 0xe000000
#define LED3_OFF 0x4e000000
#define LEDS_OFF (LED0_OFF | LED1_OFF | LED2_OFF | LED3_OFF)
#define BUF_SIZE 256

//extern int errno;

int main()
{
	int fd,i;
	ssize_t read_ret, write_ret;
	uint32_t data_read=0;
	int data_write;
	uint32_t buf[BUF_SIZE];
	int sys_call_val = 0;

	//Open the PCI driver
	fd= open("/dev/ece_led",O_RDWR);
	if(fd==-1)
	{
		perror("fd: error is as follows ");

		return 0;
	}
	printf("open=%d\n",fd);
	sleep(2);
	//Read the value from driver
	read_ret = read(fd,&data_read,sizeof(data_read));
/*	if(read_ret < 0){
		perror("error on read\n");
		return -1;
	}


	sys_call_val = buf[0];
	
	printf("blink rate =%d per second\n",sys_call_val);	

	for(i=0;i<10;i++)
		printf("looop=%d\n", i);
	printf("Enter everyting to close:\n");
	if(close(fd) == -1){
		perror("error on close");
		return -1;
	}
*/
	close(fd);	
	return 0;
}
