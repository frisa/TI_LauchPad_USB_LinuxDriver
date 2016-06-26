/*
 ============================================================================
 Name        : consapp.c
 Author      : root
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    int fd;
    int arr[3] = {2,8,4};
    int idx = 0;
    int cmd = 0;
    fd = open("/dev/xbdev1", O_RDWR);
    while (1)
    {
    	cmd = arr[idx++];
    	write(fd, &cmd, 1);
    	printf("[TI] Send the value = %d \n", cmd);
    	usleep(100000);
    	if (3 == idx)
    	{
    		idx = 0;
    	}
    }
    close(fd);
	return EXIT_SUCCESS;
}
