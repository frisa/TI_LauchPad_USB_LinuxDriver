/*
 * main.c
 *
 *  Created on: Jun 25, 2016
 *      Author: root
 */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main()
{
    int fd;
    int cmd = 0x02;
    fd = open("/dev/xbdev1", O_RDWR);
    write(fd, &cmd, 1);
    myclose(fd);

	printf("Load the module to the kernel\n");
	return 0;
}

