#ifndef SERIAL_H
#define SERIAL_H

void delay(int sec);
int OpenDev(char *Dev);
void set_speed(int fd, int speed);
int set_Parity(int fd,int databits,int stopbits,int parity);

#endif

