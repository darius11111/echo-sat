#ifndef TRACKER_H_
#define TRACKER_H_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>

void send_g(const char *cmd, int serial);
void move(double incr, int serial);
void move2(double incr, int serial);

#endif // TRACKER_H_
