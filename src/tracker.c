#include "tracker.h"

char buf[50];

void send_g(const char *cmd, int serial) {
   write(serial, cmd, strlen(cmd));
   usleep(20000);
}

char *concat(const char *s1, const char *s2) {
    char *res = malloc(strlen(s1) + strlen(s2) + 1); // +1 is null terminator
    strcpy(res, s1);
    strcpy(res, s2);
    return res;
}

void move(double incr, int serial) {
    sprintf(buf, "%f", incr);

    char *cmd = concat("G0 X", buf);
    char *cmd2 = concat(cmd, "\n");
    send_g(cmd2, serial);
}
