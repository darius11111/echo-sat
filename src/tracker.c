#include "tracker.h"

char buf[50];

void send_g(const char *cmd, int serial) {
   write(serial, cmd, strlen(cmd));
   usleep(20000);
}

char *concat(const char *s1, const char *s2) {
    char *res = malloc(strlen(s1) + strlen(s2) + 1); // +1 is null terminator
    if (!res) return NULL;

    strcpy(res, s1);
    strcat(res, s2);
    return res;
}

void move(double incr, int serial) {
    sprintf(buf, "%.1f", incr);

    char *cmd = concat("G0 X", buf);
    char *cmd2 = concat(cmd, "\n");
    printf("%s", cmd2);
    send_g(cmd2, serial);

    free(cmd);
    free(cmd2);
}

void move2(double incr, int serial) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "G0 X%.1f\n", incr);

    printf("%s", cmd);
    send_g(cmd, serial);
}
