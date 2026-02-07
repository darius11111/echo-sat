#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <stdlib.h>

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

/* ---------- Antenna GPS position ---------- */
const double ANT_LAT = 50.225015998214474;
const double ANT_LON = 8.636638588785202;

/* ---------- Motor scaling ---------- */
#define STEPS_PER_DEG_Y 10.0  // motor steps per antenna degree
#define Y_GEAR_RATIO 5.0      // motor rotates 5x for 1° antenna yaw

/* ---------- Serial setup ---------- */
void setup_serial(int fd, speed_t baud) {
    struct termios tty;
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcsetattr(fd, TCSANOW, &tty);
}

/* ---------- G-code sender ---------- */
void send_cmd(int fd, const char *cmd) {
    write(fd, cmd, strlen(cmd));
    tcdrain(fd);
    usleep(150000);
}

/* ---------- Packet parser ---------- */
int parse_packet(const char *line, double *lat, double *lon) {
    const char *p;

    p = strstr(line, "LAT=");
    if (!p) return 0;
    if (strncmp(p + 4, "None", 4) == 0) return 0;
    *lat = atof(p + 4);

    p = strstr(line, "LON=");
    if (!p) return 0;
    if (strncmp(p + 4, "None", 4) == 0) return 0;
    *lon = atof(p + 4);

    return 1;
}

int main(void) {
    /* ---------- Open receiver ---------- */
    int rx_fd = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY | O_SYNC);
    if (rx_fd < 0) {
        perror("open /dev/ttyACM0");
        return 1;
    }

    /* ---------- Open motor controller ---------- */
    int motor_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (motor_fd < 0) {
        perror("open /dev/ttyUSB0");
        close(rx_fd);
        return 1;
    }

    setup_serial(rx_fd, B115200);
    setup_serial(motor_fd, B115200);

    /* ---------- Wake controller ---------- */
    write(motor_fd, "\r\n\r\n", 4);
    usleep(2000000);
    tcflush(motor_fd, TCIFLUSH);

    send_cmd(motor_fd, "G90\n");   // absolute mode
    send_cmd(motor_fd, "G21\n");   // mm units
    send_cmd(motor_fd, "G1 F1000\n");

    /* ---------- Read a single packet ---------- */
    char buf[256];
    int idx = 0;
    double lat = 0.0, lon = 0.0;

    while (1) {
        char c;
        if (read(rx_fd, &c, 1) == 1) {
            if (c == '\n' || c == '\r') {
                if (idx == 0)
                    continue;

                buf[idx] = '\0';
                idx = 0;

                if (parse_packet(buf, &lat, &lon)) {
                    printf("TARGET → LAT=%.6f LON=%.6f\n", lat, lon);
                    break;  // exit loop after first valid GPS
                } else {
                    printf("NO GPS → %s\n", buf);
                }
            } else if (idx < (int)sizeof(buf) - 1) {
                buf[idx++] = c;
            }
        }
    }

    /* ---------- Compute azimuth (yaw) ---------- */
    double lat1 = DEG2RAD(ANT_LAT);
    double lat2 = DEG2RAD(lat);
    double dlon = DEG2RAD(lon - ANT_LON);

    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlon);

    double az = atan2(y, x);
    az = fmod(az + 2*M_PI, 2*M_PI);

    /* ---------- Convert to Y motor units with gear ratio ---------- */
    double Y = RAD2DEG(az) * STEPS_PER_DEG_Y * Y_GEAR_RATIO;

    char gcode[128];
    snprintf(gcode, sizeof(gcode),
             "G1 Y%.2f\n", Y);

    send_cmd(motor_fd, gcode);

    printf("ANTENNA POINTED → Y motor = %.2f\n", Y);
    printf("%f", Y);

    close(rx_fd);
    close(motor_fd);
    return 0;
}
