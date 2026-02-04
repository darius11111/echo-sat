#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <termios.h>

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

/* Antenna GPS position */
const double ANT_LAT = 50.225015998214474;
const double ANT_LON = 8.636638588785202;
const double ANT_ALT = 160.0;

/* Motor scaling */
#define STEPS_PER_DEG_X 10.0
#define STEPS_PER_DEG_Y 10.0

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

void send_cmd(int fd, const char *cmd) {
    write(fd, cmd, strlen(cmd));
    tcdrain(fd);
    usleep(200000);
}

int main(void) {
    /* ---------- Open receiver ---------- */
    int rx_fd = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY | O_SYNC);
    if (rx_fd < 0) {
        perror("open ttyACM0");
        return 1;
    }

    /* ---------- Open motors ---------- */
    int motor_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (motor_fd < 0) {
        perror("open ttyUSB0");
        close(rx_fd);
        return 1;
    }

    /* ---------- Configure serial ---------- */
    setup_serial(rx_fd, B115200);     // adjust if needed
    setup_serial(motor_fd, B115200);  // adjust if needed

    /* Wake motor controller */
    write(motor_fd, "\r\n\r\n", 4);
    usleep(2000000);
    tcflush(motor_fd, TCIFLUSH);

    send_cmd(motor_fd, "G90\n");
    send_cmd(motor_fd, "G21\n");
    send_cmd(motor_fd, "G1 F1000\n");

    /* ---------- Read → Track → Move ---------- */
    char buf[256];
    int idx = 0;

    while (1) {
        char c;
        if (read(rx_fd, &c, 1) == 1) {
            if (c == '\n') {
                buf[idx] = '\0';
                idx = 0;

                double alt, lat, lon;
                if (sscanf(buf, "%lf,%lf,%lf", &alt, &lat, &lon) == 3) {
                    /* --- Compute angles (same as before) --- */
                    double lat1 = DEG2RAD(ANT_LAT);
                    double lat2 = DEG2RAD(lat);
                    double dlon = DEG2RAD(lon - ANT_LON);

                    double y = sin(dlon) * cos(lat2);
                    double x = cos(lat1)*sin(lat2) -
                               sin(lat1)*cos(lat2)*cos(dlon);

                    double az = atan2(y, x);
                    az = fmod(az + 2*M_PI, 2*M_PI);

                    double R = 6371000.0;
                    double dlat = lat2 - lat1;
                    double a = sin(dlat/2)*sin(dlat/2) +
                               cos(lat1)*cos(lat2)*
                               sin(dlon/2)*sin(dlon/2);

                    double cdist = 2 * atan2(sqrt(a), sqrt(1-a));
                    double ground = R * cdist;

                    double el = atan2(alt - ANT_ALT, ground);

                    double X = RAD2DEG(az) * STEPS_PER_DEG_X;
                    double Y = RAD2DEG(el) * STEPS_PER_DEG_Y;

                    char gcode[128];
                    snprintf(gcode, sizeof(gcode),
                             "G1 X%.2f Y%.2f\n", X, Y);
                    send_cmd(motor_fd, gcode);
                }
            } else if (idx < (int)sizeof(buf)-1) {
                buf[idx++] = c;
            }
        }
    }
}
