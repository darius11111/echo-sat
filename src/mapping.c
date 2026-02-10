#include <stdio.h>
#include <math.h>
#include "tracker.h"

#define RAD_K M_PI/180.0f
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

double lon1, lat1;

double calculate_azimuth(double lat1, double lon1, double lat2, double lon2) {

    // deg conversion to pirads
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double delta_lambda = (lon2 - lon1) * DEG_TO_RAD;

    // az calc
    double x = sin(delta_lambda) * cos(phi2);
    double y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda);

    double azimuth = atan2(x, y);

    azimuth *= RAD_TO_DEG;
    /*
    if (azimuth < 0.0)
        azimuth += 360.0;
        */
    return azimuth; //angle based from true north, mount must be aligned to true north
}

const double testBase_LAT = 50.22496794111937;
const double testBase_LON = 8.6366650683187;

double lat, lon;

int main(int argc, char **argv) {
    char line[128];
    struct termios tty;

    int serial_g = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_g < 0) {
        perror("open");
        return 1;
    }

    tcgetattr(serial_g, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tcsetattr(serial_g, TCSANOW, &tty);

    send_g("G90\n", serial_g); //G90 for absolute mode
                               //G91 for relative mode
    send_g("G92\n", serial_g);
    send_g("M201 X50\n", serial_g);
    send_g("G0 F1500\n", serial_g);

    //send_g("G0 X-10\n", serial_g);

    FILE *rec = fopen(argv[1], "r");
    if (!rec)
        return 1;

    double az = calculate_azimuth(testBase_LAT, testBase_LON, 52.48371990820221, 13.426799330106624);
    puts("code loading...");
    printf("angle between coords is %f\n", az);
    move(az, serial_g);

    while (fgets(line, sizeof(line), rec)) {
        if (sscanf(line, "%lf,%lf", &lat, &lon) == 2) {
            double search_ang = calculate_azimuth(testBase_LAT, testBase_LON, lat, lon);
            printf("lon: %f, lat: %f\nANGLE(DEG):%f\n", lon, lat, search_ang);
            move(search_ang, serial_g);
        }
    }

    close(serial_g);

    return 0;
}
