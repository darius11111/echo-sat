#include <stdio.h>
#include <math.h>
#include "tracker.h"

#define RAD_K M_PI/180.0f
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
#define len 245.0

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
    return azimuth * -1; //angle based from true north, mount must be aligned to true north
}

double calculate_altitude_angle(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2) {
    double delta_H = alt2 - alt1;

    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double delta_lambda = (lon2 - lon1) * DEG_TO_RAD;
    double dPhi = (lat2 - lat1) * M_PI / 180.0;

    double a = sin(dPhi/2.0) * sin(dPhi/2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda/2.0) * sin(delta_lambda/2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double distance = 6371000 * c;

    double theta = atan2(delta_H, distance);

    return theta * RAD_TO_DEG;
}

double calc_mm(double ang_deg) {
    double ang = (ang_deg - 20) * DEG_TO_RAD;
    //double hypo = len / cos(ang);
    double x = len * tan(ang);

    return x;
}

const double testBase_LAT = 53.08558089090216;
const double testBase_LON = 8.804766163095788;

double lat, lon, alt;

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
    send_g("M211 S0\n", serial_g);
    send_g("M206 X0 Y0 Z0\n", serial_g);
    send_g("G92 X0 Y0 Z0\n", serial_g);
    send_g("M201 X100\n", serial_g);
    send_g("G0 F2000\n", serial_g);

    FILE *rec = fopen(argv[1], "r");
    if (!rec)
        return 1;
    // PARIS EXAMPLE

    double az = calculate_azimuth(49.87427, 8.65941, 48.856614, 2.35222);
    double dec = calc_mm(30.0);
    puts("code loading...");
    printf("winkel ist: %f\n", az);
    printf("test: %f", dec);
//    move2(az, serial_g);
    //move_alt(dec, serial_g);
    //sleep(5);

    //send_g("G0 X0\n", serial_g);
    //send_g("G0 Z0\n", serial_g);


    while (fgets(line, sizeof(line), rec)) {
        printf("RAW: %s", line);
        if (sscanf(line, "%lf,%lf,%lf", &lat, &lon, &alt) == 3) {
            printf("parsed -> %f, %f, %f", lat, lon, alt);

            double search_ang = calculate_azimuth(testBase_LAT, testBase_LON, lat, lon);
            double alt_ang = calculate_altitude_angle(testBase_LAT, testBase_LON, 100.0, lat, lon, alt);

            printf("Azimuth: %f deg\n", search_ang);
            printf("Altitude angle: %f deg\n", alt_ang);
            move2(search_ang, serial_g);
            double pitch_movement = calc_mm(alt_ang);
            if (alt_ang > 0) //check
                move_alt(pitch_movement, serial_g);
        }
    }

    close(serial_g);

    return 0;
}
