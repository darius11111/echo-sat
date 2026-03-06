#include <stdio.h>
#include <math.h>

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

    return theta;
}

const double testBase_LAT = 50.22496794111937;
const double testBase_LON = 8.6366650683187;

double lat, lon, alt;

int main(int argc, char **argv) {
    char line[128];

    FILE *rec = fopen(argv[argc-1], "r");
    if (!rec)
        return 1;

    while (fgets(line, sizeof(line), rec)) {
        printf("RAW: %s", line);
        if (sscanf(line, "%lf,%lf,%lf", &lat, &lon, &alt) == 3) {
            printf("parsed -> %f, %f, %f", lat, lon, alt);

            double search_ang = calculate_azimuth(testBase_LAT, testBase_LON, lat, lon);
            double alt_ang = calculate_altitude_angle(testBase_LAT, testBase_LON, 100.0, lat, lon, alt);

            printf("Azimuth: %f deg\n", search_ang);
            printf("Altitude angle: %f deg\n", alt_ang);
        }
    }

    return 0;
}
