#include <stdio.h>
#include "raylib.h"
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

    if (azimuth < 0.0)
        azimuth += 360.0;

    return azimuth; //angle based from true north, mount must be aligned to true north
}

const double testBase_LAT = 50.22496794111937;
const double testBase_LON = 8.6366650683187;

double lat, lon;

int main(int argc, char **argv) {
    char line[128];

    FILE *rec = fopen(argv[1], "r");
    if (!rec)
        return 1;
    double az = calculate_azimuth(testBase_LAT, testBase_LON, 48.77767930696142, 2.4429348160795894);
    puts("code loading...");
    printf("angle between coords is %f\n", az);

    /*FILE *g_Station = fopen(argv[2], "w");
    if (!g_Station)
        return 1;
    */
    //printf("Type in adjustments: \n");
    //size_t ad_X, ad_Y;



    while (fgets(line, sizeof(line), rec)) {
        if (sscanf(line, "%lf,%lf", &lat, &lon) == 2) {
            double search_ang = calculate_azimuth(testBase_LAT, testBase_LON, lat, lon);
            printf("lon: %f, lat: %f\nANGLE(DEG):%f\n", lon, lat, search_ang);
        }
    }

    //scanf("%zu", &ad_X);
    //scanf("%zu", &ad_Y);

    return 0;
}
