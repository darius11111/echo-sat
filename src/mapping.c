#include <stdio.h>
#include "raylib.h"
#include <math.h>

#define RAD_K M_PI/180.0f

double lon, lat;

double calculate_azimuth(double lat1, double lon1, double lat2, double lon2) {

    // deg conversion to pirads
    double phi1 = lat1 * RAD_K;
    double phi2 = lat2 * RAD_K;
    double delta_lambda = (lon2 - lon1) * RAD_K;

    // az calc
    double x = sin(delta_lambda) * cos(phi2);
    double y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda);

    double azimuth = atan2(x, y);

    azimuth *= RAD_K;

    if (azimuth < 0.0)
        azimuth += 360.0;

    return azimuth;
}

int main(int argc, char **argv) {
    FILE *rec = fopen(argv[1], "r");
    if (!rec)
        return 1;

    FILE *g_Station = fopen(argv[2], "w");
    if (!g_Station)
        return 1;

    printf("Type in adjustments: \n");
    size_t ad_X, ad_Y;

    while (fscanf(rec, "%lf,%lf", &lon, &lat)) {
        printf("lon: %f, lat: %f\n", lon, lat);
    }

    scanf("%zu", &ad_X);
    scanf("%zu", &ad_Y);

    return 0;
}
