#include <stdio.h>
#include <unistd.h>

int main(int argc, const char **argv) {
    const char** lat_A_str = argv[2];
    const char** lat_B_str = argv[3];
    FILE *fp = fopen(argv[1], "r");
    if (!fp) {
        perror("fopen");
        return 1;
    }

    char buf[256];

    while (fgets(buf, sizeof(buf), fp)) {
        printf("Received: %s", buf);
        fflush(stdout);
    }

    fclose(fp);
    return 0;
}
