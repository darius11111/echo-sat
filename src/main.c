#include <stdio.h>
#include <unistd.h>

int main(void) {
    FILE *fp = fopen("/dev/ttyACM0", "r");
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
