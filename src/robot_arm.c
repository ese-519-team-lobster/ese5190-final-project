#include "pico/stdlib.h"
#include <stdio.h>
#include "proj_version.h"


int main() {
    while (!stdio_usb_connected());
    printf("Version: %d", PROJECT_VERSION);
    return 1;
}