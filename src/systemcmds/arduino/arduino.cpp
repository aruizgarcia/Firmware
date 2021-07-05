/**
 * @file arduino.cpp
 * Author: Alberto Ruiz Garcia <a.ruizgarcia-1@tudelft.nl>
 * Faculty of Aerospace Engineering - Delft University of Technology
 *
 * PWM commands to communicate and synchronize with onboard microcontroller.
 * Inspired in pwm.cpp from PX4 systemcmds
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_module.h>
#include <px4_cli.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#include <arch/board/board.h>

#include "systemlib/err.h"
#include <parameters/param.h>

#include "arduino_params.c"
#include "drivers/drv_pwm_output.h"

// PWM output for each command
#define SLOW_MODE -0.8
#define FAST_MODE -0.3
#define IDLE 0.0
#define SLAVE_RESET 0.35
#define GENERAL_RESET 0.85

static void usage(const char *reason);
__BEGIN_DECLS
__EXPORT int    arduino_main(int argc, char *argv[]);
__END_DECLS


static void
usage(const char *reason)
{
    if (reason != nullptr) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("Arduino", "system command");
    PRINT_MODULE_USAGE_COMMAND_DESCR("send", "Sends the command selected with -c flag");
    PRINT_MODULE_USAGE_COMMAND_DESCR("info", "Outputs the current PWM value sent");
    PRINT_MODULE_USAGE_PARAM_COMMENT("Flags:");
    PRINT_MODULE_USAGE_PARAM_FLAG('c', "PWM command: SLOW_MODE/FAST_MODE/SLAVE_RESET/GENERAL_RESET",true);
    PRINT_MODULE_USAGE_PARAM_COMMENT("\t\t.-SLOW_MODE: lowers Arduino sampling rate to allow comms with ground station");
    PRINT_MODULE_USAGE_PARAM_COMMENT("\t\t.-FAST_MODE: increases Arduino sampling rate to take measurements at max rate");
    PRINT_MODULE_USAGE_PARAM_COMMENT("\t\t.-SLAVE_RESET: resets the slave Arduino connected to Air Data Computer");
    PRINT_MODULE_USAGE_PARAM_COMMENT("\t\t.-GENERAL_RESET: resets both the master and slave Arduino");
    PRINT_MODULE_USAGE_PARAM_COMMENT("Example usage: arduino send -c FAST_MODE");
}


int
arduino_main(int argc, char *argv[])
{
    int ch;
    uint8_t command_recvd = 0;
    float pwm_command = 0.0f; // 1500 us

    if (argc < 2) {
        usage(nullptr);
        return 1;
    }

    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "c:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {

        case 'c':
            command_recvd = 1;
            if (strcmp(myoptarg,"SLOW_MODE") == 0) {
                PX4_INFO("Command = SLOW_MODE");
                pwm_command = SLOW_MODE;
            } else if (strcmp(myoptarg,"FAST_MODE") == 0) {
                PX4_INFO("Command = FAST_MODE");
                pwm_command = FAST_MODE;
            } else if (strcmp(myoptarg,"SLAVE_RESET") == 0) {
                PX4_INFO("Command = SLAVE_RESET");
                pwm_command = SLAVE_RESET;
            } else if (strcmp(myoptarg,"GENERAL_RESET") == 0) {
                PX4_INFO("Command = GENERAL_RESET");
                pwm_command = GENERAL_RESET;
            } else if (strcmp(myoptarg, "IDLE") == 0) {
                PX4_INFO("Command = IDLE");
                pwm_command = IDLE; 
            } else {
                PX4_ERR("Command not recognized");
                command_recvd = 0;
            }
            break;

            default:
                usage(nullptr);
                return 1;
        }
    }

    if(myoptind >= argc) {
        usage(nullptr);
        return 1;
    }

    const char *command = argv[myoptind];

    if(!strcmp(command,"send")){

        if (command_recvd > 0){
            PX4_INFO("PWM value = %1.1f",(double)pwm_command);
            param_set(param_find("ARDUINO_PWM"), &pwm_command);
            PX4_INFO("Command sent.");
        }

    } else if(!strcmp(command, "info")){
        const char *dev = "/dev/pwm_output1";
        int fd = px4_open(dev,0);
        int ret;
        servo_position_t arduino_pwm_value;
        int arduino_ch = 0;
        param_get(param_find("ARDUINO_AUX_CH"), &arduino_ch);
        arduino_ch --; // Subtract one (index starts at 0)
        ret = px4_ioctl(fd, PWM_SERVO_GET(arduino_ch),(unsigned long) &arduino_pwm_value);
        if (ret != OK){
            PX4_ERR("Error getting PWM values");
        } else {
            PX4_INFO("Current PWM value = %u us", arduino_pwm_value);
        }
    } else {
        PX4_ERR("Command not recognized");
    }

    return 0;
}