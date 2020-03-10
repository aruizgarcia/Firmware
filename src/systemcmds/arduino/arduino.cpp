/****************************************************************************
 *
 *   Copyright (c) 2013, 2014, 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pwm.cpp
 *
 * PWM servo output configuration and monitoring tool.
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
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
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#include <arch/board/board.h>

#include "systemlib/err.h"
#include <parameters/param.h>
#include "drivers/drv_pwm_output.h"

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/uORB.h>
#include "arduino_params.c"


#define SLOW_MODE -0.9
#define FAST_MODE 0.1
#define SLAVE_RESET 0.5
#define GENERAL_RESET 1.0

static void	usage(const char *reason);
__BEGIN_DECLS
__EXPORT int	arduino_main(int argc, char *argv[]);
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
This command is used to configure PWM outputs for servo and ESC control.

The default device `/dev/pwm_output0` are the Main channels, AUX channels are on `/dev/pwm_output1` (`-d` parameter).

It is used in the startup script to make sure the PWM parameters (`PWM_*`) are applied (or the ones provided
by the airframe config if specified). `pwm info` shows the current settings (the trim value is an offset
and configured with `PWM_MAIN_TRIMx` and `PWM_AUX_TRIMx`).

The disarmed value should be set such that the motors don't spin (it's also used for the kill switch), at the
minimum value they should spin.

Channels are assigned to a group. Due to hardware limitations, the update rate can only be set per group. Use
`pwm info` to display the groups. If the `-c` argument is used, all channels of any included group must be included.

The parameters `-p` and `-r` can be set to a parameter instead of specifying an integer: use -p p:PWM_MIN for example.

Note that in OneShot mode, the PWM range [1000, 2000] is automatically mapped to [125, 250].

### Examples
Set the PWM rate for all channels to 400 Hz:
$ pwm rate -a -r 400

Test the outputs of eg. channels 1 and 3, and set the PWM value to 1200 us:
$ pwm arm
$ pwm test -c 13 -p 1200

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("Arduino", "command");
}


int
arduino_main(int argc, char *argv[])
{
	int ch;
	int rv = 1;
	float pwm_command = 0.0f; // 1500 us 
	
	if (argc < 2) {
		usage(nullptr);
		return 1;
	}

	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "v:c:e:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'c': 
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
			}
			PX4_INFO("PWM value = %1.1f",(double)pwm_command); 
			break; 		

			default:
			usage(nullptr);
			return 1;
		}
	}
	
	param_set(param_find("ARDUINO_PWM"), &pwm_command);

	PX4_INFO("Command sent."); 
	rv = 0; 
	return rv; 

}
