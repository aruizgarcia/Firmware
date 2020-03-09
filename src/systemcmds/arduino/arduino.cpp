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

#define SLOW_MODE 1000
#define FAST_MODE 1350
#define SLAVE_RESET 1650
#define GENERAL_RESET 1900

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
	const char *dev; 
	//bool error_on_warn = false;
	int ch;
	int ret;
	int rv = 1;
	char *ep;
	uint32_t set_mask = 0;
	unsigned long channels;
	unsigned single_ch = 0;
	int pwm_value = 0;

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
				pwm_value = SLOW_MODE; 
				PX4_INFO("PWM value = %d",pwm_value); 
			} else if (strcmp(myoptarg,"FAST_MODE") == 0) {
				PX4_INFO("Command = FAST_MODE"); 
				pwm_value = FAST_MODE; 
				PX4_INFO("PWM value = %d",pwm_value); 
			} else if (strcmp(myoptarg,"SLAVE_RESET") == 0) {
				PX4_INFO("Command = SLAVE_RESET"); 
				pwm_value = SLAVE_RESET; 
				PX4_INFO("PWM value = %d",pwm_value); 
			} else if (strcmp(myoptarg,"GENERAL_RESET") == 0) {
				PX4_INFO("Command = GENERAL_RESET"); 
				pwm_value = GENERAL_RESET; 
				PX4_INFO("PWM value = %d",pwm_value); 
			}
			break; 		

			default:
			usage(nullptr);
			return 1;
		}
	}


	// Set device (aux port) and channels 
	dev = "/dev/pwm_output1"; // AUX output pwm channels
	channels = strtoul("3", &ep, 0); 
	while ((single_ch = channels % 10)) {
		set_mask |= 1 << (single_ch - 1);
		channels /= 10;
	}

	if (myoptind >= argc) {
		usage(nullptr);
		return 1;
	}

	const char *command = argv[myoptind];

	/* open for ioctl only */
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return 1;
	}

	/* get the number of servo channels */
	unsigned servo_count;
	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
		rv = 1; 
		goto err_out_arduino_nopwm;
	}

	if (!strcmp(command, "send")) {

//		if (set_mask == 0) {
//			usage("no channels set");
//			return 1;
//		}

		if (pwm_value == 0) {
			usage("no PWM provided");
			return 1;
		}
		// get current servo values 
		struct pwm_output_values last_spos;

		for (unsigned i = 0; i < servo_count; i++) {

			ret = px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_GET(%d)", i);
				return 1;
			} 
		}

		// perform PWM output 

		if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
				PX4_ERR("Failed to enter servo test mode");
				goto err_out_arduino_nopwm;
		}
	for (unsigned i = 0; i < servo_count; i++) {
		if (set_mask & 1 << i) {
				ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);
				if (ret != OK) {
					PX4_ERR("Arduino, CH(%d)", i);
					goto err_out_arduino;
				}
			}
		}
/*
	for (unsigned i = 0; i < servo_count; i++) {
		if (set_mask & 1 << i) {
				ret = px4_ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);
				if (ret != OK) {
					PX4_ERR("Arduino, CH(%d)", i);
					goto err_out_arduino;
				} else {
					PX4_INFO("Channel #%d, pwm = %d",i,last_spos.values[i]); 
			}
		}
	}
*/
	//ret = px4_ioctl(fd, PWM_SERVO_SET(7), pwm_value);
//	if (ret != OK) {
//		PX4_ERR("Arduino PWM set failed");
//	goto err_out_arduino;
//	}
//
//	goto err_out_arduino; 	

		
	px4_usleep(2542); 
#ifdef __PX4_NUTTX
	up_pwm_update(); // Trigger all timer's channels in oneshot
#endif
	rv = 0; 
	if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
		rv = 1;
		PX4_ERR("Failed to exit servo test mode");
	}

	return rv; 


err_out_arduino:
			if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
					rv = 1;
					PX4_ERR("Failed to exit servo test mode");
			}

err_out_arduino_nopwm:
		return rv;

	}

	usage(nullptr);
	return 0;

}
