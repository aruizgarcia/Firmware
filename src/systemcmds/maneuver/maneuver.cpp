/****************************************************************************
 *
*
 ****************************************************************************/

/**
 * @file maneuver.cpp
 * 
 * Programmable maneuvers for aerodynamic model identification using the test 
 * mode for the servos.
 * 
 * @author Alberto Ruiz Garcia <aruizgarcia-1@tudelft.nl<
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

#include <drivers/drv_hrt.h>

static void	usage(const char *reason);
__BEGIN_DECLS
__EXPORT int	maneuver_main(int argc, char *argv[]);
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
This command is used to ...

### Examples
...

...

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("maneuver", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("arm", "Arm output");

	PRINT_MODULE_USAGE_PARAM_COMMENT("These parameters apply to all commands:");
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "Verbose output", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('e', "Exit with 1 instead of 0 on error", true);

}

/* Function prototypes */
static int pwm_arm (int fd); 
unsigned check_commanded_us(unsigned val, int min_pulse, int max_pulse); 

/* MAIN PROGRAM */

int
maneuver_main(int argc, char *argv[])
{
	const char *dev0 = "/dev/pwm_output0"; // PWM_OUTPUT0_DEVICE_PATH; 
	const char *dev1 = "/dev/pwm_output1"; // PWM_OUTPUT1_DEVICE_PATH; 
	bool print_verbose = false;
	bool error_on_warn = false;
	int ch;
	int ret;
	int rv = 1;
	char *ep;
	uint32_t set_mask0 = 0;
	uint32_t set_mask1 = 0;
	unsigned long channels0 = 0;
	unsigned long channels1 = 0;
	unsigned single_ch = 0;
	uint8_t control_surface = 0; 
	int max_pulse_right = 1500; 
	int max_pulse_left = 1500; 
	int min_pulse_right = 1500; 
	int min_pulse_left = 1500;  
	int servo_center_right = 1500; 
	int servo_center_left = 1500; 
	
	int amplitude = 0;  
	int duration = 0; 

	if (argc < 2) {
		usage(nullptr);
		return 1;
	}

	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "a:c:t:v:e:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'v':
			print_verbose = true;
			break;

		case 'e':
			error_on_warn = true;
			break;
		case 'a':
			if (px4_get_parameter_value(myoptarg, amplitude) != 0) {
				PX4_ERR("CLI argument parsing for maneuver amplitude failed");
				return 1;
			} else {
				PX4_INFO("Maneuver amplitude: %d%%", amplitude); 
			}
			break;  
		case 'c': 
			if (strcmp(myoptarg,"elevator") == 0) {
				PX4_INFO("Control surface:    elevator"); 
				control_surface = 1;  
			} else if (strcmp(myoptarg,"elevons") == 0) {
				PX4_INFO("Control surface:    elevons"); 
				control_surface = 2;  
			} else if (strcmp(myoptarg,"ailerons") == 0) {
				PX4_INFO("Control surface:    ailerons"); 
				control_surface = 3;  
			} else if (strcmp(myoptarg,"rudders") == 0) {
				PX4_INFO("Control surface:    rudders"); 
				control_surface = 4;  
//			} else if (strcmp(myoptarg,"full_elevons") == 0) {
//				PX4_INFO("Control surface = full elevons"); 
//				control_surface = 5;  
			}	
			break; 	
		case 't': 
			if (px4_get_parameter_value(myoptarg, duration) != 0) {
				PX4_ERR("CLI argument parsing for maneuver duration failed");
				return 1;
			} else {
				PX4_INFO("Maneuver duration:  %d seconds", duration); 
			}
			break; 

		default:
			usage(nullptr);
			return 1;
		}
	}


	/* Set output channels for the selected maneuver*/
	switch(control_surface){

		case 1: /* Elevators */
			 
			channels0 = strtoul("3", &ep, 0);  // Right wing  
			 param_get(param_find("PWM_MAIN_MAX3"),  &max_pulse_right);
			 param_get(param_find("PWM_MAIN_MIN3"),  &min_pulse_right);
			 param_get(param_find("PWM_MAIN_FAIL3"), &servo_center_right);

			channels1 = strtoul("5", &ep, 0); // Left wing  
			 param_get(param_find("PWM_AUX_MAX5"), &max_pulse_left);
			 param_get(param_find("PWM_AUX_MIN5"), &min_pulse_left);
			 param_get(param_find("PWM_AUX_FAIL5"), &servo_center_left);

			break; 

		case 2:  /* Elevons */ 
			channels0 = strtoul("4", &ep, 0); 
			 param_get(param_find("PWM_MAIN_MAX4"), &max_pulse_right);
			 param_get(param_find("PWM_MAIN_MIN4"), &min_pulse_right);
			 param_get(param_find("PWM_MAIN_FAIL4"), &servo_center_right); 

			 channels1 = strtoul("6", &ep, 0); 
			 param_get(param_find("PWM_AUX_MAX6"), &max_pulse_left);
			 param_get(param_find("PWM_AUX_MIN6"), &min_pulse_left);
			 param_get(param_find("PWM_AUX_FAIL6"), &servo_center_left);

			break; 

		case 3:  /* Ailerons */ 
			 channels0 = strtoul("5", &ep, 0); 
			 param_get(param_find("PWM_MAIN_MAX5"), &max_pulse_right);
			 param_get(param_find("PWM_MAIN_MIN5"), &min_pulse_right);
			 param_get(param_find("PWM_MAIN_FAIL5"), &servo_center_right); 

			 channels1 = strtoul("7", &ep, 0); 
			 param_get(param_find("PWM_AUX_MAX7"), &max_pulse_left);
			 param_get(param_find("PWM_AUX_MIN7"), &min_pulse_left);
			 param_get(param_find("PWM_AUX_FAIL7"), &servo_center_left);

			 break; 

		case 4:  /* Rudders */ 
			 channels0 = strtoul("6", &ep, 0); 
			 param_get(param_find("PWM_MAIN_MAX6"), &max_pulse_right);
			 param_get(param_find("PWM_MAIN_MIN6"), &min_pulse_right);
			 param_get(param_find("PWM_MAIN_FAIL6"), &servo_center_right); 

			 channels1 = strtoul("8", &ep, 0); 
			 param_get(param_find("PWM_AUX_MAX8"), &max_pulse_left);
			 param_get(param_find("PWM_AUX_MIN8"), &min_pulse_left);
			 param_get(param_find("PWM_AUX_FAIL8"), &servo_center_left);

			break; 
	}

	/* Generate bitmask */
	while ((single_ch = channels0 % 10)) {
		set_mask0 |= 1 << (single_ch - 1);
		channels0 /= 10;
	}

	while ((single_ch = channels1 % 10)) {
		set_mask1 |= 1 << (single_ch - 1);
		channels1 /= 10;
	}

	if (myoptind >= argc) {
		usage(nullptr);
		return 1;
	}

	const char *command = argv[myoptind];

	if (print_verbose && set_mask0 > 0) {
		PX4_INFO("Channels (MAIN): ");
		printf("    ");

		for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
			if (set_mask0 & 1 << i) {
				printf("%d ", i + 1);
			}
		}

		printf("\n");
	}

	if (print_verbose && set_mask1 > 0) {
		PX4_INFO("Channels (AUX): ");
		printf("    ");

		for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
			if (set_mask1 & 1 << i) {
				printf("%d ", i + 1);
			}
		}

		printf("\n");
	}

	/* open for ioctl only */
	int fd_main = px4_open(dev0, 0);
	int fd_aux = px4_open(dev1, 0);

	if (fd_main < 0) {
		PX4_ERR("can't open %s", dev0);
		return 1;
	}

	if (fd_aux < 0) {
		PX4_ERR("can't open %s", dev1);
		return 1;
	}

	/* get the number of servo channels */
	unsigned servo_count_main;
	unsigned servo_count_aux;
	ret = px4_ioctl(fd_main, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count_main);
	if (ret != OK) {
		PX4_ERR("MAIN: PWM_SERVO_GET_COUNT");
		return error_on_warn;
	}

	ret = px4_ioctl(fd_aux, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count_aux);
	if (ret != OK) {
		PX4_ERR("AUX : PWM_SERVO_GET_COUNT");
		return error_on_warn;
	}

	if (!strcmp(command, "doublet")) {

		if (pwm_arm(fd_main) != 0) {
			PX4_ERR("MAIN: error arming PWM outputs");
			return 1;
		} else {
		 	PX4_INFO("MAIN: PWM outputs armed");
		}

		if (pwm_arm(fd_aux) != 0) {
			PX4_ERR("AUX : error arming PWM outputs");
			return 1;
		} else {
		 	PX4_INFO("AUX : PWM outputs armed");
		}

		if (!set_mask0 && !set_mask1) {
			usage("No channels set");
			return 1;
		} else if (!set_mask0 || !set_mask1) {
			PX4_ERR("Main or aux channels not set");
			return 1; 
		}

		/* get current servo values */
		struct pwm_output_values last_spos_main;
		struct pwm_output_values last_spos_aux;

		for (unsigned i = 0; i < servo_count_main; i++) {

			ret = px4_ioctl(fd_main, PWM_SERVO_GET(i), (unsigned long)&last_spos_main.values[i]);

			if (ret != OK) {
				PX4_ERR("MAIN: PWM_SERVO_GET(%d)", i);
				return 1;
			}
		}

		for (unsigned i = 0; i < servo_count_aux; i++) {

			ret = px4_ioctl(fd_aux, PWM_SERVO_GET(i), (unsigned long)&last_spos_aux.values[i]);

			if (ret != OK) {
				PX4_ERR("AUX : PWM_SERVO_GET(%d)", i);
				return 1;
			}
		}
		
 		// Perform PWM output

		uint16_t seconds_to_ms = 1000; 
		uint64_t maneuver_duration = duration * seconds_to_ms; 
		double default_duration = 10000.0;
		double scale_ratio = maneuver_duration/default_duration; 
		unsigned steps_timing_ms[] = {0, 1000, 4000, 7000, 10000};

		// Scale the maneuver timing to match the desired duration
		for (uint8_t idx = 0; idx < sizeof(steps_timing_ms)/sizeof(steps_timing_ms[0]); idx++ ){
			steps_timing_ms[idx] *= scale_ratio; 
		}

		uint64_t start_time;
		uint64_t maneuver_time;  
		double servo_command = 0;		
		int phase = 0; 
		int prev_phase = 0; 
		bool disp_pwm = false; 

		/* Open console directly to grab CTRL-C signal */
		struct pollfd fds;
		fds.fd = 0; /* stdin */
		fds.events = POLLIN;

		PX4_WARN("WARNING: Starting maneuver in 5 seconds. \n \t\t Press any key to abort now!.");
		px4_sleep(5);

        if (::ioctl(fd_main, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
            PX4_ERR("Failed to Enter pwm test mode (MAIN)");
            goto err_out_no_test;
        }
        
		if (::ioctl(fd_aux, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
			PX4_ERR("Failed to Enter pwm test mode (AUX)");
			goto err_out_no_test;
		}
		
		PX4_INFO("<< STARTING AUTOMATIC MANEUVER >>"); 

		start_time = hrt_absolute_time(); 
		do{
			unsigned val;
			maneuver_time = (hrt_absolute_time() - start_time)/1000;  

			if (maneuver_time < steps_timing_ms[1]) {
				      servo_command = 0;
				      phase = 1; 
				    } else if (maneuver_time  < steps_timing_ms[2]) {
				      servo_command = (float) (amplitude/100.0);
				      phase = 2; 
				    } else if (maneuver_time  < steps_timing_ms[3]) {
				      servo_command = -(float) (amplitude/100.0);
				      phase = 3; 
				    } else {// end of maneuver
				      servo_command = 0;
				      phase = 4; 
			}

			if (phase != prev_phase) {
					PX4_INFO("Phase #%d, time = %llu ms", phase, maneuver_time);
					prev_phase ++; 
					disp_pwm = true; 
			}

			/* RIGHT WING */ 
			for (unsigned i = 0; i < servo_count_main; i++) {
				if (set_mask0 & 1 << i) {					

					servo_center_right = last_spos_main.values[i]; 
				    if (servo_command > 0.0){
				    	val = servo_center_right + (max_pulse_right - servo_center_right) * servo_command; 
				    } else {
				    	val = servo_center_right - (min_pulse_right - servo_center_right) * servo_command; 
				    }

				    if(disp_pwm) {
						PX4_INFO("  -MAIN: Channel #%d -- PWM value [us] = %d",i+1,val); 
					}

					val = check_commanded_us(val, min_pulse_right, max_pulse_right); 
					ret = px4_ioctl(fd_main, PWM_SERVO_SET(i), val);
					if (ret != OK) {
						PX4_ERR("MAIN: PWM_SERVO_SET(%d)", i);
						goto err_out;
					}
				} else { // Write last position before starting maneuver
					val = check_commanded_us(last_spos_main.values[i], 1000, 2000); 
					ret = px4_ioctl(fd_main, PWM_SERVO_SET(i), val); 
					if (ret != OK) {
						PX4_ERR("MAIN: PWM_SERVO_SET(%d)", i);
						goto err_out;
					}
				}
			}

			/* LEFT WING  */
			for (unsigned i = 0; i < servo_count_aux; i++) {
				if (set_mask1 & 1 << i) {

					servo_center_left = last_spos_aux.values[i]; 
				    if (servo_command > 0.0){
				    	val = servo_center_left + (max_pulse_left - servo_center_left) * servo_command; 
				    } else {
				    	val = servo_center_left - (min_pulse_left - servo_center_left) * servo_command; 
				    }

				    if(disp_pwm) {
						PX4_INFO("  -AUX : Channel #%d -- PWM value [us] = %d",i+1,val); 
					}

					val = check_commanded_us(val, min_pulse_left, max_pulse_left); 
					ret = px4_ioctl(fd_aux, PWM_SERVO_SET(i), val);
					if (ret != OK) {
						PX4_ERR("AUX : PWM_SERVO_SET(%d)", i);
						goto err_out;
					}
				} else { // Write last position before starting maneuver
					val = check_commanded_us(last_spos_aux.values[i], 1000, 2000); 
					ret = px4_ioctl(fd_aux, PWM_SERVO_SET(i), val); 
					if (ret != OK) {
						PX4_ERR("AUX : PWM_SERVO_SET(%d)", i);
						goto err_out;
					}
				}
			}

			if (disp_pwm) disp_pwm = false; 

			/* abort on user request */
			char c;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				ret = read(0, &c, 1);

				if (ret > 0) {
					/* reset output to the last value */
					for (unsigned j = 0; j < servo_count_main; j++) {
						if (set_mask0 & 1 << j) {

							ret = px4_ioctl(fd_main, PWM_SERVO_SET(j), last_spos_main.values[j]);
							if (ret != OK) {
								PX4_ERR("PWM_SERVO_SET(%d)", j);
								goto err_out;
							}
						}
					}

					for (unsigned j = 0; j < servo_count_aux; j++) {
						if (set_mask1 & 1 << j) {

							ret = px4_ioctl(fd_aux, PWM_SERVO_SET(j), last_spos_aux.values[j]);
							if (ret != OK) {
								PX4_ERR("PWM_SERVO_SET(%d)", j);
								goto err_out;
							}
						}
					}

					PX4_INFO("User abort\n");
					rv = 0;
					goto err_out; 					
				}
			}
		
		} while (maneuver_time < maneuver_duration);
	
	PX4_INFO("<< MANEUVER EXECUTION FINISHED >>"); 
	rv = 0; 
	goto err_out;  

	err_out:
		if (::ioctl(fd_main, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
			rv = 1;
			PX4_ERR("Failed to Exit pwm test mode (MAIN)");
			return rv; 
		}
		if (::ioctl(fd_aux, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
			rv = 1;
			PX4_ERR("Failed to Exit pwm test mode (AUX)");
			return rv; 
		}	
	err_out_no_test: 
		return rv; 
	}	

	return rv; 
}


int pwm_arm(int fd) {

	int ret; 

	/* tell safety that its ok to disable it with the switch */
    ret = px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);

    if (ret != OK) {
            err(1, "PWM_SERVO_SET_ARM_OK");
    }

    /* tell IO that the system is armed (it will output values if safety is off) */
    ret = px4_ioctl(fd, PWM_SERVO_ARM, 0);

    if (ret != OK) {
            err(1, "PWM_SERVO_ARM");
    }

    return 0;

}

unsigned check_commanded_us(unsigned val, int min_pulse, int max_pulse) {

	if (val > (uint16_t)max_pulse) {
		return max_pulse; 
	} else if (val < (uint16_t)min_pulse) {
		return min_pulse; 
	} else {
		return val; 
	}

}
