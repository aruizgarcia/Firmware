
/**
 * @file arduino_params.c
 *
 * Parameters defined for the arduino commands.
 *
 * @author Alberto Ruiz García
 */

#include <px4_config.h>
#include <parameters/param.h>

/**
 * Slow mode: lower sampling rate to allow ground communications
 * 
 * @group 
 * @reboot_required true
 */

PARAM_DEFINE_FLOAT(ARDUINO_PWM,0.0f); 
PARAM_DEFINE_INT32(ARDUINO_AUX_CH,0); 
