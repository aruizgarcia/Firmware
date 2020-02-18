
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

PARAM_DEFINE_INT32(IDLE, 900); 
PARAM_DEFINE_INT32(SLOW_MODE, 1000); 
PARAM_DEFINE_INT32(FAST_MODE, 1350); 
PARAM_DEFINE_INT32(SLAVE_RESET, 1600); 
PARAM_DEFINE_INT32(GENERAL_RESET, 1800); 