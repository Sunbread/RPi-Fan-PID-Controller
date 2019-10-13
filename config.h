#ifndef _FAN_CONTROL_CONFIG_H
#define _FAN_CONTROL_CONFIG_H
/* Begin Configuration */
#define FILE_PID_PATH "/run/fan-control.pid" // This "PID" is not below "PID"s
#define DEBUG_BCM2835 0 // 0 to disable, 1 to enable
#define SETPOINT_TEMP 45 // 'C
#define GPIO 18 // Pin 12
#define OUTPUT_WHILE_EXIT HIGH
#define PWM_GPIO_ALT BCM2835_GPIO_FSEL_ALT5 // GPIO 18
#define PWM_CLOCK_DIVIDER BCM2835_PWM_CLOCK_DIVIDER_2 // 9.6 MHz for steps
#define PWM_CHANNEL 0 // GPIO 18
#define PWM_MODE 1 // Mark-Space mode
#define PWM_RANGE 960 // 9.6 MHz / 960 = 10 KHz
#define DATA_LOWER_BOUND 480 // 1/2 Duty-cycle
#define DATA_UPPER_BOUND 960 // 1/1 Duty-cycle
#define PERIOD 1 // second
#define PID_BOUND 10
#define PID_K_P 2
#define PID_K_I 1
#define PID_K_D 1
/* End Configuration */
#endif
