#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <bcm2835.h>

#include "config.h"

typedef struct
{
    double period;
    double bound;
    double k_p, k_i, k_d;
    double integral;
    double differential;
    double last_error;
} pid_status;

static inline int get_thermal_zone0_temp(void)
{
    FILE* fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (!fp) return -1;
    int temp;
    int ret = fscanf(fp, "%d", &temp);
    fclose(fp);
    if (ret == 0 || ret == EOF) return -1;
    else return temp;
}

static inline int pid_status_init(pid_status* status,
    double period, double bound, double k_p, double k_i, double k_d)
{
    if (!isnormal(period) || signbit(period)) return 0;
    if (!isnormal(bound) && !isinf(bound))
        return 0;
    if (!isnormal(k_p) && fpclassify(k_p) != FP_ZERO)
        return 0;
    if (!isnormal(k_i) && fpclassify(k_i) != FP_ZERO)
        return 0;
    if (!isnormal(k_d) && fpclassify(k_d) != FP_ZERO)
        return 0;
    status->period = period;
    status->bound = fabs(bound);
    status->k_p = k_p;
    status->k_i = k_i;
    status->k_d = k_d;
    status->integral = 0.0;
    status->differential = 0.0;
    status->last_error = 0.0;
    return 1;
}

static inline double pid_next(pid_status* status, double error)
{
    if (!isnormal(error) && fpclassify(error) != FP_ZERO)
        return NAN;
    errno = 0;
    if (status->k_i != 0.0)
        status->integral += (error + status->last_error) * status->period / 2;
    if (status->k_d != 0.0)
        status->differential = (error - status->last_error) / status->period;
    status->last_error = error;
    double p = status->k_p * error;
    double i = status->k_i * status->integral;
    double d = status->k_d * status->differential;
    double u = p + i + d;
    // back-calculation anti-windup
    if (isfinite(status->bound) && fabs(u) > status->bound)
    {
        if (signbit(u)) u = -status->bound;
        else u = status->bound;
        if (status->k_i != 0) status->integral = (u - p - d) / status->k_i;
    }
    if (errno != 0) return NAN;
    return u;
}

static inline void check_root(void)
{
    if (geteuid()) // non-root
    {
        fputs("You don't have permission to run this program.\n", stderr);
        fputs("Command \"sudo\" could help you. Give it a try! :P\n", stderr);
        exit(1);
    }
}

// this "PID" is different from above
static inline void check_pid_file(char* path)
{
    int fd = open(path, O_WRONLY | O_CREAT,
        S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd == -1)
    {
        fprintf(stderr, "Can't open %s\n", path);
        exit(1);
    }
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    fl.l_pid = getpid();
    if (fcntl(fd, F_SETLK, &fl) == -1)
    {
        fprintf(stderr, "Can't lock %s\n", path);
        fputs("Maybe there is already a same program running?\n", stderr);
        exit(1);
    }
    if (ftruncate(fd, 0) == -1)
    {
        fprintf(stderr, "Can't truncate %s\n", path);
        exit(1);
    }
    char buf[64];
    int len =
        snprintf(buf, sizeof(buf) / sizeof(char), "%d\n", (int) getpid());
    if (len < 0 || write(fd, buf, len) != len)
    {
        fprintf(stderr, "Can't write to %s\n", path);
        exit(1);
    }
}

static inline void stop_pwm(void)
{
    bcm2835_gpio_fsel(GPIO, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(GPIO, OUTPUT_WHILE_EXIT);
    bcm2835_pwm_set_clock(BCM2835_SPI_CLOCK_DIVIDER_1);
    bcm2835_pwm_set_mode(PWM_CHANNEL, 0, 0);
    bcm2835_close();
}

static void handle_signal(int signal)
{
    unlink(FILE_PID_PATH);
    exit(128 + signal);
}

static void handle_signal_with_pwm(int signal)
{
    stop_pwm();
    handle_signal(signal);
}

int main(void)
{
    check_root();
    umask(0);
    signal(SIGINT, SIG_IGN);
    signal(SIGTERM, SIG_IGN);
    check_pid_file(FILE_PID_PATH);
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    pid_status status;
    if (!pid_status_init(&status, PERIOD, PID_BOUND,
        PID_K_P, PID_K_I, PID_K_D))
        return 1;
    int selected = 0;
    double feedback = 0.0;
    bcm2835_set_debug(DEBUG_BCM2835);
    signal(SIGINT, SIG_IGN);
    signal(SIGTERM, SIG_IGN);
    if (!bcm2835_init()) return 1;
    signal(SIGINT, handle_signal_with_pwm);
    signal(SIGTERM, handle_signal_with_pwm);
    bcm2835_pwm_set_clock(PWM_CLOCK_DIVIDER);
    bcm2835_pwm_set_mode(PWM_CHANNEL, PWM_MODE, 1);
    bcm2835_pwm_set_range(PWM_CHANNEL, PWM_RANGE);
    while (1)
    {
        int dutycycle_data = round((-feedback / PID_BOUND + 1) / 2 *
            (DATA_UPPER_BOUND - DATA_LOWER_BOUND)) + DATA_LOWER_BOUND;
        bcm2835_pwm_set_data(PWM_CHANNEL, dutycycle_data);
        if (!selected)
        {
            bcm2835_gpio_fsel(GPIO, PWM_GPIO_ALT);
            selected = 1;
        }
        sleep(PERIOD);
        int temp = get_thermal_zone0_temp();
        if (temp == -1)
        {
            fputs("Can't get temperature.\n", stderr);
            fputs("This program is aborted.\n", stderr);
            stop_pwm();
            return 1;
        }
        double error = SETPOINT_TEMP - temp / 1000.0;
        feedback = pid_next(&status, error);
        if (isnan(feedback))
        {
            fputs("Feedback has been NaN, controller is abnormal.\n", stderr);
            fputs("This program is aborted.\n", stderr);
            stop_pwm();
            return 1;
        }
    }
}
