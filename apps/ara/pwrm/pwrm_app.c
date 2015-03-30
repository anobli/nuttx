/*
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted.
 *
 * @brief BDB Power Measurement SVC Application
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pwr_measure.h>
#include <pwrm.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

#define DEFAULT_CONVERSION_TIME     ina230_ct_1_1ms /* 1.1ms */
#define DEFAULT_AVG_SAMPLE_COUNT    ina230_avg_count_64 /* 64 samples average */
#define DEFAULT_CURRENT_LSB         100 /* 100uA current LSB */
#define DEFAULT_REFRESH_RATE        500000 /* 500ms */
#define DEFAULT_LOOPCOUNT           1
#define DEFAULT_CONTINUOUS          0

/* #define DEBUG_BDBPWR */

#ifdef DEBUG_BDBPWR
#define dbg_verbose printf
#else
#define dbg_verbose(format, ...)
#endif

static pwrm_rail *pwrm_bdb_rails[DEV_MAX_RAIL_COUNT][DEV_COUNT];
static uint32_t refresh_rate = DEFAULT_REFRESH_RATE;
static uint32_t loopcount = DEFAULT_LOOPCOUNT;
static uint8_t continuous = DEFAULT_CONTINUOUS;
static uint8_t user_dev_id = DEV_COUNT;
static uint8_t user_rail_id = DEV_MAX_RAIL_COUNT;
static uint32_t current_lsb;
static ina230_conversion_time conversion_time;
static ina230_avg_count avg_count;

static const char ct_strings[9][8] = {
    "140us",
    "204us",
    "332us",
    "588us",
    "1.1ms",
    "2.116ms",
    "4.156ms",
    "8.244ms",
    "ERROR"};

static const int16_t avg_count_array[8] = {
    1,
    4,
    16,
    64,
    128,
    256,
    512,
    1024};

static void usage(void)
{
            printf("Usage: bdbpwr [-d device] [-r rail] [-i current_lsb] [-t conversion_time] [-g avg_count] [-u refresh_rate] [-l loop] [-c] [-h]\n");
            printf("         -d: select device (SW, APB[1-3], GPB[1-2]) (default: all).\n");
            printf("         -r: select rail (default: all):\n");
            printf("             VSW_1P1_PLL, VSW_1P1_CORE, VSW_1P8_UNIPRO, VSW_1P8_IO\n");
            printf("             VAPB[1-3]_1P1_CORE, VAPB[1-3]_1P1_PLL1, VAPB[1-3]_1P2_CDSI_PLL, VAPB[1-3]_1P2_CDSI, VAPB[1-3]_1P2_HSIC, VAPB[1-3]_1P8_UNIPRO, VAPB[1-3]_1P8_IO, VAPB[1-3]_1P1_PLL2\n");
            printf("             VGPB[1-2]_1P1_CORE, VGPB[1-2]_1P1_PLL1, VGPB[1-2]_SDIO, VGPB[1-2]_1P2_HSIC, VGPB[1-2]_1P8_UNIPRO, VGPB[1-2]_1P8_IO, VGPB[1-2]_1P1_PLL2\n");
            printf("         -i: select current measurement precision (LSB, in uA) (default: 100uA):\n");
            printf("         -t: select conversion time (default: 1.1ms):\n");
            printf("             Available options: 140us, 204us, 332us, 588us, 1.1ms, 2.116ms, 4.156ms, 8.244ms.\n");
            printf("         -n: select number of averaging samples (default: 64).\n");
            printf("             Available options: 1, 4, 16, 64, 128, 256, 512, 1024.\n");

            printf("         -u: display refresh rate in milliseconds (default: 500).\n");
            printf("         -l: select number of power measurements (default: 1).\n");
            printf("         -c: select continuous power measurements mode (default: disabled).\n");
            printf("         -h: print help.\n\n");
}


static int pwrm_main_init(void)
{
    uint8_t d, r;
    int ret, rcount;

    ret = pwrm_init(current_lsb, conversion_time, avg_count);
    if (ret) {
        fprintf(stderr, "Error during initialization!!! (%d)\n", ret);
        return ret;
    }

    for (d = 0; d < DEV_COUNT; d++) {
        for (r = 0; r < DEV_MAX_RAIL_COUNT; r++) {
            pwrm_bdb_rails[r][d] = NULL;
        }
    }

    for (d = 0; d < DEV_COUNT; d++) {
        rcount = pwrm_dev_rail_count(d);
        for (r = 0; r < rcount; r++) {
            pwrm_bdb_rails[r][d] = pwrm_init_rail(d, r);
            if (pwrm_bdb_rails[r][d] == NULL) {
                fprintf(stderr, "%s(): Failed to init %s rail! (%d)\n",
                        __func__, pwrm_rail_name(d, r), ret);
                return ret;
            }
        }
    }
    printf("Power measurement HW initialized.\n");
    return 0;
}

static void pwrm_main_deinit(void)
{
    uint8_t d, r;
    int rcount;

    printf("Deinitializing Power measurement HW...\n");
    for (d = 0; d < DEV_COUNT; d++) {
        rcount = pwrm_dev_rail_count(d);
        for (r = 0; r < rcount; r++) {
            if (pwrm_bdb_rails[r][d] != NULL) {
                pwrm_deinit_rail(pwrm_bdb_rails[r][d]);
            }
        }
    }
    pwrm_deinit();
    printf("\nPower measurement HW deinitialized.\n");
}


static ina230_conversion_time pwrm_main_optarg_to_ct(const char *optarg)
{
    if (optarg == NULL) {
        return ina230_ct_count;
    } else if (strcmp(optarg, "140us") == 0) {
        return ina230_ct_140us;
    } else if (strcmp(optarg, "204us") == 0) {
        return ina230_ct_204us;
    } else if (strcmp(optarg, "332us") == 0) {
        return ina230_ct_332us;
    } else if (strcmp(optarg, "588us") == 0) {
        return ina230_ct_588us;
    } else if (strcmp(optarg, "1.1ms") == 0) {
        return ina230_ct_1_1ms;
    } else if (strcmp(optarg, "2.116ms") == 0) {
        return ina230_ct_2_116ms;
    } else if (strcmp(optarg, "4.156ms") == 0) {
        return ina230_ct_4_156ms;
    } else if (strcmp(optarg, "8.244ms") == 0) {
        return ina230_ct_8_244ms;
    } else {
        return ina230_ct_count;
    }
}

static const char *pwrm_main_ct_to_string(ina230_conversion_time time)
{
    if (time >=  ina230_ct_count) {
        return ct_strings[ina230_ct_count];
    } else {
        return ct_strings[time];
    }
}

static ina230_avg_count pwrm_main_optarg_to_avg_count(const char *optarg)
{
    uint32_t val;

    if (optarg == NULL) {
        return ina230_avg_count_max;
    }

    if (sscanf(optarg, "%u", &val) != 1) {
        return ina230_avg_count_max;
    }

    switch (val) {
    case 1:
        return ina230_avg_count_1;

    case 4:
        return ina230_avg_count_4;

    case 16:
        return ina230_avg_count_16;

    case 64:
        return ina230_avg_count_64;

    case 128:
        return ina230_avg_count_128;

    case 256:
        return ina230_avg_count_256;

    case 512:
        return ina230_avg_count_512;

    case 1024:
        return ina230_avg_count_1024;

    default:
        return ina230_avg_count_max;
    }
}

static int16_t pwrm_main_avg_count_to_int(ina230_avg_count count)
{
    if (count >= ina230_avg_count_max) {
        return -1;
    } else {
        return avg_count_array[count];
    }
}

static int pwrm_main_get_user_options(int argc, char **argv)
{
    char c;
    int ret = 0;

    /*
     * As Nuttx apps are not really apps, (it is allocated statically),
     * make sure all static variables are initialized each time
     * the application is launched.
     */
    refresh_rate = DEFAULT_REFRESH_RATE;
    loopcount = DEFAULT_LOOPCOUNT;
    continuous = DEFAULT_CONTINUOUS;
    user_dev_id = DEV_COUNT;
    user_rail_id = DEV_MAX_RAIL_COUNT;
    current_lsb = DEFAULT_CURRENT_LSB;
    conversion_time = DEFAULT_CONVERSION_TIME;
    avg_count = DEFAULT_AVG_SAMPLE_COUNT;

    dbg_verbose("%s(): retrieving user options...\n", __func__);
    optind = 1;
    while ((c = getopt(argc, argv, "hcd:r:l:u:i:t:n:")) != 255) {
        switch (c) {
        case 'd':
            ret = pwrm_device_id(optarg, &user_dev_id);
            if (ret) {
                fprintf(stderr, "Invalid device! (%s)\n", optarg);
                return -EINVAL;
            } else {
                dbg_verbose("%s => dev=%u.\n", optarg, user_dev_id);
                printf("%s device selected.\n", optarg);
            }
            break;

        case 'r':
            ret = pwrm_rail_id(optarg, &user_dev_id, &user_rail_id);
            if (ret) {
                fprintf(stderr, "Invalid rail! (%s)\n", optarg);
                return -EINVAL;
            } else {
                dbg_verbose("%s => dev=%u, rail=%u.\n",
                            optarg, user_dev_id, user_rail_id);
                printf("%s rail selected.\n", optarg);
            }
            break;

        case 'i':
            ret = sscanf(optarg, "%u", &current_lsb);
            if (ret != 1) {
                current_lsb = DEFAULT_CURRENT_LSB;
                fprintf(stderr, "Invalid current precision (%s)! Using default %uuA.\n",
                        optarg, current_lsb);
            } else {
                dbg_verbose("%s => current_lsb=%uuA.\n",
                            optarg, current_lsb);
                printf("Selected %uuA current measurement precision.\n",
                       current_lsb);
            }
            break;

        case 't':
            conversion_time = pwrm_main_optarg_to_ct(optarg);
            if (conversion_time == ina230_ct_count) {
                conversion_time = DEFAULT_CONVERSION_TIME;
                fprintf(stderr, "Invalid conversion time (%s)! Using default %s.\n",
                        optarg, pwrm_main_ct_to_string(conversion_time));
            } else {
                dbg_verbose("%s => conversion_time=%u.\n", optarg, conversion_time);
                printf("Selected %s conversion time.\n", optarg);
            }
            break;

        case 'n':
            avg_count = pwrm_main_optarg_to_avg_count(optarg);
            if (avg_count == ina230_avg_count_max) {
                avg_count = DEFAULT_AVG_SAMPLE_COUNT;
                fprintf(stderr, "Invalid averaging sample count (%s)! Using default %d.\n",
                        optarg, pwrm_main_avg_count_to_int(avg_count));
            } else {
                dbg_verbose("%s => avg_count=%u.\n", optarg, avg_count);
                printf("Selected %s averaging sample count.\n", optarg);
            }
            break;

        case 'c':
            continuous = 1;
            printf("Continuous monitoring mode enabled. Press CTRL+C to stop.\n");
            break;

        case 'u':
            ret = sscanf(optarg, "%u", &refresh_rate);
            if (ret != 1) {
                refresh_rate = DEFAULT_REFRESH_RATE;
                fprintf(stderr,
                        "Invalid display refresh rate (%s)! Using default %ums.\n",
                        optarg, refresh_rate / 1000);
            } else {
                refresh_rate *= 1000;
                printf("Using %uus sampling rate.\n", refresh_rate);
            }
            break;

        case 'l':
            ret = sscanf(optarg, "%u", &loopcount);
            if (ret != 1) {
                loopcount = DEFAULT_LOOPCOUNT;
                fprintf(stderr,
                        "Invalid argument (%s). Use default %u.\n",
                        optarg, loopcount);
            } else {
                printf("Looping %u times.\n", loopcount);
            }
            break;

        case 'h':
        default:
            return -EINVAL;
        }
    }

    dbg_verbose("%s(): done\n", __func__);
    return 0;
}

static void pwrm_main_get_device_list(uint8_t *d_start, uint8_t *d_end)
{
    if (user_dev_id == DEV_COUNT) {
        *d_start = 0;
        *d_end = DEV_COUNT;
    } else {
        *d_start = user_dev_id;
        *d_end = user_dev_id + 1;
    }
    dbg_verbose("%s(): user_dev_id=%u => d_start=%u d_end=%u\n",
                __func__, user_dev_id, *d_start, *d_end);
}

static void pwrm_main_get_rail_list(uint8_t dev,
                                    uint8_t *r_start, uint8_t *r_end)
{
    if (user_rail_id == DEV_MAX_RAIL_COUNT) {
        *r_start = 0;
        *r_end = pwrm_dev_rail_count(dev);
    } else {
        *r_start = user_rail_id;
        *r_end = user_rail_id + 1;
    }
    dbg_verbose("%s(): dev=%u => r_start=%u r_end=%u\n",
                __func__, dev, *r_start, *r_end);
}

static uint8_t pwrm_main_get_max_rail_count(void)
{
    uint8_t max_rcount;

    if (user_dev_id == DEV_COUNT) {
        max_rcount = DEV_MAX_RAIL_COUNT;
    } else {
        if (user_rail_id == DEV_MAX_RAIL_COUNT) {
            max_rcount = pwrm_dev_rail_count(user_dev_id);
        } else {
            max_rcount = 1;
        }
    }
    dbg_verbose("%s(): max_rcount=%u\n", __func__, max_rcount);
    return max_rcount;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int pwrm_main(int argc, char *argv[])
#endif
{
    int ret;
    uint8_t d, r;
    int max_rcount;
    pwr_measure m;
    uint8_t d_start, d_end;
    uint8_t r_start, r_end;
    char separator[512];
    char header[512];
    char s_uV[DEV_MAX_RAIL_COUNT][16];
    char s_uA[DEV_MAX_RAIL_COUNT][16];
    char s_uW[DEV_MAX_RAIL_COUNT][16];

    ret = pwrm_main_get_user_options(argc, argv);
    if (ret) {
        usage();
        exit(-EINVAL);
    }

    ret = pwrm_main_init();
    if (ret) {
        pwrm_main_deinit();
        exit(ret);
    }

    printf("\nGetting power measurements...\n\n");
    if ((continuous) || (loopcount > 1)) {
         /* Clear terminal */
        printf("\033[2J\033[1;1H");
    }

    sprintf(separator, "---------------");
    sprintf(header, "| Module/Rail |");
    max_rcount = pwrm_main_get_max_rail_count();
    for (r = 0; r < max_rcount; r++) {
        strcat(separator, "---------------------");
        strcat(header, "                    |");
    }

    pwrm_main_get_device_list(&d_start, &d_end);
    do {
        printf("%s\n", separator);
        printf("%s\n", header);

        for (d = d_start; d < d_end; d++) {
            printf("| %-11s |", pwrm_dev_name(d));
            pwrm_main_get_rail_list(d, &r_start, &r_end);
            for (r = r_start; r < max_rcount; r++) {
                if (r < r_end) {
                    printf(" %-18s |", pwrm_rail_name(d, r));
                } else {
                    printf(" %-18s |", "");
                }
            }
            printf("\n");

            for (r = r_start; r < r_end; r++) {
                ret = pwrm_measure_rail(pwrm_bdb_rails[r][d], &m);
                if (ret) {
                    sprintf(s_uV[r], "%-10s", "ERROR");
                    sprintf(s_uA[r], "%-10s", "ERROR");
                    sprintf(s_uW[r], "%-10s", "ERROR");
                } else {
                    sprintf(s_uV[r], "%-10d", m.uV / 1000);
                    sprintf(s_uA[r], "%-10d", m.uA);
                    sprintf(s_uW[r], "%-10d", m.uW);
                }
            }
            printf("| %-11s |", "");
            for (r = r_start; r < max_rcount; r++) {
                if (r < r_end) {
                    printf(" V (mV): %s |", s_uV[r]);
                } else {
                    printf(" %-18s |", "");
                }
            }
            printf("\n| %-11s |", "");
            for (r = r_start; r < max_rcount; r++) {
                if (r < r_end) {
                    printf(" I (uA): %s |", s_uA[r]);
                } else {
                    printf(" %-18s |", "");
                }
            }
            printf("\n| %-11s |", "");
            for (r = r_start; r < max_rcount; r++) {
                if (r < r_end) {
                    printf(" P (uW): %s |", s_uW[r]);
                } else {
                    printf(" %-18s |", "");
                }
            }
            printf("\n%s\n", separator);
        }
        if (!continuous) {
            loopcount -= 1;
        }
        if (loopcount != 0) {
            usleep(refresh_rate);
            /* Clear terminal */
            printf("\033[2J\033[1;1H");
        }
    } while (loopcount != 0);

    pwrm_main_deinit();

    return 0;
}
