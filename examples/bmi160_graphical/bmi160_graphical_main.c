/****************************************************************************
 * bmi160_graphical/bmi160_graphical_main.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bmi160_graphical.h"
#include "camera.h"
#include "nximage.h"
#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/sensors/bmi160.h>
#include <sdk/config.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Set the angles of your board! */
#define X_ANGLE DEG_TO_RADIAN(0.0)
#define Y_ANGLE DEG_TO_RADIAN(25.0)
#define Z_ANGLE DEG_TO_RADIAN(0.0)

#define ACC_DEVPATH "/dev/accel0"
#define GYRO_DEVPATH "/dev/gyro0"

#define ESC_CHAR 0x1B

#define DEG_TO_RADIAN(d) (d * (3.14 / 180))
#define RADIAN_TO_DEG(r) r *(180 / 3.14)

#define MAXIMUM_ROTATION_SPEED 20
#define MAXIMUM_RESULTANT 50

#define ACCEL_RANGE 2000.0 // mg
#define ACCEL_RESOLUTION ACCEL_RANGE / INT16_MAX

#define LED0 PIN_I2S1_BCK
#define LED1 PIN_I2S1_LRCK
#define LED2 PIN_I2S1_DATA_IN
#define LED3 PIN_I2S1_DATA_OUT

#define CALIB_LOOPS 100

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct calib_accel_t {
	int32_t x;
	int32_t y;
	int32_t z;
};

struct calib_gyro_t {
	int32_t x;
	int32_t y;
	int32_t z;
};

struct calib_accel_gyro_st_s {
	struct calib_accel_t accel;
	struct calib_gyro_t gyro;
};

static BMI160_gyro_range_t range = BMI160_GYRO_RANGE_2000;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * bmi160_graphical_main
 ****************************************************************************/
static void led(int led, bool state) { board_gpio_write(led, state); }

/****************************************************************************
 * bmi160 calibration related
 ****************************************************************************/
int calibration(struct accel_gyro_st_s *calibration_data, int fd) {
	int ret;
	int calibration;
	struct offset_s bmi160_offset = {{0}};
	struct calib_accel_gyro_st_s calib = {{0}};

	ret = ioctl(fd, SNIOC_SET_GYRO_RANGE, BMI160_GYRO_RANGE_2000);
	printf("Setting Gyro rate, return %d\n", ret);
	for (calibration = 0; calibration < CALIB_LOOPS; calibration++) {

		ret = read(fd, calibration_data, sizeof(struct accel_gyro_st_s));
		if (ret != sizeof(struct accel_gyro_st_s)) {
			fprintf(stderr, "Read failed.\n");
			break;
		}
		calib.gyro.x += calibration_data->gyro.x;
		calib.gyro.y += calibration_data->gyro.y;
		calib.gyro.z += calibration_data->gyro.z;

		calib.accel.x += calibration_data->accel.x;
		calib.accel.y += calibration_data->accel.y;
		calib.accel.z += calibration_data->accel.z;
	}

	calib.gyro.x = -calib.gyro.x / CALIB_LOOPS;
	calib.gyro.y = -calib.gyro.y / CALIB_LOOPS;
	calib.gyro.z = -calib.gyro.z / CALIB_LOOPS;

	calib.accel.x = -calib.accel.x / CALIB_LOOPS;
	calib.accel.y = -calib.accel.y / CALIB_LOOPS;
	calib.accel.z = -calib.accel.z / CALIB_LOOPS;

	bmi160_offset.gyro.x = calib.gyro.x & 0x0ff;
	bmi160_offset.gyro.y = calib.gyro.y & 0x0ff;
	bmi160_offset.gyro.z = calib.gyro.z & 0x0ff;

	bmi160_offset.gyro_x_msb = calib.gyro.x >> 8 & 0x03;
	bmi160_offset.gyro_y_msb = calib.gyro.y >> 8 & 0x03;
	bmi160_offset.gyro_z_msb = calib.gyro.z >> 8 & 0x03;

	bmi160_offset.accel.x = calib.accel.x & 0x0ff;
	bmi160_offset.accel.y = calib.accel.y & 0x0ff;
	bmi160_offset.accel.z = calib.accel.z & 0x0ff;

	bmi160_offset.gyro_offset_en = 1;
	bmi160_offset.accel_offset_en = 0;

	ret = ioctl(fd, SNIOC_SET_OFFSET, (long unsigned int)&bmi160_offset);
	return ret;
}

/****************************************************************************
 * bmi160_accel related
 ****************************************************************************/

static inline uint16_t accel_resultant(struct accel_gyro_st_s *data) {
	double resultant;
	resultant = sqrt(pow(data->accel.x, 2.0) + pow(data->accel.y, 2.0) + pow(data->accel.z, 2.0));
	resultant = resultant * ACCEL_RESOLUTION;
	printf("Resultant of: % 7d % 7d % 7d is % 7lfmg    \n", data->accel.x, data->accel.y, data->accel.z, resultant);
	return (uint16_t)resultant;
}

static inline int16_t accel_angle(int16_t vector_a, int16_t vector_b) {
	double angle;
	angle = atan((double)vector_a / vector_b);
	angle = RADIAN_TO_DEG(angle);

	printf("Acceleromenter angle: % 7d % 7d , angle is: %2.3f    \n", vector_a, vector_b, angle);
	return (int16_t)angle * 100;
}

static inline int16_t accel_angle_xz(struct accel_gyro_st_s *calib_data, struct accel_gyro_st_s *data, abs_angle_t *accel_angles) {
	int16_t angle;

	angle = accel_angle(data->accel.x - calib_data->accel.x, data->accel.z - calib_data->accel.z);
	return angle;
}

static inline int16_t accel_angle_xy(struct accel_gyro_st_s *calib_data, struct accel_gyro_st_s *data, abs_angle_t *accel_angles) {
	int16_t angle;

	angle = accel_angle(data->accel.x - calib_data->accel.x, data->accel.y - calib_data->accel.y);
	return angle;
}

/****************************************************************************
 * bmi160_gyro related
 ****************************************************************************/
static inline double gyro_k(void) {
	switch (range) {
	case BMI160_GYRO_RANGE_2000:
		return 61;
		break;
	case BMI160_GYRO_RANGE_1000:
		return 30.5;
		break;
	case BMI160_GYRO_RANGE_500:
		return 15.3;
		break;
	case BMI160_GYRO_RANGE_250:
		return 7.6;
		break;
	case BMI160_GYRO_RANGE_125:
		return 3.8;
		break;
	}
	return 0;
}

static inline int32_t gyro_rate_mdeg_per_sec(int16_t gyroval) { return gyroval * gyro_k(); }

//#define TIMER_TICK_US 39 /* Strange, according to datasheet it is easy to think that this value should be multiplied with sensortime. */
#define TIMER_TICK_US 10

int64_t gyro_degrees(int16_t gyroval, int32_t dtime) {
	int32_t dtime_us;
	int64_t angle_mdegrees;
	dtime_us = (dtime * TIMER_TICK_US);

	angle_mdegrees = gyro_rate_mdeg_per_sec(gyroval) * dtime_us;

	return angle_mdegrees;
}

int gyro_absolute_angle(struct accel_gyro_st_s *data, abs_angle_t *angle) {
	static int32_t last = 0;
	int32_t d_time;

	d_time = data->sensor_time - last;

	angle->x += gyro_degrees(data->gyro.x, d_time);
	angle->y += gyro_degrees(data->gyro.y, d_time);
	angle->z += -gyro_degrees(data->gyro.z, d_time);

	last = data->sensor_time;
	return 0;
}

typedef struct {
	int p;
	int i;
	int d;
	int64_t error;
} pid_reg_t;

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int bmi160_graphical_main(int argc, char *argv[])
#endif
{
	int ret = 0;
	int fd = 0;
	struct accel_gyro_st_s data = {{0}};

	struct accel_gyro_st_s calibration_data = {{0}};
	uint16_t calibrated_resultant;
	uint16_t resultant;
	int16_t accel_angle = 0;
	abs_angle_t absolute_angles = {0}; /* This "should" be initialized to something approperiate*/
	abs_angle_t absolute_accel_angles = {0};
	uint32_t prev;
	void *camera_buf;
	int img_w;
	int img_h;
	bool first_loop = true;
	pid_reg_t pid = {.error = 0, .p = 4, .i = 0, .d = 0};
	int64_t reg_result = 0;
	int16_t rotation;

	printf("Called board_bmi160_initialize: %d\n", ret);
	nximage_initialize();
	nxwindow_initialize();

	board_gpio_write(LED0, -1);
	board_gpio_config(LED0, 0, 0, true, PIN_FLOAT);
	board_gpio_write(LED3, -1);
	board_gpio_config(LED3, 0, 0, true, PIN_FLOAT);
	board_gpio_write(LED1, -1);
	board_gpio_config(LED1, 0, 0, true, PIN_FLOAT);
	board_gpio_write(LED2, -1);
	board_gpio_config(LED2, 0, 0, true, PIN_FLOAT);

#ifndef SIMULATION_MODE
	fd = open(ACC_DEVPATH, O_RDONLY);
#endif

	if (fd < 0) {
		printf("Device %s open failure. %d\n", ACC_DEVPATH, fd);
		return -1;
	}

	prev = 0;
	ret = calibration(&calibration_data, fd);

#ifndef SIMULATION_MODE
	ret = ioctl(fd, SNIOC_SET_GYRO_RANGE, range);
#endif

	printf("%c[?25h", ESC_CHAR); /* Hide cursor*/
	calibrated_resultant = accel_resultant(&calibration_data);

	calibration_data.accel.x = calibration_data.accel.x - (sin(X_ANGLE) * (1000 * INT16_MAX / 2000.0));
	calibration_data.accel.y = calibration_data.accel.y - (sin(Y_ANGLE) * (1000 * INT16_MAX / 2000.0));
	calibration_data.accel.z = calibration_data.accel.z - (cos(Z_ANGLE) * (1000 * INT16_MAX / 2000.0));

	printf("%c[?25l", ESC_CHAR); /* Hide cursor*/
	for (;;) {
		led(LED0, 0);
		led(LED1, 0);
		led(LED2, 0);
		led(LED3, 0);

#ifndef SIMULATION_MODE
		ret = read(fd, &data, sizeof(struct accel_gyro_st_s));
#else
		ret = sizeof(struct accel_gyro_st_s);
		data.sensor_time += 25000 / TIMER_TICK_US;
		data.gyro.x = 30;
		data.gyro.y = 5;
		data.gyro.z = 12;
#endif
		if (ret != sizeof(struct accel_gyro_st_s)) {
			fprintf(stderr, "Read failed.\n");
			break;
		}

		/* If sensing time has been changed, show 6 axis data. */
		if (prev != data.sensor_time) {

			printf("%c[0;0H", ESC_CHAR); /* Set cursor on 0x0 position */
			printf("[% 5d] % 7d, % 7d, % 7d / % 7d, % 7d, % 7d  \n", data.sensor_time, data.gyro.x, data.gyro.y, data.gyro.z, data.accel.x, data.accel.y,
				   data.accel.z);
			fflush(stdout);
			gyro_absolute_angle(&data, &absolute_angles);
			printf("Absolute angles: X=% 6lld, Y=% 6lld, Z=% 6lld \n", absolute_angles.x / FACTOR_LARGE, absolute_angles.y / FACTOR_LARGE,
				   absolute_angles.z / FACTOR_LARGE);
#define USE_Z_GYRO

#ifdef USE_Y_GYRO
			rotation = data.gyro.y;
#endif
#ifdef USE_Z_GYRO
			rotation = data.gyro.z;
#endif

			if (data.accel.z > 0) {
				if (rotation > -MAXIMUM_ROTATION_SPEED && rotation < MAXIMUM_ROTATION_SPEED) {
					resultant = accel_resultant(&data);
					if (resultant > calibrated_resultant - MAXIMUM_RESULTANT && resultant < calibrated_resultant + MAXIMUM_RESULTANT) {
#if defined(USE_Y_GYRO) && defined(USE_ACCEL_CALIB)

						accel_angle = accel_angle_xz(&calibration_data, &data, &absolute_accel_angles);
						pid.error = absolute_angles.y + (int64_t)accel_angle * FACTOR_LARGE / 100;
						reg_result = pid.error / 10 * pid.p;
						absolute_angles.y -= reg_result;

#endif

#if defined(USE_Z_GYRO) && defined(USE_ACCEL_CALIB)

						accel_angle = accel_angle_xy(&calibration_data, &data, &absolute_accel_angles);
						pid.error = absolute_angles.z + (int64_t)accel_angle * FACTOR_LARGE / 100;
						reg_result = pid.error / 10 * pid.p;
						absolute_angles.z -= reg_result;

#endif
						led(LED0, 1);
					} else {
						led(LED1, 1); /* Rotation speed to high indication for accel calibration */
					}
				} else {
					led(LED2, 1); /* Acceleration to high indication for accel calibration */
				}
			} else {
				led(LED3, 1); /* Leaning angle to high indication for accel calibration */
			}
			camera_get_image(&camera_buf, &img_w, &img_h);
			nx_window_redraw(&data, camera_buf, &absolute_angles);
			close_buf();
			if (first_loop) {

				printf("%c[J", ESC_CHAR);
				first_loop = false;
				memset(&absolute_angles, 0x00, sizeof(absolute_angles));
				continue;
			}
			prev = data.sensor_time;
			printf("%c[J", ESC_CHAR); /* ERASE DOWN */
		}
	}

	close(fd);

	return 0;
}

/**
 * Use this function as entry point in nuttx sdk configuration.
 */
int bmi160_graphical(int argc, char *argv[]) {
	(void)boardctl(BOARDIOC_INIT, 0); /* Needed to be able to use this function as the entry point in the SDK. */
	return bmi160_graphical_main(argc, argv);
}
