/****************************************************************************
 * bmi160_rotating/bmi160_rotating_main.c
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

#include <sdk/config.h>

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <sys/ioctl.h>

#include "nximage.h"
#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/sensors/bmi160.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_CAMERA_LCD_DEVNO
#define CONFIG_EXAMPLES_CAMERA_LCD_DEVNO 0
#endif

#define SIZEOF_B0 23
#define OFFSET_B0 0

#define SIZEOF_B1 13
#define OFFSET_B1 (OFFSET_B0 + SIZEOF_B0 * 2)

#define SIZEOF_B2 13
#define OFFSET_B2 (OFFSET_B1 + SIZEOF_B1 * 2)

#define SIZEOF_M 10
#define OFFSET_M (OFFSET_B2 + SIZEOF_B2 * 2)

#define SIZEOF_I 4
#define OFFSET_I (OFFSET_M + SIZEOF_M * 2)

#define SIZEOF_1 6
#define OFFSET_1 (OFFSET_I + SIZEOF_I * 2)

#define SIZEOF_60 41
#define OFFSET_60 (OFFSET_1 + SIZEOF_1 * 2)

#define SIZEOF_61 20
#define OFFSET_61 (OFFSET_60 + SIZEOF_60 * 2)

#define SIZEOF_00 20
#define OFFSET_00 (OFFSET_61 + SIZEOF_61 * 2)

#define SIZEOF_01 20
#define OFFSET_01 (OFFSET_00 + SIZEOF_00 * 2)

#define TOTAL_SIZE (OFFSET_01 + SIZEOF_01 * 2)

#define WIDTHOF_B 34.988258
#define WIDTHOF_M 40.000000
#define WIDTHOF_I 5.000000
#define WIDTHOF_1 20.000000
#define WIDTHOF_6 30.000000
#define WIDTHOF_0 30.000000

#define WIDTHOF_SPACE 5.000000

#define HEIGHTOF_TEXT 50
#define WIDTHOF_TEXT                                                                                                                                           \
	(WIDTHOF_B + WIDTHOF_SPACE + WIDTHOF_M + WIDTHOF_SPACE + WIDTHOF_I + WIDTHOF_SPACE + WIDTHOF_1 + WIDTHOF_SPACE + WIDTHOF_6 + WIDTHOF_SPACE + WIDTHOF_0)
#define DEPTHOF_TEXT 10

#define VIEW_PLANE 140

#define swap(a, b)                                                                                                                                             \
	{                                                                                                                                                          \
		int16_t t = a;                                                                                                                                         \
		a = b;                                                                                                                                                 \
		b = t;                                                                                                                                                 \
	}

#define ACC_DEVPATH "/dev/accel0"

#define FACTOR_LARGE 1000000
#define TIMER_TICK_US 10

#define RESET_VALUE 500
#define MIN_ROTATION_SPEED 20

#define ESC_CHAR 0x1B

#define ROTATION_SPEED_FACTOR 2

#define DEG_TO_RADIAN(d) (d * (3.14 / 180))
#define RADIAN_TO_DEG(r) r *(180 / 3.14)

#define LED0 PIN_I2S1_BCK
#define LED1 PIN_I2S1_LRCK
#define LED2 PIN_I2S1_DATA_IN
#define LED3 PIN_I2S1_DATA_OUT

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct {
	int64_t x;
	int64_t y;
	int64_t z;
} abs_angle_t;

typedef struct {
	float x;
	float y;
	float z;
} abs_angle_float_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct nximage_data_s g_nximage = {
	NULL,  /* hnx */
	NULL,  /* hbkgd */
	0,	 /* xres */
	0,	 /* yres */
	false, /* havpos */
	{0},   /* sem */
	0	  /* exit code */
};

static float tableofB0[] = {0.000000,  50.000000, 20.000000, 50.000000, 23.420428, 49.604817, 26.660630, 48.440090, 29.549876, 46.567189, 31.935929, 44.084801,
							33.693064, 41.123724, 34.728697, 37.839982, 34.988258, 34.406598, 34.458071, 31.004481, 33.166073, 27.812892, 31.180340, 25.000000,
							33.166073, 22.187108, 34.458071, 18.995519, 34.988258, 15.593402, 34.728697, 12.160018, 33.693064, 8.876276,  31.935929, 5.915199,
							29.549876, 3.432811,  26.660630, 1.559910,  23.420428, 0.395183,  20.000000, 0.000000,  0.000000,  0.000000};

static float tableofB1[] = {5.000000,  22.500000, 20.000000, 22.500000, 22.703899, 22.071745, 25.143121, 20.828899, 27.078899,
							18.893121, 28.321745, 16.453899, 28.750000, 13.750000, 28.321745, 11.046101, 27.078899, 8.606879,
							25.143121, 6.671101,  22.703899, 5.428255,  20.000000, 5.000000,  5.000000,  5.000000};

static float tableofB2[] = {5.000000,  45.000000, 20.000000, 45.000000, 22.703899, 44.571745, 25.143121, 43.328899, 27.078899,
							41.393121, 28.321745, 38.953899, 28.750000, 36.250000, 28.321745, 33.546101, 27.078899, 31.106879,
							25.143121, 29.171101, 22.703899, 27.928255, 20.000000, 27.500000, 5.000000,  27.500000};

static float tableofM[] = {0.000000,  50.000000, 20.000000, 30.000000, 40.000000, 50.000000, 40.000000, 0.000000, 35.000000, 0.000000,
						   35.000000, 37.928932, 20.000000, 22.928932, 5.000000,  37.928932, 5.000000,  0.000000, 0.000000,  0.000000};

static float tableofI[] = {0.000000, 50.000000, 5.000000, 50.000000, 5.000000, 0.000000, 0.000000, 0.000000};

static float tableof1[] = {0.000000, 30.000000, 20.000000, 50.000000, 20.000000, 0.000000, 15.000000, 0.000000, 15.000000, 37.928932, 3.535534, 26.464466};

static float tableof60[] = {0.000000,  35.000000, 0.734152,  10.635255, 2.864745,  43.816779, 6.183221,  47.135255, 10.364745, 49.265848, 15.000000, 50.000000,
							19.635255, 49.265848, 23.816779, 47.135255, 27.135255, 43.816779, 29.265848, 39.635255, 30.000000, 35.000000, 25.000000, 35.000000,
							24.510565, 38.090170, 23.090170, 40.877853, 20.877853, 43.090170, 18.090170, 44.510565, 15.000000, 45.000000, 11.909830, 44.510565,
							9.122147,  43.090170, 6.909830,  40.877853, 5.489435,  38.090170, 5.000000,  35.000000, 5.000000,  26.180340, 8.782575,  28.650774,
							13.129164, 29.882875, 17.645467, 29.764874, 21.921786, 28.307475, 25.570193, 25.642886, 28.259724, 22.012825, 29.746397, 17.746595,
							29.895347, 13.231207, 28.693064, 8.876276,  26.248612, 5.076859,  22.783740, 2.177621,  18.612765, 0.441568,  14.114057, 0.026186,
							9.695718,  0.969156,  5.758557,  3.184936,  2.659736,  6.472522,  0.680364,  10.533679, 0.000000,  15.000000};

static float tableof61[] = {5.489435,  11.909830, 6.909830,  9.122147,  9.122147,  6.909830,  11.909830, 5.489435,  15.000000, 5.000000,
							18.090170, 5.489435,  20.877853, 6.909830,  23.090170, 9.122147,  24.510565, 11.909830, 25.000000, 15.000000,
							24.510565, 18.090170, 23.090170, 20.877853, 20.877853, 23.090170, 18.090170, 24.510565, 15.000000, 25.000000,
							11.909830, 24.510565, 9.122147,  23.090170, 6.909830,  20.877853, 5.489435,  18.090170, 5.000000,  15.000000};

static float tableof00[] = {0.000000,  35.000000, 0.734152,  39.635255, 2.864745,  43.816779, 6.183221,  47.135255, 10.364745, 49.265848, 15.000000,
							50.000000, 19.635255, 49.265848, 23.816779, 47.135255, 27.135255, 43.816779, 29.265848, 39.635255, 30.000000, 35.000000,
							30.000000, 15.000000, 29.265848, 10.364745, 27.135255, 6.183221,  23.816779, 2.864745,  19.635255, 0.734152,  15.000000,
							0.000000,  10.364745, 0.734152,  6.183221,  2.864745,  2.864745,  6.183221,  0.734152,  10.364745, 0.000000,  15.000000};

static float tableof01[] = {5.000000,  35.000000, 5.489435,  38.090170, 6.909830,  40.877853, 9.122147,  43.090170, 11.909830, 44.510565, 15.000000,
							45.000000, 18.090170, 44.510565, 20.877853, 43.090170, 23.090170, 40.877853, 24.510565, 38.090170, 25.000000, 35.000000,
							25.000000, 15.000000, 24.510565, 11.909830, 23.090170, 9.122147,  20.877853, 6.909830,  18.090170, 5.489435,  15.000000,
							5.000000,  11.909830, 5.489435,  9.122147,  6.909830,  6.909830,  9.122147,  5.489435,  11.909830, 5.000000,  15.000000};

enum {
	x,
	y,
	z,
} xyz;

static uint16_t buff[320 * 240];
static float text3d[TOTAL_SIZE][3];
static float text3d_initial[TOTAL_SIZE][3];
static uint16_t text2d[TOTAL_SIZE][2];

static int16_t z_offset = 150;
static int16_t x_offset = 0;
static int16_t y_offset = 0;

static BMI160_gyro_range_t range = BMI160_GYRO_RANGE_2000;

static int32_t last_sensor_time = 0;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int nximage_initialize(void) {
	FAR NX_DRIVERTYPE *dev;
	nxgl_mxpixel_t color;
	int ret;

	/* Initialize the LCD device */

	printf("nximage_initialize: Initializing LCD\n");
	ret = board_lcd_initialize();
	if (ret < 0) {
		printf("nximage_initialize: board_lcd_initialize failed: %d\n", -ret);
		return ERROR;
	}

	/* Get the device instance */

	dev = board_lcd_getdev(CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
	if (!dev) {
		printf("nximage_initialize: board_lcd_getdev failed, devno=%d\n", CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
		return ERROR;
	}

	/* Turn the LCD on at 75% power */

	(void)dev->setpower(dev, ((3 * CONFIG_LCD_MAXPOWER + 3) / 4));

	/* Then open NX */

	printf("nximage_initialize: Open NX\n");
	g_nximage.hnx = nx_open(dev);
	if (!g_nximage.hnx) {
		printf("nximage_initialize: nx_open failed: %d\n", errno);
		return ERROR;
	}

	/* Set background color to white */

	color = 0xffff;
	nx_setbgcolor(g_nximage.hnx, &color);
	ret = nx_requestbkgd(g_nximage.hnx, &g_nximagecb, NULL);
	if (ret < 0) {
		printf("nximage_initialize: nx_requestbkgd failed: %d\n", errno);
		nx_close(g_nximage.hnx);
		return ERROR;
	}

	while (!g_nximage.havepos) {
		(void)sem_wait(&g_nximage.sem);
	}
	printf("nximage_initialize: Screen resolution (%d,%d)\n", g_nximage.xres, g_nximage.yres);

	return 0;
}

static void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
	int16_t dx, dy;
	int16_t err;
	int16_t ystep;
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);

	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	dx = x1 - x0;
	dy = abs(y1 - y0);
	err = dx / 2;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0 <= x1; x0++) {
		if (steep) {
			buff[x0 * g_nximage.xres + y0] = color;
		} else {
			buff[y0 * g_nximage.xres + x0] = color;
		}

		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

static void calculate_2dpoints(void) {
	float tmp1, tmp2;

	for (uint16_t i = 0; i < TOTAL_SIZE; i++) {
		tmp1 = ((text3d[i][0] * VIEW_PLANE / text3d[i][2]) + (g_nximage.xres / 2));
		tmp2 = ((text3d[i][1] * VIEW_PLANE / text3d[i][2]) + (g_nximage.yres / 2));
		text2d[i][0] = (uint16_t)tmp1;
		text2d[i][1] = (uint16_t)tmp2;
	}
}

static void draw_element(uint8_t size, uint16_t offset) {
	for (uint8_t i = 0; i < size - 1; i++) {
		draw_line(text2d[i + offset][0], text2d[i + offset][1], text2d[i + 1 + offset][0], text2d[i + 1 + offset][1], 0x0000);
	}

	draw_line(text2d[size - 1 + offset][0], text2d[size - 1 + offset][1], text2d[0 + offset][0], text2d[0 + offset][1], 0x0000);

	for (uint8_t i = 0; i < size; i++) {
		draw_line(text2d[i + offset][0], text2d[i + offset][1], text2d[i + size + offset][0], text2d[i + size + offset][1], 0x0000);
	}

	for (uint8_t i = 0; i < size - 1; i++) {
		draw_line(text2d[i + size + offset][0], text2d[i + size + offset][1], text2d[i + 1 + size + offset][0], text2d[i + 1 + size + offset][1], 0x0000);
	}

	draw_line(text2d[size - 1 + size + offset][0], text2d[size - 1 + size + offset][1], text2d[0 + size + offset][0], text2d[0 + size + offset][1], 0x0000);
}

static void add_element(float *table, uint8_t size, uint16_t offset, float offset_x) {
	int i = 0;

	for (i = 0; i < size; i++) {
		text3d[i + offset][0] = table[i * 2] - WIDTHOF_TEXT / 2 + offset_x;
		text3d[i + offset][1] = HEIGHTOF_TEXT - table[i * 2 + 1] - HEIGHTOF_TEXT / 2;
		text3d[i + offset][2] = DEPTHOF_TEXT / 2 + z_offset;
		text3d[i + size + offset][0] = table[i * 2] - WIDTHOF_TEXT / 2 + offset_x;
		text3d[i + size + offset][1] = HEIGHTOF_TEXT - table[i * 2 + 1] - HEIGHTOF_TEXT / 2;
		text3d[i + size + offset][2] = -DEPTHOF_TEXT / 2 + z_offset;
	}
}

static void zrotate(float sin_q, float cos_q) {
	float tx, ty, temp;

	for (uint16_t i = 0; i < TOTAL_SIZE; i++) {
		tx = text3d[i][0] - x_offset;
		ty = text3d[i][1] - y_offset;

		temp = tx * cos_q - ty * sin_q;
		ty = tx * sin_q + ty * cos_q;
		tx = temp;

		text3d[i][0] = tx + x_offset;
		text3d[i][1] = ty + y_offset;
	}
}

static void yrotate(float sin_q, float cos_q) {
	float tx, tz, temp;

	for (uint16_t i = 0; i < TOTAL_SIZE; i++) {
		tx = text3d_initial[i][0] - x_offset;
		tz = text3d[i][2] - z_offset;

		temp = tz * cos_q - tx * sin_q;
		tx = tz * sin_q + tx * cos_q;
		tz = temp;

		text3d[i][0] = tx + x_offset;
		text3d[i][2] = tz + z_offset;
	}
}

static void xrotate(float sin_q, float cos_q) {
	float ty, tz, temp;

	for (uint16_t i = 0; i < TOTAL_SIZE; i++) {
		ty = text3d_initial[i][1] - y_offset;
		tz = text3d_initial[i][2] - z_offset;

		temp = ty * cos_q - tz * sin_q;
		tz = ty * sin_q + tz * cos_q;
		ty = temp;

		text3d[i][1] = ty + y_offset;
		text3d[i][2] = tz + z_offset;
	}
}

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

static int64_t gyro_degrees(int16_t gyroval, int32_t dtime) {
	int32_t dtime_us;
	int64_t angle_mdegrees;

	dtime_us = (dtime * TIMER_TICK_US);

	angle_mdegrees = gyro_rate_mdeg_per_sec(gyroval) * dtime_us;

	return angle_mdegrees;
}
inline static int angle_limit(int64_t *angle) {
	if (*angle > 360 * FACTOR_LARGE) {
		*angle = *angle - 360 * FACTOR_LARGE;
	}
	if (*angle < -360 * FACTOR_LARGE) {
		*angle = *angle + 360 * FACTOR_LARGE;
	}
}

static int gyro_absolute_angle(struct accel_gyro_st_s *data, abs_angle_t *angle) {
	int32_t d_time;

	d_time = data->sensor_time - last_sensor_time;

	angle->x += gyro_degrees(-data->gyro.x, d_time);
	angle->y += gyro_degrees(-data->gyro.y, d_time);
	angle->z += gyro_degrees(-data->gyro.z, d_time);

	angle_limit(&angle->x);
	angle_limit(&angle->y);
	angle_limit(&angle->z);

	last_sensor_time = data->sensor_time;

	return 0;
}

void reset_image(abs_angle_t *absolute_angles) {
	absolute_angles->x = 0;
	absolute_angles->y = 0;
	absolute_angles->z = 0;

	add_element(tableofB0, SIZEOF_B0, OFFSET_B0, 0);
	add_element(tableofB1, SIZEOF_B1, OFFSET_B1, 0);
	add_element(tableofB2, SIZEOF_B2, OFFSET_B2, 0);
	add_element(tableofM, SIZEOF_M, OFFSET_M, WIDTHOF_B + WIDTHOF_SPACE);
	add_element(tableofI, SIZEOF_I, OFFSET_I, WIDTHOF_B + WIDTHOF_SPACE + WIDTHOF_M + WIDTHOF_SPACE);
	add_element(tableof1, SIZEOF_1, OFFSET_1, WIDTHOF_B + WIDTHOF_SPACE + WIDTHOF_M + WIDTHOF_SPACE + WIDTHOF_I + WIDTHOF_SPACE);
	add_element(tableof60, SIZEOF_60, OFFSET_60, WIDTHOF_B + WIDTHOF_SPACE + WIDTHOF_M + WIDTHOF_SPACE + WIDTHOF_I + WIDTHOF_SPACE + WIDTHOF_1 + WIDTHOF_SPACE);
	add_element(tableof61, SIZEOF_61, OFFSET_61, WIDTHOF_B + WIDTHOF_SPACE + WIDTHOF_M + WIDTHOF_SPACE + WIDTHOF_I + WIDTHOF_SPACE + WIDTHOF_1 + WIDTHOF_SPACE);
	add_element(tableof00, SIZEOF_00, OFFSET_00,
				WIDTHOF_B + WIDTHOF_SPACE + WIDTHOF_M + WIDTHOF_SPACE + WIDTHOF_I + WIDTHOF_SPACE + WIDTHOF_1 + WIDTHOF_SPACE + WIDTHOF_6 + WIDTHOF_SPACE);
	add_element(tableof01, SIZEOF_01, OFFSET_01,
				WIDTHOF_B + WIDTHOF_SPACE + WIDTHOF_M + WIDTHOF_SPACE + WIDTHOF_I + WIDTHOF_SPACE + WIDTHOF_1 + WIDTHOF_SPACE + WIDTHOF_6 + WIDTHOF_SPACE);

	calculate_2dpoints();
	memset(buff, 0xffff, 320 * 240 * 2);
	draw_element(SIZEOF_B0, OFFSET_B0);
	draw_element(SIZEOF_B1, OFFSET_B1);
	draw_element(SIZEOF_B2, OFFSET_B2);
	draw_element(SIZEOF_M, OFFSET_M);
	draw_element(SIZEOF_I, OFFSET_I);
	draw_element(SIZEOF_1, OFFSET_1);
	draw_element(SIZEOF_60, OFFSET_60);
	draw_element(SIZEOF_61, OFFSET_61);
	draw_element(SIZEOF_00, OFFSET_00);
	draw_element(SIZEOF_01, OFFSET_01);
	nximage_image(g_nximage.hbkgd, (void *)buff);
}

static void led(int led, bool state) { board_gpio_write(led, state); }
/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int bmi160_rotating_main(int argc, FAR char *argv[])
#else
int bmi160_rotating_main(int argc, char *argv[])
#endif
{
	int ret;
	float angle, sin_angle, cos_angle;
	int exitcode = ERROR;
	int fd = 0;
	uint32_t prev = 0;
	uint32_t reset_counter = 0;
	struct accel_gyro_st_s data = {{0}};
	abs_angle_t absolute_angles = {0};
        (void)boardctl(BOARDIOC_INIT, 0); /* Needed to be able to use this function as the entry point in the SDK. */
	abs_angle_float_t current_angles = {0};

	board_gpio_write(LED0, -1);
	board_gpio_config(LED0, 0, 0, true, PIN_FLOAT);
	board_gpio_write(LED3, -1);
	board_gpio_config(LED3, 0, 0, true, PIN_FLOAT);
	board_gpio_write(LED1, -1);
	board_gpio_config(LED1, 0, 0, true, PIN_FLOAT);
	board_gpio_write(LED2, -1);
	board_gpio_config(LED2, 0, 0, true, PIN_FLOAT);

	fd = open(ACC_DEVPATH, O_RDONLY);
	if (fd < 0) {
		printf("Device %s open failure. %d\n", ACC_DEVPATH, fd);
		return -1;
	}

	ret = ioctl(fd, SNIOC_SET_GYRO_RANGE, range);

	ret = nximage_initialize();
	if (ret < 0) {
		printf("bmi160_rotation_main: Failed to get NX handle: %d\n", errno);
		return ERROR;
	}

	reset_image(&absolute_angles);

	ret = read(fd, &data, sizeof(struct accel_gyro_st_s));
	if (ret != sizeof(struct accel_gyro_st_s)) {
		fprintf(stderr, "Read failed.\n");
	}

	last_sensor_time = data.sensor_time;
	printf("%c[?25h", ESC_CHAR); /* Hide cursor*/
	printf("%c[?25l", ESC_CHAR); /* Hide cursor*/
	printf("%c[0;0H", ESC_CHAR); /* Set cursor on 0x0 position */
	printf("%c[J", ESC_CHAR);	/* ERASE DOWN */
	memcpy(text3d_initial, text3d, sizeof(text3d_initial));

	for (;;) {
		led(LED0, 0);
		led(LED1, 0);
		led(LED2, 0);
		led(LED3, 0);
		ret = read(fd, &data, sizeof(struct accel_gyro_st_s));
		if (ret != sizeof(struct accel_gyro_st_s)) {
			fprintf(stderr, "Read failed.\n");
			break;
		}

		if (prev != data.sensor_time) {
			printf("%c[0;0H", ESC_CHAR); /* Set cursor on 0x0 position */
			printf("[%07d] % 6d, % 6d, % 6d / % 6d, % 6d, % 6d       \n", data.sensor_time, data.gyro.x, data.gyro.y, data.gyro.z, data.accel.x, data.accel.y,
				   data.accel.z);
			fflush(stdout);
			led(LED2, 1);

			if (data.gyro.x > -MIN_ROTATION_SPEED && data.gyro.x < MIN_ROTATION_SPEED && data.gyro.y > -MIN_ROTATION_SPEED &&
				data.gyro.y < MIN_ROTATION_SPEED && data.gyro.z > -MIN_ROTATION_SPEED && data.gyro.z < MIN_ROTATION_SPEED) {
				led(LED0, 1);
				absolute_angles.x -= absolute_angles.x / 10;
				absolute_angles.y -= absolute_angles.y / 10;
				absolute_angles.z -= absolute_angles.z / 10;

			} else {
				led(LED1, 1);
				data.gyro.x = data.gyro.x * ROTATION_SPEED_FACTOR;
				data.gyro.y = data.gyro.y * ROTATION_SPEED_FACTOR;
				data.gyro.z = data.gyro.z * ROTATION_SPEED_FACTOR;
				gyro_absolute_angle(&data, &absolute_angles);

				reset_counter = 0;
			}

			printf("Absolute angles: X=% 6lld, Y=% 6lld, Z=% 6lld\n\n", absolute_angles.x / FACTOR_LARGE, absolute_angles.y / FACTOR_LARGE,
				   absolute_angles.z / FACTOR_LARGE);

			angle = (float)absolute_angles.x / FACTOR_LARGE * M_PI_F / 180;

			{
				sin_angle = sin(angle);
				cos_angle = cos(angle);
				xrotate(sin_angle, cos_angle);
				current_angles.x = angle;
			}

			angle = (float)absolute_angles.y / FACTOR_LARGE * M_PI_F / 180;

			{
				sin_angle = sin(-1 * (angle));
				cos_angle = cos(-1 * (angle));
				yrotate(sin_angle, cos_angle);
				current_angles.y = angle;
			}

			angle = (float)absolute_angles.z / FACTOR_LARGE * M_PI_F / 180;

			{
				sin_angle = sin(-1 * (angle));
				cos_angle = cos(-1 * (angle));
				zrotate(sin_angle, cos_angle);
				current_angles.z = angle;
			}

			calculate_2dpoints();
			memset(buff, 0xffff, 320 * 240 * 2);
			draw_element(SIZEOF_B0, OFFSET_B0);
			draw_element(SIZEOF_B1, OFFSET_B1);
			draw_element(SIZEOF_B2, OFFSET_B2);
			draw_element(SIZEOF_M, OFFSET_M);
			draw_element(SIZEOF_I, OFFSET_I);
			draw_element(SIZEOF_1, OFFSET_1);
			draw_element(SIZEOF_60, OFFSET_60);
			draw_element(SIZEOF_61, OFFSET_61);
			draw_element(SIZEOF_00, OFFSET_00);
			draw_element(SIZEOF_01, OFFSET_01);
			nximage_image(g_nximage.hbkgd, (void *)buff);

			prev = data.sensor_time;
			printf("%c[J", ESC_CHAR); /* ERASE DOWN */
		} else {
			led(LED2, 2);
		}
	};

	exitcode = OK;

	nx_close(g_nximage.hnx);
	close(fd);

	return exitcode;
}

/**
 * Use this function as entry point in nuttx sdk configuration.
 */
int bmi160_rotating(int argc, char *argv[]) {
	(void)boardctl(BOARDIOC_INIT, 0); /* Needed to be able to use this function as the entry point in the SDK. */
	return bmi160_rotating_main(argc, argv);
}
