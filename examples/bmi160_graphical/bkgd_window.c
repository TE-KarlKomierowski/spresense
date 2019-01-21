/****************************************************************************
 * camera/camera_bkgd.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <debug.h>
#include <errno.h>
#include <math.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxfonts.h>
#include <nuttx/nx/nxglib.h>

#include "bmi160_graphical.h"
#include "nximage.h"

#ifndef CONFIG_EXAMPLES_CAMERA_LCD_DEVNO
#define CONFIG_EXAMPLES_CAMERA_LCD_DEVNO 0
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define WINDOW_BG_COLOR 0xFFFF

#define GRAPH_ACCEL_OFFSET 20
#define GRAPH_GYRO_OFFSET -20

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

#if 1
#define WINDOW_CAMERA_WIDTH SCREEN_WIDTH / 2
#define WINDOW_CAMERA_HEIGHT SCREEN_HEIGHT / 2
#else
#define WINDOW_CAMERA_WIDTH 200
#define WINDOW_CAMERA_HEIGHT 180
#endif

#define WINDOW_GRAPH_WIDTH (WINDOW_CAMERA_WIDTH)
#define WINDOW_GRAPH_HEIGHT (WINDOW_CAMERA_HEIGHT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nximage_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect, bool more, FAR void *arg);
static void nximage_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size, FAR const struct nxgl_point_s *pos, FAR const struct nxgl_rect_s *bounds,
							 FAR void *arg);
#ifdef CONFIG_NX_XYINPUT
static void nximage_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos, uint8_t buttons, FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
static void nximage_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch, FAR void *arg);
#endif

void *fb;

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct nximage_data_s g_nximage = {
	NULL,  /* hnx */
	NULL,  /* hbkgd */
	NULL,  /* win*/
	NULL,  /* camwin*/
	0,	 /* xres */
	0,	 /* yres */
	false, /* havpos */
	{0},   /* sem */
	0	  /* exit code */
};

NXWINDOW graph_wnd;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Background window call table */

const struct nx_callback_s g_nximagecb = {
	nximage_redraw,  /* redraw */
	nximage_position /* position */
#ifdef CONFIG_NX_XYINPUT
	,
	nximage_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
	,
	nximage_kbdin /* my kbdin */
#endif
};
const struct nx_callback_s g_nxwincb = {
	nximage_redraw,  /* redraw */
	nximage_position /* position */
#ifdef CONFIG_NX_XYINPUT
	,
	nximage_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
	,
	nximage_kbdin /* my kbdin */
#endif
};

struct nxgl_size_s graph_window_size = {
	.w = WINDOW_GRAPH_WIDTH,  /* Width in pixels */
	.h = WINDOW_GRAPH_HEIGHT, /* Height in rows */
};

struct nxgl_size_s camera_window_size = {
	.w = WINDOW_CAMERA_WIDTH,  /* Width in pixels */
	.h = WINDOW_CAMERA_HEIGHT, /* Height in rows */
};

FAR const struct nxgl_rect_s rectfill = {{.x = 0, .y = 0}, {.x = WINDOW_GRAPH_WIDTH, .y = WINDOW_GRAPH_HEIGHT}};

FAR struct nxgl_point_s graph_window_pos = {.x = (SCREEN_WIDTH / 2) - (WINDOW_GRAPH_WIDTH / 2), .y = SCREEN_HEIGHT - WINDOW_GRAPH_HEIGHT};

FAR struct nxgl_point_s camera_window_pos = {.x = (SCREEN_WIDTH / 2) - (WINDOW_CAMERA_WIDTH / 2), .y = 0};

FAR struct nxgl_point_s circle_center = {.x = 5, .y = 5};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_redraw
 *
 * Description:
 *   NX re-draw handler
 *
 ****************************************************************************/

static void nximage_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect, bool more, FAR void *arg) {
	printf("%s hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n", __func__, hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y, more ? "true" : "false");
#if 0
        nxgl_mxpixel_t c;
        c = 0x00f0;

        nx_drawcircle(hwnd, FAR &circle_center, 10, 1, &c);
        nx_fill(g_nximage.win, &rectfill, &c);
#endif
}

/****************************************************************************
 * Name: nximage_position
 *
 * Description:
 *   NX position change handler
 *
 ****************************************************************************/

static void nximage_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size, FAR const struct nxgl_point_s *pos, FAR const struct nxgl_rect_s *bounds,
							 FAR void *arg) {
	/* Report the position */

	printf("%s hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n", __func__, hwnd, size->w, size->h, pos->x, pos->y, bounds->pt1.x, bounds->pt1.y,
		   bounds->pt2.x, bounds->pt2.y);

	/* Have we picked off the window bounds yet? */

	if (!g_nximage.havepos) {
		/* Save the background window handle */

		g_nximage.hbkgd = hwnd;

		/* Save the window limits */

		g_nximage.xres = bounds->pt2.x + 1;
		g_nximage.yres = bounds->pt2.y + 1;

		g_nximage.havepos = true;
		sem_post(&g_nximage.sem);
		printf("Have xres=%d yres=%d\n", g_nximage.xres, g_nximage.yres);
	}
}

/****************************************************************************
 * Name: nximage_mousein
 *
 * Description:
 *   NX mouse input handler
 *
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
static void nximage_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos, uint8_t buttons, FAR void *arg) {
	printf("nximage_mousein: hwnd=%p pos=(%d,%d) button=%02x\n", hwnd, pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nximage_kbdin
 *
 * Description:
 *   NX keyboard input handler
 *
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nximage_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch, FAR void *arg) {
	ginfo("hwnd=%p nch=%d\n", hwnd, nch);

	/* In this example, there is no keyboard so a keyboard event is not
	 * expected.
	 */

	printf("nximage_kbdin: Unexpected keyboard callback\n");
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_image
 *
 * Description:
 *   Put the NuttX logo in the center of the display.
 *
 ****************************************************************************/

void nximage_image(NXWINDOW hwnd, FAR const void *image) {
	FAR struct nxgl_point_s origin;
	FAR struct nxgl_rect_s dest;
	FAR const void *src[CONFIG_NX_NPLANES];
	int ret;

	origin.x = 0;
	origin.y = 0;

	/* Set up the destination to whole LCD screen */

	dest.pt1.x = 0;
	dest.pt1.y = 0;
	dest.pt2.x = g_nximage.xres - 1;
	dest.pt2.y = g_nximage.yres - 1;

	src[0] = image;

	// ret = nx_bitmap((NXWINDOW)g_nximage.win, &dest, src, &origin, g_nximage.xres * sizeof(nxgl_mxpixel_t));
	printf("nximage_image: hwnd %d, %p %p %p %d\n", hwnd, &dest, src[0], &origin, g_nximage.xres * sizeof(nxgl_mxpixel_t));
	ret = nx_bitmap((NXWINDOW)hwnd, &dest, src, &origin, g_nximage.xres * sizeof(nxgl_mxpixel_t));
	if (ret < 0) {
		printf("nximage_image: nx_bitmapwindow failed: %d\n", errno);
	}
}

int nximage_initialize(void) {
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

	/* Set background color to black */

	color = WINDOW_BG_COLOR;
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
	fb = malloc(g_nximage.xres * g_nximage.yres * sizeof(uint16_t));
	printf("nximage_initialize: Screen resolution (%d,%d)\n", g_nximage.xres, g_nximage.yres);

	return 0;
}

FAR static nxgl_mxpixel_t fbw[WINDOW_GRAPH_WIDTH * WINDOW_GRAPH_HEIGHT] = {0xffff};
FAR static nxgl_mxpixel_t fbc[WINDOW_CAMERA_WIDTH * WINDOW_CAMERA_HEIGHT] = {0xffff};

void nxwindow_initialize() {
	int ret;
	nxgl_mxpixel_t color;

	/* Create the graph window */
	printf("%s: \n", __func__);
	g_nximage.win = nx_openwindow(g_nximage.hnx, &g_nxwincb, NULL);

	ret = nx_setsize(g_nximage.win, &graph_window_size);
	if (ret) {
		printf("%s: nx_setsize: %d\n", __func__, ret);
	}

	ret = nx_setposition(g_nximage.win, &graph_window_pos);
	if (ret) {
		printf("%s: nx_setposition: %d\n", __func__, ret);
	}
	color = 0x00f0;
	ret = nx_setbgcolor(g_nximage.win, &color);
	if (ret) {
		printf("%s: nx_setbgcolor: %d\n", __func__, ret);
	}

	memset(fbw, WINDOW_BG_COLOR, WINDOW_GRAPH_WIDTH * WINDOW_GRAPH_HEIGHT * sizeof(nxgl_mxpixel_t));

	/* Create the camera window */
	g_nximage.camwin = nx_openwindow(g_nximage.hnx, &g_nxwincb, NULL);

	ret = nx_setsize(g_nximage.camwin, &camera_window_size);
	if (ret) {
		printf("%s: nx_setsize: %d\n", __func__, ret);
	}

	ret = nx_setposition(g_nximage.camwin, &camera_window_pos);
	if (ret) {
		printf("%s: nx_setposition: %d\n", __func__, ret);
	}
	color = 0x00fF;
	ret = nx_setbgcolor(g_nximage.camwin, &color);
	if (ret) {
		printf("%s: nx_setbgcolor: %d\n", __func__, ret);
	}

	memset(fbc, WINDOW_BG_COLOR, WINDOW_CAMERA_WIDTH * WINDOW_CAMERA_HEIGHT * sizeof(nxgl_mxpixel_t));
}

int nx_window_push_fb(void *img_buf, abs_angle_t *angles) {
	int ret = ERROR;
	int16_t x, y, x_new, y_new, cameraImgWidth, cameraImgHeight;
	float x_tmp, y_tmp, sin_angle, cos_angle;
	uint16_t *cameraImgBuff;
	uint16_t temp_px;

	static float angle = 0.0f;

	FAR struct nxgl_point_s origin_cam;
	FAR struct nxgl_point_s origin;
	FAR struct nxgl_rect_s dest;
	FAR struct nxgl_rect_s dest_cam;

	origin.x = 0;
	origin.y = 0;

	origin_cam.x = 0; // 320/2-WINDOW_CAMERA_WIDTH/2;
	origin_cam.y = 0; // 240/2 - WINDOW_CAMERA_HEIGHT/2 ;

	dest.pt1.x = 0;
	dest.pt1.y = 0;
	dest.pt2.x = WINDOW_GRAPH_WIDTH - 1;
	dest.pt2.y = WINDOW_GRAPH_HEIGHT - 1;

	dest_cam.pt1.x = 0;
	dest_cam.pt1.y = 0;
	dest_cam.pt2.x = WINDOW_CAMERA_WIDTH - 1;
	dest_cam.pt2.y = WINDOW_CAMERA_HEIGHT - 1;

	// dest_cam.pt2.x = 320 - 1;
	// dest_cam.pt2.y = 240 - 1;

	FAR const void *src[CONFIG_NX_NPLANES];
	FAR const void *src_img[CONFIG_NX_NPLANES];

	src[0] = (const void *)fbw;

	cameraImgBuff = (uint16_t *)img_buf;
	cameraImgWidth = 320;
	cameraImgHeight = 240;

	angle = (float)angles->z / FACTOR_LARGE;
	angle = angle * 3.14 / 180;

	// printf("Angles: z: %lld)

	sin_angle = sinf(angle);
	cos_angle = cosf(angle);

	for (y = 0; y < WINDOW_CAMERA_HEIGHT; y++) {
		for (x = 0; x < WINDOW_CAMERA_WIDTH; x++) {

			x_tmp = (((-WINDOW_CAMERA_WIDTH / 2) + x) * cos_angle - ((-WINDOW_CAMERA_HEIGHT / 2) + y) * sin_angle) + (cameraImgWidth / 2);
			y_tmp = (((-WINDOW_CAMERA_WIDTH / 2) + x) * sin_angle + ((-WINDOW_CAMERA_HEIGHT / 2) + y) * cos_angle) + (cameraImgHeight / 2);

			x_new = (int16_t)(x_tmp);
			y_new = (int16_t)(y_tmp);

			fbc[y * WINDOW_CAMERA_WIDTH + (WINDOW_CAMERA_WIDTH - x)] = cameraImgBuff[y_new * cameraImgWidth + x_new + cameraImgWidth];
			fbw[y * WINDOW_CAMERA_WIDTH + (WINDOW_CAMERA_WIDTH - x)] =
				cameraImgBuff[(WINDOW_CAMERA_HEIGHT / 2 + y) * cameraImgWidth + x + (WINDOW_CAMERA_WIDTH / 2)];
		}
	}

	src_img[0] = (const void *)fbc;

	src[0] = (const void *)fbw;
	ret = nx_bitmap((NXWINDOW)g_nximage.win, &dest, src, &origin_cam, WINDOW_CAMERA_WIDTH * sizeof(nxgl_mxpixel_t));
	// ret = nx_bitmap((NXWINDOW)g_nximage.win, &dest, src, &origin, WINDOW_GRAPH_WIDTH * sizeof(nxgl_mxpixel_t));
	ret = nx_bitmap((NXWINDOW)g_nximage.camwin, &dest_cam, src_img, &origin_cam, WINDOW_CAMERA_WIDTH * sizeof(nxgl_mxpixel_t));

	return ret;
}

void window_update_pt(nxgl_mxpixel_t *const fbdata, const int height, const int width, const struct accel_gyro_st_s *data) {
	int x, y;

	for (y = 0; y < height; y++) {
		for (x = 1; x < width; x++) {
			fbdata[y * width + x - 1] = fbdata[y * width + x];
		}
	}
	for (y = 0; y < height; y++) {
		fbdata[y * width + (width - 1)] = WINDOW_BG_COLOR;
	}
#if 0
	fbdata[(GRAPH_ACCEL_OFFSET + data->accel.x + (height / 2)) * width + (width - 1)] = 0x0000;
	fbdata[(GRAPH_ACCEL_OFFSET + data->accel.y + (height / 2)) * width + (width - 1)] = 0x0011;
	fbdata[(GRAPH_ACCEL_OFFSET + data->accel.z + (height / 2)) * width + (width - 1)] = 0x0022;

	fbdata[(GRAPH_GYRO_OFFSET + data->gyro.x + (height / 2)) * width + (width - 1)] = 0x3300;
	fbdata[(GRAPH_GYRO_OFFSET + data->gyro.y + (height / 2)) * width + (width - 1)] = 0x0440;
	fbdata[(GRAPH_GYRO_OFFSET + data->gyro.z + (height / 2)) * width + (width - 1)] = 0x5500;
#endif

	// fbdata[(GRAPH_ACCEL_OFFSET + 6) * width + (width - 1)] = 0x0011;
	// fbdata[(GRAPH_ACCEL_OFFSET + 10) * width + (width - 1)] = 0x0022;
}

void nx_window_redraw(struct accel_gyro_st_s *data, void *img_buf, abs_angle_t *angles) {

	window_update_pt((nxgl_mxpixel_t * const)fbw, WINDOW_GRAPH_HEIGHT, WINDOW_GRAPH_WIDTH, data);
	nx_window_push_fb(img_buf, angles);
}
