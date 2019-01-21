#include <sdk/config.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxfonts.h>
#include <nuttx/nx/nxglib.h>

//#include "nximage.h"

struct nximage_data_s g_nximage = {
	NULL,  /* hnx */
	NULL,  /* hbkgd */
	0,	 /* xres */
	0,	 /* yres */
	false, /* havpos */
	{0},   /* sem */
	0	  /* exit code */
};

static inline int nximage_initialize() {
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

	color = 0;
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
