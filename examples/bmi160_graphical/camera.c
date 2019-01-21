/****************************************************************************
 * camera/camera_main.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "video/video.h"
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/mkfatfs.h>

#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <sys/mount.h>

#include <arch/board/board.h>
#include <arch/chip/cisif.h>
#include <arch/chip/pm.h>

#include <imageproc/imageproc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Display of vsync timing */
/* #define CAMERA_MAIN_CISIF_INTRTRACE */

/* Note: Buffer size must be multiple of 32. */

#define IMAGE_JPG_SIZE (512 * 1024)	/* 512kB */
#define IMAGE_YUV_SIZE (320 * 240 * 2) /* QVGA YUV422 */

#define VIDEO_BUFNUM (2)
#define STILL_BUFNUM (1)

#define DEFAULT_REPEAT_NUM (10)

#define IMAGE_FILENAME_LEN (32)



#define itou8(v) ((v) < 0 ? 0 : ((v) > 255 ? 255 : (v)))


/****************************************************************************
 * Private Types
 ****************************************************************************/
struct uyvy_s {
	uint8_t u0;
	uint8_t y0;
	uint8_t v0;
	uint8_t y1;
};

struct v_buffer {
	uint32_t *start;
	uint32_t length;
};
typedef struct v_buffer v_buffer_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct v_buffer *buffers_video;
static struct v_buffer *buffers_still;
static unsigned int n_buffers;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int camera_prepare(int fd, enum v4l2_buf_type type, uint32_t buf_mode, uint32_t pixformat, uint16_t hsize, uint16_t vsize, uint8_t buffernum) {
	int ret;
	int cnt;
	uint32_t fsize;
	struct v4l2_format fmt = {0};
	struct v4l2_requestbuffers req = {0};
	struct v4l2_buffer buf = {0};
	struct v_buffer *buffers;

	/* VIDIOC_REQBUFS initiate user pointer I/O */

	req.type = type;
	req.memory = V4L2_MEMORY_USERPTR;
	req.count = buffernum;
	req.mode = buf_mode;

	ret = ioctl(fd, VIDIOC_REQBUFS, (unsigned long)&req);
	if (ret < 0) {
		printf("Failed to VIDIOC_REQBUFS: errno = %d\n", errno);
		return ret;
	}

	/* VIDIOC_S_FMT set format */

	fmt.type = type;
	fmt.fmt.pix.width = hsize;
	fmt.fmt.pix.height = vsize;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;
	fmt.fmt.pix.pixelformat = pixformat;

	ret = ioctl(fd, VIDIOC_S_FMT, (unsigned long)&fmt);
	if (ret < 0) {
		printf("Failed to VIDIOC_S_FMT: errno = %d\n", errno);
		return ret;
	}

	/* VIDIOC_QBUF enqueue buffer */

	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		buffers_video = malloc(sizeof(v_buffer_t) * buffernum);
		buffers = buffers_video;
	}

	if (!buffers) {
		printf("Out of memory\n");
		return ret;
	}

	if (pixformat == V4L2_PIX_FMT_UYVY) {
		fsize = IMAGE_YUV_SIZE;
	} else {
		fsize = IMAGE_JPG_SIZE;
	}

	for (n_buffers = 0; n_buffers < buffernum; ++n_buffers) {
		buffers[n_buffers].length = fsize;

		/* Note: VIDIOC_QBUF set buffer pointer. */
		/*       Buffer pointer must be 32bytes aligned. */

		buffers[n_buffers].start = memalign(32, fsize);
		if (!buffers[n_buffers].start) {
			printf("Out of memory\n");
			return ret;
		}
	}

	for (cnt = 0; cnt < n_buffers; cnt++) {
		memset(&buf, 0, sizeof(v4l2_buffer_t));
		buf.type = type;
		buf.memory = V4L2_MEMORY_USERPTR;
		buf.index = cnt;
		buf.m.userptr = (unsigned long)buffers[cnt].start;
		buf.length = buffers[cnt].length;

		ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)&buf);
		if (ret) {
			printf("Fail QBUF %d\n", errno);
			return ret;
			;
		}
	}

	/* VIDIOC_STREAMON start stream */
#if 1
	ret = ioctl(fd, VIDIOC_STREAMON, (unsigned long)&type);
	if (ret < 0) {
		printf("Failed to VIDIOC_STREAMON: errno = %d\n", errno);
		return ret;
	}
#endif
	return OK;
}

static void free_buffer(struct v_buffer *buffers, uint8_t bufnum) {
	uint8_t cnt;
	if (buffers) {
		for (cnt = 0; cnt < bufnum; cnt++) {
			if (buffers[cnt].start) {
				free(buffers[cnt].start);
			}
		}

		free(buffers);
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
struct v4l2_buffer buf;
static int v_fd;
int camera_get_image(void **camera_buf, int *img_w, int *img_h) {
	int ret;
	int exitcode = ERROR;

	struct stat stat_buf;
	uint32_t buf_type;
	uint32_t format;
	static bool first_run = true;

	imageproc_initialize();

	/* In SD card is available, use SD card.
	 * Otherwise, use SPI flash.
	 */
	buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format = V4L2_PIX_FMT_UYVY;
	if (first_run) {
		printf("\n###########################\nInitializing camera\n");

		ret = video_initialize("/dev/video");
		if (ret != 0) {
			printf("ERROR: Failed to initialize video: errno = %d\n", errno);
			goto errout_with_nx;
		}

		v_fd = open("/dev/video", 0);

		if (v_fd < 0) {
			printf("ERROR: Failed to open video: errno = %d\n", errno);
			goto errout_with_video_init;
		}

		/* Prepare VIDEO_CAPTURE */

		ret = camera_prepare(v_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_BUF_MODE_RING, V4L2_PIX_FMT_UYVY, VIDEO_HSIZE_QVGA, VIDEO_VSIZE_QVGA, VIDEO_BUFNUM);
		if (ret < 0) {
			goto errout_with_buffer;
		}
		first_run = false;
	}
	/* Note: VIDIOC_DQBUF acquire capture data. */


	memset(&buf, 0, sizeof(v4l2_buffer_t));
	buf.type = buf_type;
	buf.memory = V4L2_MEMORY_USERPTR;


	ret = ioctl(v_fd, VIDIOC_DQBUF, (unsigned long)&buf);
	if (ret) {
		printf("Fail DQBUF %d\n", errno);
		goto errout_with_buffer;
	}


		imageproc_convert_yuv2rgb((void *)buf.m.userptr, VIDEO_HSIZE_QVGA, VIDEO_VSIZE_QVGA);
		/* Note: VIDIOC_QBUF reset released buffer pointer. */

	*camera_buf = (void *)buf.m.userptr;
	*img_w = VIDEO_VSIZE_QVGA;
	*img_h = VIDEO_HSIZE_QVGA;

#if 0
        /* Note: VIDIOC_QBUF reset released buffer pointer. */
        if (ioctl(v_fd, VIDIOC_QBUF, (unsigned long)&buf)) {
		printf("Fail QBUF %d\n", errno);

	}
#endif
	exitcode = OK;
	return exitcode;

errout_with_buffer:
errout_with_video_init:
errout_with_nx:
	release_camera();
	exitcode = ERROR;
	return exitcode;
}

void close_buf(void)
{

	if (ioctl(v_fd, VIDIOC_QBUF, (unsigned long)&buf)) {
		printf("Fail QBUF %d\n", errno);

	}
	}

void release_camera() {

	close(v_fd);
	free_buffer(buffers_video, VIDEO_BUFNUM);
	video_uninitialize();
	imageproc_finalize();
}
