#pragma once

int camera_get_image(void **camera_buf, int *x, int *y);
void release_camera(void);
void close_buf(void);
