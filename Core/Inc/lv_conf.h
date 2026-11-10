#ifndef LV_CONF_H
#define LV_CONF_H

/* This project runs LVGL from the bare-metal main loop. */
#define LV_USE_OS LV_OS_NONE

/* The ST7735S module and LVGL draw buffers use RGB565. */
#define LV_COLOR_FORMAT_DEFAULT LV_COLOR_FORMAT_RGB565
#define LV_MEM_SIZE (32U * 1024U)

/* Widgets planned for the first UI screens. */
#define LV_USE_LABEL 1
#define LV_USE_CHART 1

/* LVGL v9.6 ST7735 driver is built on the generic MIPI driver. */
#define LV_USE_GENERIC_MIPI 1
#define LV_USE_ST7735 1

#endif /* LV_CONF_H */
