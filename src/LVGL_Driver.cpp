#include "esp_sleep.h"
/*****************************************************************************
  | File        :   LVGL_Driver.c
  
  | help        : 
    The provided LVGL library file must be installed first
******************************************************************************/
#include "LVGL_Driver.h"
#include "PWR_Key.h"

static esp_timer_handle_t s_lvgl_tick_timer = NULL;
static lv_disp_draw_buf_t draw_buf;
// Allocated in Lvgl_Init() — NOT as static initialisers.
// Static initialisers run before setup(), potentially before PSRAM is ready,
// causing heap_caps_malloc(MALLOC_CAP_SPIRAM) to silently return NULL.
// A NULL rot_buf makes the flush callback call lv_disp_flush_ready() immediately
// without sending any pixels, so the display never updates.
static lv_color_t *buf1    = nullptr;
static lv_color_t *buf2    = nullptr;
static lv_color_t *rot_buf = nullptr;

// Rotate a rectangle 90° clockwise.  For a square display the rotated area
// happens to be identical, but the helper is kept for correctness.
static inline void rotate_area_clockwise_90(const lv_area_t *src, lv_area_t *dst) {
  dst->x1 = LCD_WIDTH - 1 - src->y2;
  dst->y1 = src->x1;
  dst->x2 = LCD_WIDTH - 1 - src->y1;
  dst->y2 = src->x2;
}

static void rotate_pixels_clockwise_90(const lv_area_t *area,
                                       const lv_color_t *src,
                                       lv_color_t *dst) {
  const uint16_t src_w = lv_area_get_width(area);
  const uint16_t src_h = lv_area_get_height(area);
  const uint16_t dst_w = src_h;

  for (uint16_t y = 0; y < src_h; y++) {
    for (uint16_t x = 0; x < src_w; x++) {
      const uint16_t dst_x = src_h - 1 - y;
      const uint16_t dst_y = x;
      dst[dst_y * dst_w + dst_x] = src[y * src_w + x];
    }
  }
}

/*  Display flushing
    Rotate the LVGL frame 90° clockwise then DMA it to the SPD2010 panel.
    lv_disp_flush_ready() is called from the DMA-done ISR (LCD_OnColorTransferDone)
    once the last chunk finishes — NOT from here.
*/
void Lvgl_Display_LCD(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  // Log the very first flush so we can confirm pixels are being sent.
  static bool flushed_once = false;
  if (!flushed_once) {
    flushed_once = true;
    Serial.printf("[LVGL] First flush: area=(%d,%d)-(%d,%d) rot_buf=%p\n",
                  area->x1, area->y1, area->x2, area->y2, rot_buf);
  }

  if (PWR_IsDisplayAwake()) {
    const uint32_t pixel_count = lv_area_get_size(area);
    if (rot_buf != NULL && pixel_count <= LVGL_ROT_BUF_LEN) {
      lv_area_t rotated_area;
      rotate_area_clockwise_90(area, &rotated_area);
      rotate_pixels_clockwise_90(area, color_p, rot_buf);
      LCD_addWindow(rotated_area.x1, rotated_area.y1, rotated_area.x2, rotated_area.y2,
                    (uint16_t *)&rot_buf->full);
      // lv_disp_flush_ready() is signalled by the DMA callback.
    } else {
      // rot_buf is NULL or area is larger than rotation buffer — skip transfer.
      Serial.printf("[LVGL] Flush skipped: rot_buf=%p pixel_count=%lu\n",
                    rot_buf, (unsigned long)pixel_count);
      lv_disp_flush_ready(disp_drv);
    }
  } else {
    lv_disp_flush_ready(disp_drv);
  }
}

/*Read the touchpad*/
void Lvgl_Touchpad_Read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  bool tp_pressed = false;
  uint16_t tp_x = 0;
  uint16_t tp_y = 0;
  uint8_t tp_cnt = 0;
  tp_pressed = Touch_Get_xy(&tp_x, &tp_y, NULL, &tp_cnt, CONFIG_ESP_LCD_TOUCH_MAX_POINTS);
  if (tp_pressed && (tp_cnt > 0)) {
    data->point.x = tp_y;
    data->point.y = LCD_HEIGHT - 1 - tp_x;
    data->state = LV_INDEV_STATE_PR;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void example_increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void Lvgl_Init(void) {
  // Allocate frame buffers here — PSRAM is guaranteed to be initialised by now.
  buf1    = (lv_color_t *)heap_caps_malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  buf2    = (lv_color_t *)heap_caps_malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  rot_buf = (lv_color_t *)heap_caps_malloc(LVGL_ROT_BUF_LEN      * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);

  if (!buf1 || !rot_buf) {
    Serial.printf("[LVGL] FATAL: frame buffer alloc failed — buf1=%p rot_buf=%p\n", buf1, rot_buf);
    return;
  }
  Serial.printf("[LVGL] Buffers OK — buf1=%p buf2=%p rot_buf=%p\n", buf1, buf2, rot_buf);

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LVGL_DRAW_BUF_LEN);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;
  disp_drv.flush_cb = Lvgl_Display_LCD;
  // Power optimization: avoid full-frame redraw on every LVGL flush.
  // Let LVGL redraw only invalidated regions.
  disp_drv.full_refresh = 1;
  disp_drv.draw_buf = &draw_buf;

  disp_drv.sw_rotate = 0;
  disp_drv.rotated = LV_DISP_ROT_NONE;
  LCD_RegisterLvglFlushDriver(&disp_drv);

  lv_disp_drv_register(&disp_drv);


  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = Lvgl_Touchpad_Read;
  lv_indev_drv_register(&indev_drv);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };
  esp_timer_create(&lvgl_tick_timer_args, &s_lvgl_tick_timer);
  esp_timer_start_periodic(s_lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  // Get the input device
  lv_indev_t *indev = lv_indev_get_next(NULL);

  // Loop through devices (in case you have more than one, like touch + buttons)
  while (indev) {
    if (indev->driver) {
      indev->driver->long_press_time = 1000;  // Set to 1000ms
    }
    indev = lv_indev_get_next(indev);
  }
}

void Lvgl_Loop(void) {
  long time_in_us = lv_timer_handler(); /* let the GUI do its work */
}

// Stop the LVGL tick timer when the display is off.  This eliminates a
// 20 ms periodic wakeup that prevents the CPU from staying in light sleep.
void Lvgl_PauseTick(void) {
  if (s_lvgl_tick_timer) esp_timer_stop(s_lvgl_tick_timer);
}

// Restart the LVGL tick timer when the display wakes.
void Lvgl_ResumeTick(void) {
  if (s_lvgl_tick_timer) esp_timer_start_periodic(s_lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);
}
