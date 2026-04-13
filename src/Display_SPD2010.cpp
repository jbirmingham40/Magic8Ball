#include "Display_SPD2010.h"

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_spd2010.h"
#include "lvgl.h"

#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_ops.h"

uint8_t LCD_Backlight = 60;
static lv_disp_drv_t *s_lvgl_disp_drv = NULL;
static uint16_t *s_flush_dma_buf = NULL;
static portMUX_TYPE s_flush_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_pending_flushes = 0;
static volatile bool s_wait_for_chunk = false;
static volatile bool s_chunk_done = false;

#define LCD_DMA_STAGE_PIXELS (EXAMPLE_LCD_WIDTH * EXAMPLE_LCD_HEIGHT / 10)

static bool LCD_OnColorTransferDone(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx) {
  (void)panel_io;
  (void)edata;
  (void)user_ctx;

  bool chunk_done = false;
  bool flush_done = false;

  portENTER_CRITICAL_ISR(&s_flush_mux);
  if (s_wait_for_chunk) {
    s_chunk_done = true;
    chunk_done = true;
  }
  if (s_pending_flushes > 0) {
    s_pending_flushes = s_pending_flushes - 1;
    flush_done = (s_pending_flushes == 0);
  }
  portEXIT_CRITICAL_ISR(&s_flush_mux);

  if (chunk_done) {
    s_wait_for_chunk = false;
  }
  if (flush_done && s_lvgl_disp_drv) {
    lv_disp_flush_ready(s_lvgl_disp_drv);
  }
  return false;
}

void LCD_RegisterLvglFlushDriver(lv_disp_drv_t *disp_drv) {
  s_lvgl_disp_drv = disp_drv;
}

void SPD2010_Reset(){
  Set_EXIO(EXIO_PIN2,Low);
  delay(50);
  Set_EXIO(EXIO_PIN2,High);
  delay(50);
}
void LCD_Init() {        
  SPD2010_Init();
  Touch_Init();
}

static void test_draw_bitmap(esp_lcd_panel_handle_t panel_handle)
{
  uint16_t row_line = ((EXAMPLE_LCD_WIDTH / EXAMPLE_LCD_COLOR_BITS) << 1) >> 1;
  uint8_t byte_per_pixel = EXAMPLE_LCD_COLOR_BITS / 8;
  uint8_t *color = (uint8_t *)heap_caps_calloc(1, row_line * EXAMPLE_LCD_HEIGHT * byte_per_pixel, MALLOC_CAP_DMA);
  for (int j = 0; j < EXAMPLE_LCD_COLOR_BITS; j++) {
    for (int i = 0; i < row_line * EXAMPLE_LCD_HEIGHT; i++) {
      for (int k = 0; k < byte_per_pixel; k++) {
        color[i * byte_per_pixel + k] = (SPI_SWAP_DATA_TX(BIT(j), EXAMPLE_LCD_COLOR_BITS) >> (k * 8)) & 0xff;
      }
    }
    esp_lcd_panel_draw_bitmap(panel_handle, 0, j * row_line, EXAMPLE_LCD_HEIGHT, (j + 1) * row_line, color);
  }
  free(color);
}

bool QSPI_Init(void){
  static const spi_bus_config_t host_config = {            
    .data0_io_num = ESP_PANEL_LCD_SPI_IO_DATA0,                    
    .data1_io_num = ESP_PANEL_LCD_SPI_IO_DATA1,                   
    .sclk_io_num = ESP_PANEL_LCD_SPI_IO_SCK,                   
    .data2_io_num = ESP_PANEL_LCD_SPI_IO_DATA2,                    
    .data3_io_num = ESP_PANEL_LCD_SPI_IO_DATA3,                    
    .data4_io_num = -1,                       
    .data5_io_num = -1,                      
    .data6_io_num = -1,                       
    .data7_io_num = -1,                      
    .max_transfer_sz = ESP_PANEL_HOST_SPI_MAX_TRANSFER_SIZE, 
    .flags = SPICOMMON_BUSFLAG_MASTER,       
    .intr_flags = 0,                            
  };
  if(spi_bus_initialize(SPI2_HOST, &host_config, SPI_DMA_CH_AUTO) != ESP_OK){
    printf("The SPI initialization failed.\r\n");
    return false;
  }
  printf("The SPI initialization succeeded.\r\n");
  return true;
}

esp_lcd_panel_handle_t panel_handle = NULL;
bool SPD2010_Init() {
  SPD2010_Reset();
  pinMode(ESP_PANEL_LCD_SPI_IO_TE, INPUT_PULLDOWN);
  if(!QSPI_Init()){
    printf("SPD2010 Failed to be initialized\r\n");
  }
    
  const esp_lcd_panel_io_spi_config_t io_config ={
    .cs_gpio_num = ESP_PANEL_LCD_SPI_IO_CS,               
    .dc_gpio_num = -1,                  
    .spi_mode = ESP_PANEL_LCD_SPI_MODE,                      
    .pclk_hz = ESP_PANEL_LCD_SPI_CLK_HZ,     
    .trans_queue_depth = ESP_PANEL_LCD_SPI_TRANS_QUEUE_SZ,            
    .on_color_trans_done = LCD_OnColorTransferDone,
    .user_ctx = NULL,
    .lcd_cmd_bits = ESP_PANEL_LCD_SPI_CMD_BITS,                 
    .lcd_param_bits = ESP_PANEL_LCD_SPI_PARAM_BITS,                
    .flags = {                          
      .dc_low_on_data = 0,            
      .octal_mode = 0,                
      .quad_mode = 1,                 
      .sio_mode = 0,                  
      .lsb_first = 0,                 
      .cs_high_active = 0,            
    },                                  
  };
  esp_lcd_panel_io_handle_t io_handle = NULL;
  if(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ESP_PANEL_HOST_SPI_ID_DEFAULT, &io_config, &io_handle) != ESP_OK){
    printf("Failed to set LCD communication parameters -- SPI\r\n");
    return false;
  }else{
    printf("LCD communication parameters are set successfully -- SPI\r\n");
  }
  printf("Install LCD driver of spd2010\r\n");
  spd2010_vendor_config_t vendor_config={  
    .flags = {
      .use_qspi_interface = 1,
    },
  };
  esp_lcd_panel_dev_config_t panel_config={
    .reset_gpio_num = EXAMPLE_LCD_PIN_NUM_RST,                                     
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,                   
    .data_endian = LCD_RGB_DATA_ENDIAN_BIG,                       
    .bits_per_pixel = EXAMPLE_LCD_COLOR_BITS,                                 
    // .flags = {                                                    
    //   .reset_active_high = 0,                                   
    // },                                                            
    .vendor_config = (void *) &vendor_config,                                  
  };
  esp_lcd_new_panel_spd2010(io_handle, &panel_config, &panel_handle);

  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
  // esp_lcd_panel_invert_color(panel_handle,false);

  esp_lcd_panel_disp_on_off(panel_handle, true);
  //test_draw_bitmap(panel_handle);
  printf("spd2010 LCD OK\r\n");

  if (s_flush_dma_buf == NULL) {
    s_flush_dma_buf = (uint16_t *)heap_caps_malloc(
        LCD_DMA_STAGE_PIXELS * sizeof(uint16_t),
        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (s_flush_dma_buf == NULL) {
      printf("Failed to allocate LCD DMA staging buffer\r\n");
      return false;
    }
  }
  return true;
}

void LCD_addWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,uint16_t* color)
{ 
  const uint16_t width = Xend - Xstart + 1;
  const uint16_t height = Yend - Ystart + 1;
  uint16_t max_chunk_rows = height;
  if (s_flush_dma_buf != NULL && width > 0) {
    max_chunk_rows = LCD_DMA_STAGE_PIXELS / width;
    if (max_chunk_rows == 0) {
      max_chunk_rows = 1;
    }
  }
  const uint16_t chunk_count = (height + max_chunk_rows - 1) / max_chunk_rows;

  portENTER_CRITICAL(&s_flush_mux);
  s_pending_flushes = chunk_count;
  s_wait_for_chunk = false;
  s_chunk_done = false;
  portEXIT_CRITICAL(&s_flush_mux);

  uint16_t rows_sent = 0;
  for (uint16_t chunk_index = 0; chunk_index < chunk_count; chunk_index++) {
    uint16_t chunk_rows = height - rows_sent;
    if (chunk_rows > max_chunk_rows) {
      chunk_rows = max_chunk_rows;
    }
    const uint32_t chunk_pixels = (uint32_t)width * chunk_rows;
    uint16_t *tx_buf = color + ((uint32_t)rows_sent * width);

    if (s_flush_dma_buf != NULL && chunk_pixels <= LCD_DMA_STAGE_PIXELS) {
      tx_buf = s_flush_dma_buf;
      const uint16_t *src = color + ((uint32_t)rows_sent * width);
      for (uint32_t i = 0; i < chunk_pixels; i++) {
        uint16_t pixel = src[i];
        tx_buf[i] = (uint16_t)(((pixel >> 8) & 0xFF) | ((pixel << 8) & 0xFF00));
      }
    } else {
      printf("LCD flush chunk too large for DMA staging buffer: %lu pixels\r\n",
             (unsigned long)chunk_pixels);
      for (uint32_t i = 0; i < chunk_pixels; i++) {
        uint16_t pixel = tx_buf[i];
        tx_buf[i] = (uint16_t)(((pixel >> 8) & 0xFF) | ((pixel << 8) & 0xFF00));
      }
    }

    const uint16_t chunk_y_start = Ystart + rows_sent;
    uint16_t chunk_x_end = Xstart + width;
    uint16_t chunk_y_end = chunk_y_start + chunk_rows;
    if (chunk_x_end > EXAMPLE_LCD_WIDTH)
      chunk_x_end = EXAMPLE_LCD_WIDTH;
    if (chunk_y_end > EXAMPLE_LCD_HEIGHT)
      chunk_y_end = EXAMPLE_LCD_HEIGHT;

    if (chunk_index + 1 < chunk_count) {
      portENTER_CRITICAL(&s_flush_mux);
      s_wait_for_chunk = true;
      s_chunk_done = false;
      portEXIT_CRITICAL(&s_flush_mux);
    }

    esp_lcd_panel_draw_bitmap(panel_handle, Xstart, chunk_y_start, chunk_x_end, chunk_y_end, tx_buf);

    if (chunk_index + 1 < chunk_count) {
      while (!s_chunk_done) {
        vTaskDelay(1);
      }
    }

    rows_sent += chunk_rows;
  }
}



// backlight
void Backlight_Init()
{
  ledcAttach(LCD_Backlight_PIN, Frequency, Resolution);    // 设置通道
  ledcWrite(LCD_Backlight_PIN, Dutyfactor);               // 设置亮度    
  Set_Backlight(LCD_Backlight);      //0~100  
}

void Set_Backlight(uint8_t Light)                        //
{
  if(Light > Backlight_MAX || Light < 0)
    printf("Set Backlight parameters in the range of 0 to 100 \r\n");
  else{
    // Persist the configured brightness level so wake-from-sleep restores
    // the user's selected setting instead of a stale/default value.
    LCD_Backlight = Light;
    uint32_t Backlight = Light*10;
    if(Backlight == 1000)
      Backlight = 1024;
    ledcWrite(LCD_Backlight_PIN, Backlight);
  }
}

void LCD_Sleep(bool sleep) {
  if (panel_handle) {
    esp_lcd_panel_disp_on_off(panel_handle, !sleep);
  }
}
