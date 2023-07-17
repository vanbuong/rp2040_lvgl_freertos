#include "pico/types.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "st7789.h"
#include "lvgl.h"

#define CS_HIGH()   gpio_put(TFT_CS, 1)
#define CS_LOW()    gpio_put(TFT_CS, 0)
#define DC_HIGH()   gpio_put(TFT_DC, 1)
#define DC_LOW()    gpio_put(TFT_DC, 0)

int16_t _height, _width;
uint8_t  _colstart, _rowstart, _xstart, _ystart; // some displays need this changed

static const uint8_t
  cmd_240x240[] = {                 		// Initialization commands for 7789 screens
    10,                       				// 9 commands in list:
    ST7789_SWRESET,   ST_CMD_DELAY,  		// 1: Software reset, no args, w/delay
      150,                     				// 150 ms delay
    ST7789_SLPOUT ,   ST_CMD_DELAY,  		// 2: Out of sleep mode, no args, w/delay
      255,                    				// 255 = 500 ms delay
    ST7789_COLMOD , 1+ST_CMD_DELAY,  		// 3: Set color mode, 1 arg + delay:
      0x55,                   				// 16-bit color
      10,                     				// 10 ms delay
    ST7789_MADCTL , 1,  					// 4: Memory access ctrl (directions), 1 arg:
      0x00,                   				// Row addr/col addr, bottom to top refresh
    ST7789_CASET  , 4,  					// 5: Column addr set, 4 args, no delay:
      0x00, ST7789_240x240_XSTART,          // XSTART = 0
	  (240+ST7789_240x240_XSTART) >> 8,
	  (240+ST7789_240x240_XSTART) & 0xFF,   // XEND = 240
    ST7789_RASET  , 4,  					// 6: Row addr set, 4 args, no delay:
      0x00, ST7789_240x240_YSTART,          // YSTART = 0
      (240+ST7789_240x240_YSTART) >> 8,
	  (240+ST7789_240x240_YSTART) & 0xFF,	// YEND = 240
    ST7789_INVON ,   ST_CMD_DELAY,  		// 7: Inversion ON
      10,
    ST7789_NORON  ,   ST_CMD_DELAY,  		// 8: Normal display on, no args, w/delay
      10,                     				// 10 ms delay
    ST7789_DISPON ,   ST_CMD_DELAY,  		// 9: Main screen turn on, no args, w/delay
    255 };                  				// 255 = 500 ms delay

void st7789_write_data(uint8_t c)
{
    DC_HIGH();
    CS_LOW();

    spi_write_blocking(SPI_CHAN, &c, 1);

    CS_HIGH();
}

void st7789_write_command(uint8_t c)
{
    DC_LOW();
    CS_LOW();

    spi_write_blocking(SPI_CHAN, &c, 1);

    CS_HIGH();
}

void st7789_set_rotation(uint8_t m) {

  st7789_write_command(ST7789_MADCTL);
  uint8_t rotation = m % 4; // can't be higher than 3
  switch (rotation) {
  case 0:
     st7789_write_data(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);

     _xstart = _colstart+52;
     _ystart = _rowstart+40;
     break;
  case 1:
     st7789_write_data(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);

     _ystart = _colstart+52;
     _xstart = _rowstart+40;
     break;
  case 2:
     st7789_write_data(ST7789_MADCTL_RGB);
 
     _xstart = _colstart+53;
     _ystart = _rowstart+40;
     break;

   case 3:
     st7789_write_data(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);

     _ystart = _colstart+52;
     _xstart = _rowstart+40;
     break;
  }
}

void st7789_set_addr_window(uint8_t x0, uint8_t y0, uint8_t x1,
 uint8_t y1) {

  uint16_t x_start = x0 + _xstart, x_end = x1 + _xstart;
  uint16_t y_start = y0 + _ystart, y_end = y1 + _ystart;
  

  st7789_write_command(ST7789_CASET); // Column addr set
  st7789_write_data(x_start >> 8);
  st7789_write_data(x_start & 0xFF);     // XSTART 
  st7789_write_data(x_end >> 8);
  st7789_write_data(x_end & 0xFF);     // XEND

  st7789_write_command(ST7789_RASET); // Row addr set
  st7789_write_data(y_start >> 8);
  st7789_write_data(y_start & 0xFF);     // YSTART
  st7789_write_data(y_end >> 8);
  st7789_write_data(y_end & 0xFF);     // YEND

  st7789_write_command(ST7789_RAMWR); // write to RAM
}

// fill a rectangle
void st7789_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  st7789_set_addr_window(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
  DC_HIGH();
  CS_LOW();
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spi_write_blocking(SPI_CHAN, &hi, 1);
      spi_write_blocking(SPI_CHAN, &lo, 1);
    }
  }
  CS_HIGH();
}

void st7789_fill_screen(uint16_t color) {
  st7789_fill_rect(0, 0,  _width, _height, color);
}

void st7789_draw_char(const char c, int16_t x, int16_t y)
{
  
}


void st7789_init(void)
{
    uint8_t *addr = cmd_240x240;
    uint8_t numCommands;
    uint8_t numArgs;
    uint16_t ms;

    numCommands = *addr++;   // Number of commands to follow
    while(numCommands--) {                 // For each command...
        st7789_write_command(*addr++); //   Read, issue command
        numArgs  = *addr++;    //   Number of args to follow
        ms       = numArgs & ST_CMD_DELAY;   //   If hibit set, delay follows args
        numArgs &= ~ST_CMD_DELAY;            //   Mask out delay bit
        while(numArgs--) {                   //   For each argument...
            st7789_write_data(*addr++);  //     Read, issue argument
        }

        if(ms) {
            ms = *addr++; // Read post-command delay time (ms)
            if(ms == 255) ms = 500;     // If 255, delay for 500 ms
            sleep_ms(ms);
        }
    }

    _colstart = ST7789_240x240_XSTART;
    _rowstart = ST7789_240x240_YSTART;
    _height = 240;
    _width = 240;


    st7789_set_rotation(1);
}

/* The ST7789 display controller can drive 320*240 displays, when using a 240*240
 * display there's a gap of 80px, we need to edit the coordinates to take into
 * account that gap, this is not necessary in all orientations. */
void st7789_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{

  st7789_set_addr_window(area->x1, area->y1, area->x2, area->y2);

  uint8_t hi, lo;
  uint16_t color;

  DC_HIGH();
  CS_LOW();
  for(int y=area->y1; y<=area->y2; y++) {
    for(int x=area->x1; x<=area->x2; x++) {
            color = color_map->full;
            color_map++;
            hi = color >> 8;
            lo = color;
            spi_write_blocking(SPI_CHAN, &hi, 1);
            spi_write_blocking(SPI_CHAN, &lo, 1);
    }
  }
  CS_HIGH(); 

  lv_disp_flush_ready(drv);

}