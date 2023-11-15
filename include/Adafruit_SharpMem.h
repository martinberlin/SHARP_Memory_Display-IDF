/*********************************************************************
This is an Arduino library for our Monochrome SHARP Memory Displays

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1393

These displays use SPI to communicate, 3 pins are required to
interface

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/
#ifndef LIB_ADAFRUIT_SHARPMEM
#define LIB_ADAFRUIT_SHARPMEM

#include <Adafruit_GFX.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#include <string.h>
#include "esp_log.h"

#ifdef CONFIG_IDF_TARGET_ESP32
    #define EPD_HOST    HSPI_HOST
    #define DMA_CHAN    2
#elif defined CONFIG_IDF_TARGET_ESP32S2
    #define EPD_HOST    SPI2_HOST
    #define DMA_CHAN    EPD_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S3
    #define EPD_HOST    SPI2_HOST
    #define DMA_CHAN    SPI_DMA_CH_AUTO
#elif defined CONFIG_IDF_TARGET_ESP32C3
    // chip only support spi dma channel auto-alloc
    #define EPD_HOST    SPI2_HOST
    #define DMA_CHAN    SPI_DMA_CH_AUTO
#endif

#define SHARPMEM_BIT_WRITECMD (0x01) // 0x80 in LSB format otherwise 0x01
#define SHARPMEM_BIT_VCOM (0x02)     // Sent now using SPI_DEVICE_TXBIT_LSBFIRST
#define SHARPMEM_BIT_CLEAR (0x04)

/**
 * @brief Class to control a Sharp memory display
 *
 */
class Adafruit_SharpMem : public Adafruit_GFX {
public:
  Adafruit_SharpMem(uint8_t clk, uint8_t mosi, uint8_t cs, uint16_t w = 128,
                    uint16_t h = 128);

  boolean begin();
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  uint8_t getPixel(uint16_t x, uint16_t y);
  void clearDisplay();
  void refresh(void);
  void refreshLines(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);
  
  void clearDisplayBuffer();
  void print(const std::string& text);
  void println(const std::string& text);
  void printerf(const char *format, ...);

private:
  spi_device_handle_t spi;

  uint8_t *sharpmem_buffer = NULL;
  // IOs
  uint8_t _clk;
  uint8_t _mosi;
  uint8_t _cs;

  uint16_t _width = 128;
  uint16_t _height = 128;
  
  uint8_t _sharpmem_vcom;

  uint8_t _unicodeEasy(uint8_t c);
};

#endif
