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

#include "Adafruit_SharpMem.h"


#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif
#ifndef _swap_uint16_t
#define _swap_uint16_t(a, b)                                                   \
  {                                                                            \
    uint16_t t = a;                                                            \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif
/**************************************************************************
    Sharp Memory Display Connector as detailed in Datashet
    https://cdn.sparkfun.com/assets/d/e/8/9/7/LS013B7DH03_datasheet.pdf
    -----------------------------------------------------------------------
    Pin   Function        Notes
    ===   ==============  ===============================
      1   SCLK            Serial Clock
      2   MOSI            Serial Data Input
      3   CS              Serial Chip Select
      4   EXTCOMIN        External COM Inversion Signal
      5   DISP            Display On(High)/Off(Low)
      6   3V3             3.3V out
      7   VIN             3.3-5.0V (into LDO supply)
      8   EXTMODE         COM Inversion Select (Low = SW clock/serial)
      9   GND
      10  VSSA  -> GND
 **************************************************************************/
#define TOGGLE_VCOM                                                            \
  do {                                                                         \
    _sharpmem_vcom = _sharpmem_vcom ? 0x00 : SHARPMEM_BIT_VCOM;                \
  } while (0);


/**
 * @brief Construct a new Adafruit_SharpMem object with software SPI
 *
 * @param clk The clock pin
 * @param mosi The MOSI pin
 * @param cs The display chip select pin - **NOTE** this is ACTIVE HIGH!
 * @param width The display width
 * @param height The display height
 * @param freq The SPI clock frequency desired (unlikely to be that fast in soft
 * spi mode!)
 */
Adafruit_SharpMem::Adafruit_SharpMem(uint8_t clk, uint8_t mosi, uint8_t cs,
                                     uint16_t width, uint16_t height)
    : Adafruit_GFX(width, height) {
  _clk = clk;
  _mosi = mosi; 
  _cs = cs;
  _width = width;
  _height = height;

  gpio_set_direction((gpio_num_t)_mosi, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)_clk, GPIO_MODE_OUTPUT);
  // With some GPIOs CS needs a Reset (Discovering this turned some new hairs completely white)
  gpio_reset_pin((gpio_num_t)_cs);
  gpio_set_direction((gpio_num_t)_cs, GPIO_MODE_OUTPUT);
}

/**
 * @brief Start the driver object, setting up pins and configuring a buffer for
 * the screen contents
 *
 * @return boolean true: success false: failure
 */
boolean Adafruit_SharpMem::begin(void) {
  
  // Set the vcom bit to a defined state
  _sharpmem_vcom = SHARPMEM_BIT_VCOM;
 
  sharpmem_buffer = (uint8_t *)malloc((_width * _height) / 8);

  if (!sharpmem_buffer) {
    printf("Error: sharpmem_buffer was NOT allocated\n\n");
    return false;
  }

  // Initialize SPI
    esp_err_t ret;
    // MISO not used, only Master to Slave
    spi_bus_config_t buscfg = {
        .mosi_io_num= _mosi,
        .miso_io_num= -1,
        .sclk_io_num= _clk,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=4094
    };
    // max_transfer_sz   4Kb is the defaut SPI transfer size if 0
    // debug: 50000  0.5 Mhz so we can sniff the SPI commands with a Slave
    uint16_t multiplier = 1000;

    //Config Frequency and SS GPIO

    //SPI_DEVICE_TXBIT_LSBFIRST
    spi_device_interface_config_t devcfg = {
        .mode = 0,  //SPI mode 0
        .clock_speed_hz = 4 * multiplier * 1000, // Can be up to 4 Mhz
        .spics_io_num = -1,                      // -1 == Do not control CS automatically!
        .flags = (SPI_DEVICE_TXBIT_LSBFIRST | SPI_DEVICE_3WIRE),
        .queue_size= 5
    };
    // DISABLED Callbacks pre_cb/post_cb. SPI does not seem to behave the same
    // CS / DC GPIO states the usual way

    //Initialize the SPI bus
    ret = spi_bus_initialize(EPD_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);

    //Attach the EPD to the SPI bus
    ret = spi_bus_add_device(EPD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    printf("SPI initialized. MOSI:%d CLK:%d CS:%d\n", _mosi, _clk, _cs);

  // This display is weird in that _cs is active HIGH not LOW like every other SPI device
  gpio_set_level((gpio_num_t)_cs, 0);
  return true;
}

// 1<<n is a costly operation on AVR -- table usu. smaller & faster
static const uint8_t  set[] = {1, 2, 4, 8, 16, 32, 64, 128},
                      clr[] = {(uint8_t)~1,  (uint8_t)~2,  (uint8_t)~4,
                              (uint8_t)~8,  (uint8_t)~16, (uint8_t)~32,
                              (uint8_t)~64, (uint8_t)~128};

/**************************************************************************/
/*!
    @brief Draws a single pixel in image buffer

    @param[in]  x
                The x position (0 based)
    @param[in]  y
                The y position (0 based)
    @param color The color to set:
    * **0**: Black
    * **1**: White
*/
/**************************************************************************/
void Adafruit_SharpMem::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
    return;

  switch (rotation) {
  case 1:
    _swap_int16_t(x, y);
    x = _width - 1 - x;
    break;
  case 2:
    x = _width - 1 - x;
    y = _height - 1 - y;
    break;
  case 3:
    _swap_int16_t(x, y);
    y = _height - 1 - y;
    break;
  }

  if (color) {
    sharpmem_buffer[(y * _width + x) / 8] |= set[x & 7]; // set[x & 7]
  } else {
    sharpmem_buffer[(y * _width + x) / 8] &= clr[x & 7]; // clr[x & 7]
  }
}

/**************************************************************************/
/*!
    @brief Gets the value (1 or 0) of the specified pixel from the buffer

    @param[in]  x
                The x position (0 based)
    @param[in]  y
                The y position (0 based)

    @return     1 if the pixel is enabled, 0 if disabled
*/
/**************************************************************************/
uint8_t Adafruit_SharpMem::getPixel(uint16_t x, uint16_t y) {
  if ((x >= _width) || (y >= _height))
    return 0; // <0 test not needed, unsigned

  switch (rotation) {
  case 1:
    _swap_uint16_t(x, y);
    x = _width - 1 - x;
    break;
  case 2:
    x = _width - 1 - x;
    y = _height - 1 - y;
    break;
  case 3:
    _swap_uint16_t(x, y);
    y = _height - 1 - y;
    break;
  }

  return sharpmem_buffer[(y * _width + x) / 8] & set[x & 7] ? 1 : 0;
}

/**************************************************************************/
/*!
    @brief Clears the screen
*/
/**************************************************************************/
void Adafruit_SharpMem::clearDisplay() {
  memset(sharpmem_buffer, 0xff, (_width * _height) / 8);

  gpio_set_level((gpio_num_t)_cs, 1);
  uint8_t clear_data[2] = {(uint8_t)(_sharpmem_vcom | SHARPMEM_BIT_CLEAR), 0x00};
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));        //Zero out the transaction
  t.length = sizeof(clear_data)*8; //Each data byte is 8 bits Einstein
  t.tx_buffer = clear_data;
  ret = spi_device_polling_transmit(spi, &t); // spi_device_polling_transmit

  TOGGLE_VCOM;
  gpio_set_level((gpio_num_t)_cs, 0);
  
  //printf("clearDisplay b1:%02x 2:%02x lenght:%d\n\n", clear_data[0], clear_data[1], t.length);
  assert(ret==ESP_OK);
}

/**************************************************************************/
/*!
    @brief Renders the contents of the pixel buffer on the LCD
*/
/**************************************************************************/
void Adafruit_SharpMem::refresh(void) {
  uint16_t i, currentline;

  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));       //Zero out the transaction

  gpio_set_level((gpio_num_t)_cs, 1);
  int vcom[1] = {_sharpmem_vcom | SHARPMEM_BIT_WRITECMD};
  t.length = 8;                  //Each data byte is 8 bits
  t.tx_buffer = vcom;
  ret = spi_device_transmit(spi, &t);

  TOGGLE_VCOM;

  uint8_t bytes_per_line = WIDTH / 8;
  uint16_t totalbytes = (WIDTH * HEIGHT) / 8;

  for (i = 0; i < totalbytes; i += bytes_per_line) {
    uint8_t line[bytes_per_line + 2];

    // Send address byte
    currentline = ((i + 1) / (WIDTH / 8)) + 1;
    line[0] = currentline;
    // copy over this line
    memcpy(line + 1, sharpmem_buffer + i, bytes_per_line);
    // Send end of line
    line[bytes_per_line + 1] = 0x00;

    t.length = (bytes_per_line+2) *8; // bytes_per_line+2
    t.tx_buffer = line;
    ret = spi_device_transmit(spi, &t);
    assert(ret==ESP_OK);
  }
  // Send another trailing 8 bits for the last line
  int last_line[1] = {0x00};
  t.length = 8;

  t.tx_buffer = last_line;
  ret = spi_device_transmit(spi, &t); // spi_device_polling_transmit
  gpio_set_level((gpio_num_t)_cs, 0);

  assert(ret==ESP_OK);
  }

/**************************************************************************/
/*!
    @brief Clears the display buffer without outputting to the display
*/
/**************************************************************************/
void Adafruit_SharpMem::clearDisplayBuffer() {
  memset(sharpmem_buffer, 0xFF, (_width * _height) / 8);
}

uint8_t Adafruit_SharpMem::_unicodeEasy(uint8_t c) {
  if (c<191 && c>131 && c!=176) { // 176 is °W 
    c+=64;
  }
  return c;
}

void Adafruit_SharpMem::print(const std::string& text){
   for(auto c : text) {
     if (c==195 || c==194) continue; // Skip to next letter
     c = _unicodeEasy(c);
     write(uint8_t(c));
   }
}

void Adafruit_SharpMem::println(const std::string& text){
   for(auto c : text) {
     if (c==195 || c==194) continue; // Skip to next letter

     // _unicodeEasy will just sum 64 and get the right character, should be faster and cover more chars
     c = _unicodeEasy(c);
     //c = _unicodePerChar(c); // _unicodePerChar has more control since they are only hand-picked chars
     write(uint8_t(c));
   }
   write(10); // newline
}

/**
 * @brief Similar to printf
 * Note that buffer needs to end with null character
 * @param format 
 * @param ... va_list
 */
void Adafruit_SharpMem::printerf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    char max_buffer[1024];
    int size = vsnprintf(max_buffer, sizeof max_buffer, format, args);
    va_end(args);

    if (size < sizeof(max_buffer)) {
      print(std::string(max_buffer));
    } else {
      ESP_LOGE("Epd::printerf", "max_buffer out of range. Increase max_buffer!");
    }
}
