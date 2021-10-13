//                            USER DEFINED SETTINGS
//   Set driver type, fonts to be loaded, pins used and SPI control method etc
//
//   See the User_Setup_Select.h file if you wish to be able to define multiple
//   setups and then easily select which setup file is used by the compiler.
//
//   If this file is edited correctly then all the library example sketches should
//   run without the need to make any more changes for a particular hardware setup!
//   Note that some sketches are designed for a particular TFT pixel width/height


// ##################################################################################
//
// Section 1. Call up the right driver file and any options for it
//
// ##################################################################################

// Tell the library to use 8 bit parallel mode (otherwise SPI is assumed)
//#define TFT_PARALLEL_8_BIT

// Display type -  only define if RPi display
//#define RPI_DISPLAY_TYPE // 20MHz maximum SPI

// Only define one driver, the other ones must be commented out
#define ST7796_DRIVER


// Some displays support SPI reads via the MISO pin, other displays have a single
// bi-directional SDA pin and the library will try to read this via the MOSI line.
// To use the SDA line for reading data from the TFT uncomment the following line:

//#define TFT_SDA_READ      // This option is for ESP32 ONLY, tested with ST7789 and GC9A01 display only


// For ST7735, ST7789 and ILI9341 ONLY, define the colour order IF the blue and red are swapped on your display
// Try ONE option at a time to find the correct colour order for your display

//  #define TFT_RGB_ORDER TFT_RGB  // Colour order Red-Green-Blue
//  #define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red

// #define TFT_INVERSION_ON
// #define TFT_INVERSION_OFF

//##################################################################################
//
// Section 2. Define the pins that are used to interface with the display here
//
// ##################################################################################


// We must use hardware SPI, a minimum of 3 GPIO pins is needed.

// The TFT RESET pin can be connected to the NodeMCU RST pin or 3.3V to free up a control pin
//
// The DC (Data Command) pin may be labelled AO or RS (Register Select)
//
// With some displays such as the ILI9341 the TFT CS pin can be connected to GND if no more
// SPI devices (e.g. an SD Card) are connected, in this case comment out the #define TFT_CS
// line below so it is NOT defined. Other displays such at the ST7735 require the TFT CS pin
// to be toggled during setup, so in these cases the TFT_CS line must be defined and connected.
//
// The NodeMCU D0 pin can be used for RST


// ###### EDIT THE PIN NUMBERS IN THE LINES FOLLOWING TO SUIT YOUR ESP32 SETUP   ######

// For ESP32 Dev board (only tested with ILI9341 display)
// The hardware SPI can be mapped to any pins
// The VSPI is the default SPI port

#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5   // Chip select control pin
#define TFT_DC    4   // Data Command control pin
#define TFT_RST  17   // Reset pin (could connect to RST pin)
//#define TFT_RST  -1 // Set TFT_RST to -1 if display RESET is connected to ESP32 board RST

//#define TOUCH_CS 21     // Chip select pin (T_CS) of touch screen



// ##################################################################################
//
// Section 3. Define the fonts that are to be used here
//
// ##################################################################################

// Comment out the #defines below with // to stop that font being loaded
// The ESP8366 and ESP32 have plenty of memory so commenting out fonts is not
// normally necessary. If all fonts are loaded the extra FLASH space required is
// about 17Kbytes. To save FLASH space only enable the fonts you need!

//#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

// Comment out the #define below to stop the SPIFFS filing system and smooth font code being loaded
// this will save ~20kbytes of FLASH
//#define SMOOTH_FONT


// ##################################################################################
//
// Section 4. Other options
//
// ##################################################################################

// Define the SPI clock frequency, this affects the graphics rendering speed. Too
// fast and the TFT driver will not keep up and display corruption appears.
// With an ILI9341 display 40MHz works OK, 80MHz sometimes fails
// With a ST7735 display more than 27MHz may not work (spurious pixels and lines)
// With an ILI9163 display 27 MHz works OK.

#define SPI_FREQUENCY  79000000

// Optional reduced SPI frequency for reading TFT
#define SPI_READ_FREQUENCY  20000000

// The XPT2046 requires a lower SPI clock rate of 2.5MHz so we define that here:
#define SPI_TOUCH_FREQUENCY  2000000

#define SUPPORT_TRANSACTIONS