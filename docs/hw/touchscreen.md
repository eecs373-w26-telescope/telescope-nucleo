# Touchscreen over SPI implementation description
The touchscreen module enables user interaction through a LCD display and a touch input. It supports numeric input, button control, and real-time display updates. The display is driven by ST7796S controller, while touch input is read through XPT2046 touch controller over SPI. 

## Hardware Interface
The LCD display and touch controller share a single SPI bus but use different CHIP SELECT lines. Apart from the SPI’s MOSI, MISO, SCK and the chipselects, the LCD has an RESET line and a backlight control (LED) line while the touch controller has an extra IRQ line to indicate when a touch is detected. 

## Software Architecture
1. Touch Driver (touch.cpp)
    * Reads raw 12 bit ADC values indicating touch position. Calibration. Gain a robust position through averaging. Map the raw value to pixels and then to keypad and buttons.
    * Handles SPI communication. Drive IRQ low when a touch is detected
2. Touchscreen UI (touchscreen.cpp)
    * Basic display driver. Initialize the display. Handles display rendering.
    * Process mapped touch positions. 
    * Implements UI logic (keypad, input box, buttons). Small searching functions for DSO objects

These two components are called upon in the **controller layer**

* Integrates the two modules
* Manage system states based on inputs 

## Touch Input Processing Pipeline
* Detect touch events via IRQ signal
* Read raw X/Y values via SPI
* Mapped raw X/Y values to screen coordinates
* Determine the corresponding button and return it to the controller for display and other uses

## Display 
* Initialized LCD display
* Implemented basic designs. Drawing characters, strings, input box, and buttons.
* Updates the input box according to the touch input
* States and page changes according to the touch input

## Features Implemented
* Keypad inputs and real-time  input boxes updates.
* Backspace, clearing, and enter functionality.
* Taring button to notify the controller.
* N/M button for different catalogue selection
* Searching state when the input DSO exists in the catalogue
* Error state when the input DSO does not exist in the catalogue
* DSO view switch. 
