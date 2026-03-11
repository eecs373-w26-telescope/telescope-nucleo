# STM32 Code for Telescope Processing 

## Dependencies

### Download required software
STM32CubeIDE has a lot of bloat, and has generally bad UX to work with. We instead use specific components of it, specifically STM32CubeMX and STM32CubeCLT, along with any preferred IDE, to allow for a lighter, more flexible embedded development platform. 

You should download the following:
* [STMCubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) (version 6.17.0): This is STM's .ioc configuration tool for generating C initialization code and setting up peripherals 
* [STMCubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) (version ): This is STM's command line toolset, specifically its compiler, debugger, and programmer

### Install any dependencies 
Once you clone the github directory, run the following command to install dependencies.
'''
./scripts/deps.sh
'''
Installs: cmake, build-essential, git

## Build and Flash
Run the following command to build and flash the code. 
* --src: stm32l4 is the path to where the src directory of STM32's main.c file is located
* --preset: directs the compiler to compile the program with specific flags (mainly Debug or Release). Run with Debug for testing/debugging purposes, run with Release for performance and final product purposes
* --flash: this parameter commands the script to flash the code to the connected NUCLEO
'''
./scripts/build_flash.sh --src stm32l4 --preset Debug --flash
'''

## Clean
Run the build flash script with the --clean argument 
'''
./scripts/build_flash.sh --clean
'''