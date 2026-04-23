# SD Card over Parallel 4-Bit SDIO Implementation Description
The SD card module provides persistent storage and efficient querying of astronomical object data using a custom binary format.

## Hardware Interface
The SD card interfaces with the STM32 within an integrated SD card slot on the Feather board, via 4-bit SDIO and uses the FATFS filesystem for file access.

## Software Architecture
The SD card system (sd_card.cpp) is built as a hardware abstraction layer over FATFS.

Core components:
* Filesystem Layer: Mount/unmount SD card and manage file access
* Binary File Interface: Open, validate, and read structured astronomical catalog files
* Query Engine: Perform efficient spatial queries using indexed binary data

## SD Card Data Access Pipeline
* Mount SD card filesystem
* Open binary catalog file
* Read and validate file header (magic number, bin counts)
* Use bin index table to locate relevant regions
* Seek directly to file offsets
* Read object records into RAM
* Return filtered results

## Search Pipeline
* Convert RA/Dec bounds into bin ranges
* Iterate over relevant bins only
* Use index table to jump to bin offsets
* Read object records sequentially
* Filter objects within bounds
* Return results to caller

## Features Implemented
* FATFS-based file management
* Custom binary format parsing
* Indexed spatial queries (RA/Dec bins)
* Efficient SD reads via direct file seeks
* Support for bounded and ID-based object lookup