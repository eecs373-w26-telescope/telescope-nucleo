# SD Card over Parallel 4-Bit SDIO Implementation Description
The SD card module provides persistent storage and efficient querying of astronomical object data using a custom binary format. It utilizes a 4-bit SDIO interface and the FATFS filesystem to enable high-speed access to large datasets.

## Hardware Interface
The SD card interfaces with the STM32 via a 4-bit SDIO bus on the Feather board. This interface supports high-speed data transfers required for smooth real-time catalog queries.

## Software Architecture
The SD card system (`sd_card.cpp`) is built as a hardware abstraction layer over FATFS.

Core Components:
* **Filesystem Layer**: Manages mounting/unmounting and file handle operations.
* **Binary Format Parser**: Validates file headers (magic numbers, bin counts) and interprets the spatial index.
* **Spatial Query Engine**: Performs bounded RA/Dec searches by jumping directly to relevant file offsets using a pre-calculated index table.

## SD Card Data Access Pipeline
* Mount SD card filesystem
* Open binary catalog file (e.g., `messier.bin`)
* Read and validate file header
* Use bin index table to locate relevant sky regions
* Seek directly to file offsets using `f_lseek`
* Read object records into a local buffer
* Filter objects by exact coordinates and magnitude

## Search Pipeline
* Convert target RA/Dec bounds into bin ranges
* Iterate over relevant bins only (avoiding full database scans)
* Use the index table to jump to specific bin offsets
* Read object records sequentially within the bin
* Apply angular distance filtering for precise matches
* Return filtered results to the state machine

## Features Implemented
* FATFS-based file management over 4-bit SDIO
* Custom binary format with grid-based spatial indexing
* Efficient disk I/O via direct file seeking
* Support for both bounded searches and ID-based lookups
* Magnitude-sorted bins for prioritized object retrieval
