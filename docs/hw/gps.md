# GPS over UART Implementation Description
The GPS module provides real-time positioning and timing information for the telescope system. It receives standard NMEA sentences from the GPS sensor and extracts the key fields needed by the project, including UTC time, date, latitude, longitude, fix validity, and satellite count. The module is designed to continuously update the latest valid GPS data and make it available to the rest of the system.

## Hardware Interface
The GPS sensor is connected to the STM32 through a UART interface. The module sends serial NMEA messages continuously, and the microcontroller receives the incoming byte stream for parsing. The GPS hardware itself is responsible for satellite acquisition and sentence generation, while the software layer focuses on decoding and storing useful navigation data.

## Software Architecture
The primary code for GPS is housed in GPS.cpp
* Initializes the GPS reception buffer and starts the UART receive process. Continuously reads incoming bytes, reconstructs complete NMEA sentences, and dispatches them for parsing.
* Determines whether an incoming NMEA sentence is an RMC message. Only supported sentence types are parsed, while other messages are ignored.
* Extracts UTC time, date, latitude, longitude, and fix validity from RMC messages. Converts latitude and longitude from NMEA degree-minute format into signed decimal degrees for easier downstream use.
* Provides simplified interfaces for the rest of the system. has_fix() reports whether the latest GPS data is valid, while payload() packages latitude, longitude, and satellite count into a compact structure for controller use.

## GPS Data Processing Pipeline
* Receive NMEA data from the GPS module
* Reconstruct complete sentences
* Identify supported message types
* Parse key fields from RMC messages
* Store the latest valid GPS state
* Provide processed data to the controller

## Features Implemented
* UTC time parsing
* Date parsing
* Latitude and longitude parsing
* Fix validity detection
* Satellite count extraction
* Compact GPS payload generation for controller use
