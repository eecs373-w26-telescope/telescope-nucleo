# Telescope State Machine Implementation Description
The state machine acts as the central logic hub for the telescope, managing transitions between idle tracking, object searching, and target acquisition. It integrates sensor data with the astronomy engine to drive both the local display and the Raspberry Pi HUD.

## Software Architecture
The system is implemented as a discrete state machine in `state_machine.h`. It is updated at 10Hz by the main controller loop.

Core Components:
* **State Manager**: Handles transitions between INIT, IDLE, SEARCH, and FOUND states.
* **Event Sampler**: Polls the touchscreen and internal flags to trigger transitions (e.g., search requests or cancellations).
* **Target Resolver**: Coordinates with the SD card to find objects by ID when they aren't already in the local FOV.
* **Guidance Engine**: Calculates unit vectors and angular distances for real-time slewing assistance.

## State Machine Pipeline
* Update local pose using current sensor data (IMU, Encoders, GPS)
* Refresh the list of visible objects in the current Field of View
* Sample user input events from the touchscreen
* Handle state-specific logic:
    * **IDLE**: Monitor for search requests while updating current RA/Dec.
    * **SEARCH**: Compute direction vectors to the selected target.
    * **FOUND**: Verify the target remains within the FOV radius.
* Broadcast state and guidance packets to the Raspberry Pi.

## Features Implemented
* Four-state logic flow (INIT/IDLE/SEARCH/FOUND)
* Automatic target acquisition when slewing into range
* Real-time guidance vector calculation
* Centralized pose and FOV management
* Fallback time handling (GPS -> RasPi -> Compile Time)
