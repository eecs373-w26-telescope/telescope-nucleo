# Telescope Astronomy Background

## Astronomy Basics
Astronomical objects are located on the celestial sphere using angular coordinates. The telescope does not directly measure these global coordinates; instead, it measures its local orientation (altitude and azimuth). This local orientation is different across the globe, and at different times. To identify objects in the sky, this local orientation must be converted into a global reference frame that is fixed relative to the stars.

This requires incorporating:  
* Observer location (latitude, longitude)
* Absolute time (UTC)
* Earth’s rotation (via sidereal time)

Using these, the system maps a physical pointing direction to a unique position on the celestial sphere (Right Ascension and Declination), which can then be used to query astronomical catalogs.

## Coordinate Systems  
The system uses three primary coordinate systems:

**Geographic Coordinates**  
Defines the observer’s position on Earth:
* Latitude (−90° to 90°)
* Longitude (−180° to 180°)

This is used to relate local observations to the global celestial frame.

**Horizontal Coordinates (Alt/Az)**  
Represents the telescope’s local pointing direction: 
* Altitude: angle above the horizon
* Azimuth: angle around the horizon (0–360°)

This is directly measured from hardware (encoders + IMU offsets).

**Equatorial Coordinates (RA/Dec)**

Represents fixed positions on the celestial sphere:  
* Right Ascension (RA): measured in hours (0–24h)
* Declination (Dec): measured in degrees (−90° to 90°)

This coordinate system is independent of the observer and is used for catalog storage and lookup.

**Transformations**

The program performs full transformations between coordinate systems:

Alt/Az → RA/Dec pipeline:
1. Convert angles to radians
2. Compute declination using spherical trigonometry
3. Compute hour angle (H)
4. Compute Julian Date from UTC
5. Compute Greenwich Mean Sidereal Time (GMST)
6. Convert to Local Sidereal Time (LST) using longitude
7. Compute Right Ascension from LST and hour angle

This allows mapping the telescope’s current pointing direction into global sky coordinates.

## Usage in Program
The astronomy module operates as the central pipeline connecting hardware pose to object lookup.

**Pose Update**  
The system receives:
* Altitude and azimuth (from encoders)
* Latitude and longitude (from GPS)
* UTC time

These are stored and used to compute the telescope’s equatorial position.

**FOV Computation**  
The telescope’s view is modeled as a spherical Field of View (FOV):
* Centered at current RA/Dec
* Defined by an angular radius (based on optics)

This FOV represents the region of the sky currently visible.

**Object Search Pipeline**  
The system identifies visible objects using a multi-stage process:
* Reuse objects from previous frame that still lie within the FOV (sliding window)
* Compute a coarse RA/Dec bounding box around the FOV
* Query the SD card database for objects within this region
* Apply exact angular distance filtering
* Update the current list of visible objects

**Projection for Display**  
To render objects on a screen aligned with the telescope view:
* Objects and FOV center are converted back to horizontal coordinates
* A gnomonic projection is applied
* Objects are mapped to normalized screen coordinates

This ensures objects appear correctly positioned relative to the telescope’s view direction.

## Design Decisions
**Separation of Coordinate Systems**  
Explicit struct definitions for Geographic, Horizontal, and Equatorial coordinates ensure clarity and prevent unit/representation errors. This also simplifies transformations and debugging.


**Use of Equatorial Frame for Storage**  
All catalog data is stored in RA/Dec, since it is time-invariant and independent of observer position. This avoids recomputing object positions and simplifies database design.


**Sliding Window Optimization**  
Instead of recomputing all visible objects every frame, the system:
* Retains overlapping objects between consecutive FOVs
* Only queries newly exposed regions  

This reduces SD card I/O and improves responsiveness.


**Spatial Binning for Database Queries**
* Objects are partitioned into RA/Dec bins in the binary file. This allows:
* Direct lookup of relevant sky regions
* Avoidance of full dataset scans
* Efficient use of limited memory and storage bandwidth
* Embedded-Friendly Math
* All computations use floats and lightweight math functions
* Trigonometric inputs are clamped to prevent floating-point drift errors
* Angles are consistently handled with explicit degree/radian conversions


**Projection Choice (Gnomonic)**  
Gnomonic projection was chosen because:
* Great circles map to straight lines
* It preserves relative geometry near the center of the FOV
* It is well-suited for small-angle telescope views