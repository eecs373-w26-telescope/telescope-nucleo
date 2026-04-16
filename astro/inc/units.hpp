#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct GeographicCoordinates {
    float longitude;
    float latitude;
};

struct EquatorialCoordinates {
    float right_ascension;
    float declination;

    // overloaded subtraction operator
    EquatorialCoordinates operator-(const EquatorialCoordinates& other) const {
        return {right_ascension - other.right_ascension, declination - other.declination};
    }

    // overloaded addition operator
    EquatorialCoordinates operator+(const EquatorialCoordinates& other) const {
        return {right_ascension + other.right_ascension, declination + other.declination};
    }
};

struct HorizontalCoordinates {
    float altitude;
    float azimuth;
};

struct UTC {
    int year, month, day, hour, minute, second;
};

struct GraphicsCommand {
    std::string object;
    EquatorialCoordinates position;
};

struct DSO {
    std::string name;
    EquatorialCoordinates pos;
    float brightness;
};

struct FOV {
    EquatorialCoordinates center_pos;
    float radius; //angular rad in deg
    std::vector<DSO> objects;
};

// TODO: mildly out of place?
struct Telescope {
    float objective_lens_diameter{1.25f}; //m
    float eyepiece_focal_length{25.0f}; //mm
    float telescope_focal_length{1200.0f}; //mm

    // calculate FOV?
    int approximate_FOV_radius_deg() {return 0;}
};