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

struct Telescope {
    float objective_lens_diameter{70.0f}; //mm
    float eyepiece_focal_length{20.0f}; //mm
    float telescope_focal_length{400.0f}; //mm

    // TODO: BELOW IS AN ESTIMATE:
    float eyepiece_apparent_fov_deg{50.0f};

    float magnification() const {
        if (eyepiece_focal_length <= 0.0f) return 1.0f;
        return telescope_focal_length / eyepiece_focal_length;
    }

    float approximate_FOV_radius_deg() const {
        const float mag = magnification();
        if (mag <= 0.0f) return 0.0f;
        return eyepiece_apparent_fov_deg / (2.0f * mag);
    }

    float jank_FOV_radius_deg() const {return 1.25;}
};