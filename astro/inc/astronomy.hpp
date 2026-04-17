#pragma once

#include <astro/inc/units.hpp>

#include <hw/inc/sd_card.hpp>

class Astronomy {
public:
    Astronomy(const Telescope& telescope, telescope::SDCard& sdcard);

    EquatorialCoordinates get_equatorial_coordinates() const {return eqc;}
    HorizontalCoordinates get_horizontal_coordinates() {return hc;}
    const FOV& get_current_fov() const {return current_FOV;}

    void update_pose(float altitude_deg, float azimuth_deg,
                     float latitude, float longitude,
                     const UTC& time);
    void convert_hc_to_eqc();
    void calculate_FOV();
    int find_objects_within_FOV();

    bool is_object_in_FOV(const DSO& object, const FOV& fov);
    float calculate_object_distance_from_FOV(const DSO& object, const FOV& fov);

    HorizontalCoordinates get_horizontal() const;

private:
    const Telescope telescope;
    telescope::SDCard& db;

    GeographicCoordinates gc;  // deg
    HorizontalCoordinates hc;  // shifted altitude/azimuth (deg)
    EquatorialCoordinates eqc; // shifted RA/declination (hours, deg)
    UTC utc;

    FOV old_FOV;
    FOV current_FOV;

    int azimuth_offset_deg{0};
    int altitude_offset_deg{0};

    double deg2rad(double deg);
    double rad2deg(double rad);
    double wrap24(double x);
    double angular_distance_deg(const EquatorialCoordinates& a, const EquatorialCoordinates& b);

    std::vector<DSO> intersected_points(const FOV& new_FOV, const FOV& old_FOV);
    void compute_equatorial_bounds(const FOV& fov, float& ra_min_deg, float& ra_max_deg,
                                   float& dec_min_deg, float& dec_max_deg) const;

    int calculate_adjusted_altitude(int raw_altitude_deg) const;
    int calculate_adjusted_azimuth(int raw_azimuth_deg, int yaw_deg) const;
};
