#include "../inc/astronomy.hpp"

#include <cmath>
#include <unordered_map>

//TODO: WHEN TO RESET POSE? IN STATE MACHINE OR FUNC? probs SM?
//TODO: make sure there are no timing issues with the intermediate funcs (e.g. convert pose to horizontal calculating RA/decl, then embedded search using outdated RA/decl)
//TODO: MAKE SURE CONSISTENT RAD OR DEG


Astronomy::Astronomy(const Telescope& telescope, telescope::SDCard& db)
    : telescope(telescope), db(db) {}


double Astronomy::deg2rad(double deg) {return deg * M_PI / 180.0;}
double Astronomy::rad2deg(double rad) {return rad * 180.0 / M_PI;}
double Astronomy::wrap24(double x) {
    x = std::fmod(x, 24.0);
    if (x < 0) x+= 24.0;
    return x;
}

double Astronomy::angular_distance_deg(const EquatorialCoordinates& a,
    const EquatorialCoordinates& b) const {
    const double ra1 = deg2rad(a.right_ascension * 15.0);
    const double ra2 = deg2rad(b.right_ascension * 15.0);
    const double dec1 = deg2rad(a.declination);
    const double dec2 = deg2rad(b.declination);

    double cos_theta = std::sin(dec1) * std::sin(dec2) +
    std::cos(dec1) * std::cos(dec2) * std::cos(ra1 - ra2);

    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return static_cast<float>(rad2deg(std::acos(cos_theta)));
}

int Astronomy::calculate_adjusted_altitude(int raw_altitude_deg) const {
    // INTEGRATION
    return raw_altitude_deg - altitude_offset_deg;
}

int Astronomy::calculate_adjusted_azimuth(int raw_azimuth_deg, int yaw_deg) const {
    // INTEGRATION
    int adjusted = raw_azimuth_deg + yaw_deg - azimuth_offset_deg;
    adjusted %= 360;
    if (adjusted < 0) adjusted += 360;
    return adjusted;
}

void Astronomy::update_pose(float altitude_deg, float azimuth_deg,
                            float latitude, float longitude,
                            const UTC& time) {
    gc.latitude = latitude;
    gc.longitude = longitude;
    utc = time;
    hc.altitude = altitude_deg;
    hc.azimuth = azimuth_deg;
}

void Astronomy::convert_hc_to_eqc() {
    // 0.) convert from deg to radians to make trig easier
    double alt_rad = deg2rad(hc.altitude);
    double az_rad  = deg2rad(hc.azimuth);
    double lat_rad = deg2rad(gc.latitude);
    
    // 1.) calculate declination (dec)
    double sin_dec = std::sin(alt_rad) * std::sin(lat_rad) + 
                     std::cos(alt_rad) * std::cos(lat_rad)*std::cos(az_rad);
    
    // clamp against floating point drift
    sin_dec = std::max(-1.0, std::min(1.0, sin_dec));
    double dec_rad = std::asin(sin_dec);

    // 2.) calculate local hour angle (H)
    double sin_H = (-std::sin(az_rad) * std::cos(alt_rad)) / std::cos(dec_rad);
    double cos_H = (std::sin(alt_rad) - std::sin(lat_rad)*std::sin(dec_rad)) / 
                   (std::cos(lat_rad) * std::cos(dec_rad));
    
    double H_rad = std::atan2(sin_H, cos_H);

    // 3.) calculate local sidereal time
    // calculate julian date (Y, M, D) from UTC (h, m, s)
    int year  = utc.year;
    int month = utc.month;
    int day   = utc.day;
    int hour  = utc.hour;
    int minute = utc.minute;
    int second = utc.second;

    if (month <= 2) {
        year -= 1;
        month += 12;
    }

    int A = static_cast<int>(std::floor(year / 100.0));
    int B = 2 - A + static_cast<int>(std::floor(A / 4.0));

    double day_frac = day +
                      (hour / 24.0) +
                      (minute / 1440.0) +
                      (second / 86400.0);

    double JD = std::floor(365.25 * (year + 4716)) +
                std::floor(30.6001 * (month + 1)) +
                day_frac + B - 1524.5;

    double d = JD - 2451545.0;
    double GMST_hours = wrap24(18.697374558 + 24.06570982441908 * d);
    double LST_hours  = wrap24(GMST_hours + gc.longitude / 15.0);

    double H_hours = H_rad * 12.0 / M_PI;
    double RA_hours = wrap24(LST_hours - H_hours);

    eqc.right_ascension = static_cast<float>(RA_hours);
    eqc.declination = static_cast<float>(rad2deg(dec_rad));
}


void Astronomy::calculate_FOV() {
    current_FOV.center_pos = eqc;
    //current_FOV.radius = telescope.approximate_FOV_radius_deg(); // TODO: IMPLEMENT
    current_FOV.radius = 10.0f;
    current_FOV.objects.clear();
}

bool Astronomy::is_object_in_FOV(const DSO& object, const FOV& fov) {
    return angular_distance_deg(object.pos, fov.center_pos) <= fov.radius;
}

float Astronomy::calculate_object_distance_from_FOV(const DSO& object, const FOV& fov) const {
    return static_cast<float>(angular_distance_deg(object.pos, fov.center_pos));
}

std::vector<DSO> Astronomy::intersected_points(const FOV& new_fov, const FOV& old_FOV) {
    std::vector<DSO> intersected_objects;

    for (const auto& object : old_FOV.objects) {
        if (is_object_in_FOV(object, old_FOV) && is_object_in_FOV(object, new_fov)) {
            intersected_objects.emplace_back(object);
        }
    }

    return intersected_objects;
}

void Astronomy::compute_equatorial_bounds(const FOV& fov, float& ra_min_deg, float& ra_max_deg,
                                          float& dec_min_deg, float& dec_max_deg) const {
        // Coarse bounding box, good enough for SD lookup prefilter
        const float center_ra_deg = fov.center_pos.right_ascension * 15.0f;
        const float center_dec_deg = fov.center_pos.declination;

        ra_min_deg = center_ra_deg - fov.radius;
        ra_max_deg = center_ra_deg + fov.radius;
        dec_min_deg = center_dec_deg - fov.radius;
        dec_max_deg = center_dec_deg + fov.radius;

        if (dec_min_deg < -90.0f) dec_min_deg = -90.0f;
        if (dec_max_deg > 90.0f) dec_max_deg = 90.0f;

        // TODO: for now, assume no RA wraparound handling.
        // Add wrap logic later if FOV crosses 0h / 360°.
}


int Astronomy::find_objects_within_FOV() {
    // Pipeline:
    // (1) keep overlap from old FOV in RAM
    // (2) search SD only for the coarse new window
    // (3) exact-filter in RAM
    // (4) update old/current FOV

    std::vector<DSO> new_objects = intersected_points(current_FOV, old_FOV);

    float ra_min_deg = 0.0f;
    float ra_max_deg = 0.0f;
    float dec_min_deg = 0.0f;
    float dec_max_deg = 0.0f;

    compute_equatorial_bounds(current_FOV, ra_min_deg, ra_max_deg, dec_min_deg, dec_max_deg);

    std::vector<DSO> candidates;
    int search_res = db.search_objects_in_bounds(ra_min_deg, ra_max_deg, dec_min_deg, dec_max_deg, candidates);

    for (const auto& obj : candidates) {
        if (!is_object_in_FOV(obj, current_FOV)) {
            continue;
        }

        bool already_present = false;
        for (const auto& kept : new_objects) {
            if (kept.name == obj.name) {
                already_present = true;
                break;
            }
        }

        if (!already_present) {
            new_objects.emplace_back(obj);
        }
    }

    current_FOV.objects = std::move(new_objects);
    old_FOV = current_FOV;
    return search_res;
}

HorizontalCoordinates Astronomy::get_horizontal() const {
    return hc;
}

HorizontalCoordinates Astronomy::get_target_horizontal(const EquatorialCoordinates& target_eqc) const {
    // Recompute LST from the currently stored UTC/location
    int year   = utc.year;
    int month  = utc.month;
    int day    = utc.day;
    int hour   = utc.hour;
    int minute = utc.minute;
    int second = utc.second;

    if (month <= 2) {
        year -= 1;
        month += 12;
    }

    const int A = static_cast<int>(std::floor(year / 100.0));
    const int B = 2 - A + static_cast<int>(std::floor(A / 4.0));

    const double day_frac =
        day + (hour / 24.0) + (minute / 1440.0) + (second / 86400.0);

    const double JD =
        std::floor(365.25 * (year + 4716)) +
        std::floor(30.6001 * (month + 1)) +
        day_frac + B - 1524.5;

    const double d = JD - 2451545.0;
    const double GMST_hours = wrap24(18.697374558 + 24.06570982441908 * d);
    const double LST_hours  = wrap24(GMST_hours + gc.longitude / 15.0);

    double H_hours = LST_hours - target_eqc.right_ascension;
    if (H_hours < -12.0) H_hours += 24.0;
    if (H_hours >  12.0) H_hours -= 24.0;
    const double H_rad = H_hours * M_PI / 12.0;

    const double lat_rad = deg2rad(gc.latitude);
    const double dec_rad = deg2rad(target_eqc.declination);

    // Calculate Altitude
    double sin_alt = std::sin(lat_rad) * std::sin(dec_rad) +
                     std::cos(lat_rad) * std::cos(dec_rad) * std::cos(H_rad);
    sin_alt = std::max(-1.0, std::min(1.0, sin_alt));
    const double alt_rad = std::asin(sin_alt);

    // Calculate Azimuth
    const double y = -std::sin(H_rad) * std::cos(dec_rad);
    const double x = std::cos(lat_rad) * std::sin(dec_rad) - std::sin(lat_rad) * std::cos(dec_rad) * std::cos(H_rad);
    const double az_rad = std::atan2(y, x);

    HorizontalCoordinates target_hc;
    target_hc.altitude = static_cast<float>(rad2deg(alt_rad));
    double az_deg = rad2deg(az_rad);
    if (az_deg < 0.0) az_deg += 360.0;
    target_hc.azimuth = static_cast<float>(az_deg);

    return target_hc;
}

double Astronomy::compute_parallactic_angle_rad(const EquatorialCoordinates& center) const {
    const double lat = deg2rad(gc.latitude);
    const double dec = deg2rad(center.declination);

    // Recompute LST from the currently stored UTC/location
    int year   = utc.year;
    int month  = utc.month;
    int day    = utc.day;
    int hour   = utc.hour;
    int minute = utc.minute;
    int second = utc.second;

    if (month <= 2) {
        year -= 1;
        month += 12;
    }

    const int A = static_cast<int>(std::floor(year / 100.0));
    const int B = 2 - A + static_cast<int>(std::floor(A / 4.0));

    const double day_frac =
        day + (hour / 24.0) + (minute / 1440.0) + (second / 86400.0);

    const double JD =
        std::floor(365.25 * (year + 4716)) +
        std::floor(30.6001 * (month + 1)) +
        day_frac + B - 1524.5;

    const double d = JD - 2451545.0;
    const double GMST_hours = wrap24(18.697374558 + 24.06570982441908 * d);
    const double LST_hours  = wrap24(GMST_hours + gc.longitude / 15.0);

    double H_hours = LST_hours - center.right_ascension;
    if (H_hours < -12.0) H_hours += 24.0;
    if (H_hours >  12.0) H_hours -= 24.0;

    const double H = H_hours * M_PI / 12.0;

    return std::atan2(
        std::sin(H),
        std::tan(lat) * std::cos(dec) - std::sin(dec) * std::cos(H)
    );
}

void Astronomy::project_gnomonic(const EquatorialCoordinates& center,
                                 const EquatorialCoordinates& obj,
                                 float fov_radius_deg,
                                 float& x_out, float& y_out) {
    const double ra0  = center.right_ascension * 15.0 * M_PI / 180.0;
    const double dec0 = center.declination * M_PI / 180.0;
    const double ra   = obj.right_ascension * 15.0 * M_PI / 180.0;
    const double dec  = obj.declination * M_PI / 180.0;

    const double dRA = ra - ra0;

    const double D =
        std::sin(dec0) * std::sin(dec) +
        std::cos(dec0) * std::cos(dec) * std::cos(dRA);

    // If D <= 1e-6, the object is 90 degrees or more away from the pointing center.
    // It is physically behind the focal plane and cannot be projected.
    if (D <= 1e-6) {
        // Assign a massive coordinate to force it out of bounds 
        // so the guidance distance reads as extremely large.
        x_out = 9999.0f;
        y_out = 9999.0f;
        return;
    }

    const double fov_radius_rad = fov_radius_deg * M_PI / 180.0;

    // Raw equatorial-frame gnomonic projection
    const double x_eq =
        (std::cos(dec) * std::sin(dRA) / D) / fov_radius_rad;

    const double y_eq =
        ((std::cos(dec0) * std::sin(dec) -
            std::sin(dec0) * std::cos(dec) * std::cos(dRA)) / D) / fov_radius_rad;

    x_out = static_cast<float>(x_eq);
    y_out = static_cast<float>(y_eq);
}

void Astronomy::project_gnomonic_local(const EquatorialCoordinates& center,
    const EquatorialCoordinates& obj,
    float fov_radius_deg,
    float& x_out, float& y_out) const {
    float x_eq = 0.0f;
    float y_eq = 0.0f;

    Astronomy::project_gnomonic(center, obj, fov_radius_deg, x_eq, y_eq);

    const double q = compute_parallactic_angle_rad(center);

    // Rotate equatorial projection into local horizon-aligned screen axes
    const double x_rot =  x_eq * std::cos(q) + y_eq * std::sin(q);
    const double y_rot = -x_eq * std::sin(q) + y_eq * std::cos(q);

    // screen rotation to align with display physical orientation
    x_out = static_cast<float>(-y_rot);
    y_out = static_cast<float>( x_rot);
}

void Astronomy::project_gnomonic_parallactic(const EquatorialCoordinates& center,
    const EquatorialCoordinates& obj,
    float fov_radius_deg,
    float& x_out, float& y_out) const {
    float x_eq = 0.0f;
    float y_eq = 0.0f;

    Astronomy::project_gnomonic(center, obj, fov_radius_deg, x_eq, y_eq);

    const double q = compute_parallactic_angle_rad(center);

    // Invert X-axis: RA increases East (Left). 
    // Negate to map X to West (Right) so it aligns with positive mount Azimuth.
    x_eq = -x_eq;

    // Rotate equatorial projection into local horizon-aligned axes (Right, Up)
    const double x_rot =  x_eq * std::cos(q) + y_eq * std::sin(q);
    const double y_rot = -x_eq * std::sin(q) + y_eq * std::cos(q);

    x_out = static_cast<float>(x_rot);
    y_out = static_cast<float>(y_rot);
}