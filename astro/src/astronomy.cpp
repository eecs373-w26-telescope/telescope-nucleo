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
    const EquatorialCoordinates& b) {
    const double ra1 = deg2rad(a.right_ascension * 15.0);
    const double ra2 = deg2rad(b.right_ascension * 15.0);
    const double dec1 = deg2rad(a.declination);
    const double dec2 = deg2rad(b.declination);

    double cos_theta = std::sin(dec1) * std::sin(dec2) +
    std::cos(dec1) * std::cos(dec2) * std::cos(ra1 - ra2);

    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return rad2deg(std::acos(cos_theta));
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
    az_rad = M_PI - az_rad;
    double lat_rad = deg2rad(gc.latitude);
    
    // 1.) calculate declination (dec)
    double sin_dec = std::sin(alt_rad) * std::sin(lat_rad) + 
                     std::cos(alt_rad) * std::cos(lat_rad)*std::cos(az_rad);
    
    // clamp against floating point drift
    sin_dec = std::max(-1.0, std::min(1.0, sin_dec));
    double dec_rad = std::asin(sin_dec);

    // 2.) calculate local hour angle (H)
    // Guard against pole singularity: cos(dec) -> 0 when dec -> ±90°,
    // which happens when pointing at the celestial pole. H is undefined there;
    // use H=0 (telescope on meridian) as a safe fallback.
    const double cos_dec = std::cos(dec_rad);
    double H_rad;
    if (std::abs(cos_dec) < 1e-9) {
        H_rad = 0.0;
    } else {
        double sin_H = (-std::sin(az_rad) * std::cos(alt_rad)) / cos_dec;
        double cos_H = (std::sin(alt_rad) - std::sin(lat_rad)*std::sin(dec_rad)) /
                       (std::cos(lat_rad) * cos_dec);
        H_rad = std::atan2(sin_H, cos_H);
    }

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
    // current_FOV.radius = telescope.approximate_FOV_radius_deg(); // TODO: IMPLEMENT
    current_FOV.radius = 50.0f;
    current_FOV.objects.clear();
}

bool Astronomy::is_object_in_FOV(const DSO& object, const FOV& fov) {
    return angular_distance_deg(object.pos, fov.center_pos) <= fov.radius;
}

float Astronomy::calculate_object_distance_from_FOV(const DSO& object, const FOV& fov) {
    return static_cast<float>(angular_distance_deg(object.pos, fov.center_pos));
}

std::vector<DSO> Astronomy::intersected_points(const FOV& new_fov, const FOV& old_FOV) {
    std::vector<DSO> intersected_objects;

    for (const auto& object : old_FOV.objects) {
        if (is_object_in_FOV(object, new_fov)) {
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

        const float cos_dec = std::cos(center_dec_deg * static_cast<float>(M_PI) / 180.0f);
        // Cap ra_stretch at 180 so that near-pole pointings still produce a valid
        // [ra_min, ra_max] range that search_objects_in_bounds can handle (it detects
        // full-circle coverage when the span >= 360).
        const float ra_stretch = (cos_dec > 0.01f) ? std::min(fov.radius / cos_dec, 180.0f) : 180.0f;

        ra_min_deg = center_ra_deg - ra_stretch;
        ra_max_deg = center_ra_deg + ra_stretch;
        dec_min_deg = center_dec_deg - fov.radius;
        dec_max_deg = center_dec_deg + fov.radius;

        if (dec_min_deg < -90.0f) dec_min_deg = -90.0f;
        if (dec_max_deg > 90.0f) dec_max_deg = 90.0f;
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
    // Only refresh the cache on a successful SD read. On error, keep old_FOV
    // intact so the next tick can still intersect against previously found objects.
    if (search_res == 0) {
        old_FOV = current_FOV;
    }
    return search_res;
}

HorizontalCoordinates Astronomy::get_horizontal() const {
    return hc;
}