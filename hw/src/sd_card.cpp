#include <hw/inc/sd_card.hpp>

extern FATFS SDFatFS;
extern char SDPath[4];

static constexpr uint32_t MAGIC = 0x44534F31;

namespace telescope {
int SDCard::mount() {
    if (mounted_) return 0;

    FRESULT res = f_mount(&SDFatFS, SDPath, 1);
    mounted_ = (res == FR_OK);
    return mounted_ ? 0 : -1;
}

int SDCard::unmount() {
    if (!mounted_) return 0;

    FRESULT res = f_mount(nullptr, SDPath, 1);
    mounted_ = false;
    return (res == FR_OK) ? 0 : -1;
}

int SDCard::open_catalogue(const char* path) {
    if (!mounted_) return -1;
    if (file_open_) return 0;

    FRESULT res = f_open(&file_, path, FA_READ);
    file_open_ = (res == FR_OK);
    if (!file_open_) { last_open_err_ = static_cast<int>(res); return -1; }

    FileHeader header{};
    if(read_header(header) != 0) {close_catalogue(); return -1;}

    return 0;
}


int SDCard::close_catalogue() {
    if (!file_open_) {
        return 0;
    }
    FRESULT res = f_close(&file_);
    file_open_ = false;
    header_valid_ = false;
    return (res == FR_OK) ? 0 : -1;
}

int SDCard::read_header(FileHeader& header_out) {
    if (!file_open_) {
        return -1;
    }

    if (f_lseek(&file_, 0) != FR_OK) {
        return -1;
    }

    UINT br = 0;
    if (f_read(&file_, &header_out, sizeof(FileHeader), &br) != FR_OK || br != sizeof(FileHeader)) {
        return -1;
    }

    if (header_out.magic != MAGIC) {
        return -1;
    }

    header_ = header_out;
    header_valid_ = true;
    return 0;
}

int SDCard::compute_bin_id(float ra_deg, float dec_deg, uint16_t ra_bins, uint16_t dec_bins) {
    const float ra_norm = std::fmod(std::fmod(ra_deg, 360.0f) + 360.0f, 360.0f);
    const float dec_clamped = std::max(-90.0f, std::min(90.0f, dec_deg));

    const float ra_bin_width = 360.0f / static_cast<float>(ra_bins);
    const float dec_bin_width = 180.0f / static_cast<float>(dec_bins);

    int ra_bin = static_cast<int>(ra_norm / ra_bin_width);
    int dec_bin = static_cast<int>((dec_clamped + 90.0f) / dec_bin_width);

    if (ra_bin >= ra_bins) ra_bin = ra_bins - 1;
    if (dec_bin >= dec_bins) dec_bin = dec_bins - 1;
    if (dec_bin < 0) dec_bin = 0;

    return dec_bin * ra_bins + ra_bin;
}

int SDCard::read_bin_index(uint32_t bin_id, BinIndex& out_index) {
    if (!header_valid_) {
        return -1;
    }

    const uint32_t offset = sizeof(FileHeader) + bin_id * sizeof(BinIndex);

    if (f_lseek(&file_, offset) != FR_OK) {
        return -1;
    }

    UINT br = 0;
    if (f_read(&file_, &out_index, sizeof(BinIndex), &br) != FR_OK || br != sizeof(BinIndex)) {
        return -1;
    }

    return 0;
}

int SDCard::read_object_at_current_pos(BinObjectRecord& out_record) {
    UINT br = 0;
    if (f_read(&file_, &out_record, sizeof(BinObjectRecord), &br) != FR_OK ||
        br != sizeof(BinObjectRecord)) {
        return -1;
    }
    return 0;
}

int SDCard::search_objects_in_bounds(float ra_min_deg,
                                          float ra_max_deg,
                                          float dec_min_deg,
                                          float dec_max_deg,
                                          std::vector<DSO>& out_objects,
                                          size_t max_count) {
    if (!file_open_ || !header_valid_) {
        return -1;
    }

    const int n_ra = static_cast<int>(header_.ra_bins);
    const float ra_bin_width = 360.0f / static_cast<float>(n_ra);

    // Normalize to [0, 360) and detect wrap
    const float ra_span = ra_max_deg - ra_min_deg;
    const bool full_circle = ra_span >= 360.0f;
    const float ra_lo = std::fmod(std::fmod(ra_min_deg, 360.0f) + 360.0f, 360.0f);
    const float ra_hi = std::fmod(std::fmod(ra_max_deg, 360.0f) + 360.0f, 360.0f);
    const bool ra_wraps = !full_circle && (ra_lo > ra_hi);

    const int ra_bin_lo = static_cast<int>(ra_lo / ra_bin_width);
    const int ra_bin_hi = std::min(static_cast<int>(ra_hi / ra_bin_width), n_ra - 1);

    const int dec_bin_min = std::max(0, static_cast<int>((dec_min_deg + 90.0f) / (180.0f / header_.dec_bins)));
    const int dec_bin_max = std::min(static_cast<int>(header_.dec_bins) - 1,
                                     static_cast<int>((dec_max_deg + 90.0f) / (180.0f / header_.dec_bins)));

    auto scan_dec_ra = [&](int dec_bin, int ra_bin) -> int {
        const uint32_t bin_id = static_cast<uint32_t>(dec_bin) * header_.ra_bins + static_cast<uint32_t>(ra_bin);
        BinIndex idx{};
        if (read_bin_index(bin_id, idx) != 0) return -1;
        if (idx.count == 0) return 0;
        if (f_lseek(&file_, idx.offset) != FR_OK) return -1;

        for (uint32_t i = 0; i < idx.count; ++i) {
            if (max_count > 0 && out_objects.size() >= max_count) return 0;

            BinObjectRecord rec{};
            if (read_object_at_current_pos(rec) != 0) return -1;

            const float ra = std::fmod(std::fmod(rec.ra_deg, 360.0f) + 360.0f, 360.0f);
            const bool in_ra = full_circle
                ? true
                : (ra_wraps ? (ra >= ra_lo || ra <= ra_hi) : (ra >= ra_lo && ra <= ra_hi));

            if (in_ra && rec.dec_deg >= dec_min_deg && rec.dec_deg <= dec_max_deg) {
                DSO obj{};
                if (rec.type == 1) {
                    obj.name = "N" + std::to_string(rec.id);
                } else {
                    obj.name = "M" + std::to_string(rec.id);
                }
                obj.pos.right_ascension = rec.ra_deg / 15.0f;
                obj.pos.declination = rec.dec_deg;
                obj.brightness = rec.mag;
                out_objects.emplace_back(obj);
            }
        }
        return 0;
    };

    for (int dec_bin = dec_bin_min; dec_bin <= dec_bin_max; ++dec_bin) {
        if (max_count > 0 && out_objects.size() >= max_count) break;

        if (full_circle) {
            for (int ra_bin = 0; ra_bin < n_ra; ++ra_bin) {
                if (max_count > 0 && out_objects.size() >= max_count) break;
                if (scan_dec_ra(dec_bin, ra_bin) != 0) return -1;
            }
        } else if (!ra_wraps) {
            for (int ra_bin = ra_bin_lo; ra_bin <= ra_bin_hi; ++ra_bin) {
                if (max_count > 0 && out_objects.size() >= max_count) break;
                if (scan_dec_ra(dec_bin, ra_bin) != 0) return -1;
            }
        } else {
            for (int ra_bin = ra_bin_lo; ra_bin < n_ra; ++ra_bin) {
                if (max_count > 0 && out_objects.size() >= max_count) break;
                if (scan_dec_ra(dec_bin, ra_bin) != 0) return -1;
            }
            for (int ra_bin = 0; ra_bin <= ra_bin_hi; ++ra_bin) {
                if (max_count > 0 && out_objects.size() >= max_count) break;
                if (scan_dec_ra(dec_bin, ra_bin) != 0) return -1;
            }
        }
    }

    return 0;
}

bool SDCard::find_object_by_id(uint16_t id, uint16_t type, DSO& out_obj) {
    if (!file_open_ || !header_valid_) return false;

    // Scan all bins for the matching object
    for (uint32_t b = 0; b < header_.total_bins; ++b) {
        BinIndex idx{};
        if (read_bin_index(b, idx) != 0) continue;
        if (idx.count == 0) continue;
        if (f_lseek(&file_, idx.offset) != FR_OK) continue;

        for (uint32_t i = 0; i < idx.count; ++i) {
            BinObjectRecord rec{};
            if (read_object_at_current_pos(rec) != 0) break;

            if (rec.id == id && rec.type == type) {
                out_obj.name = (type == 1 ? "N" : "M") + std::to_string(rec.id);
                out_obj.pos.right_ascension = rec.ra_deg / 15.0f;
                out_obj.pos.declination = rec.dec_deg;
                out_obj.brightness = rec.mag;
                return true;
            }
        }
    }
    return false;
}
} // namespace telescope
