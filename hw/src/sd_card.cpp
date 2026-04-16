#include <hw/inc/sd_card.hpp>

static constexpr uint32_t MAGIC = 0x44534F31;
static constexpr const char* MOUNT_PATH = ""; // default

namespace telescope {
int SDCard::mount() {
    if (mounted_) return 0;

    FRESULT res = f_mount(&fs_, MOUNT_PATH, 1);
    mounted_ = (res == FR_OK);
    return mounted_ ? 0 : -1;
}

int SDCard::unmount() {
    if (!mounted_) return 0;

    FRESULT res = f_mount(nullptr, MOUNT_PATH, 1);
    mounted_ = false;
    return (res == FR_OK) ? 0 : -1;
}

int SDCard::open_catalogue(const char* path) {
    if (!mounted_) return -1;
    if (file_open_) return 0;

    FRESULT res = f_open(&file_, path, FA_READ);
    file_open_ = (res == FR_OK);
    if (!file_open_) return -1;

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
                                          std::vector<DSO>& out_objects) {
    if (!file_open_ || !header_valid_) {
        return -1;
    }

    // Coarse bin bounds
    const int ra_bin_min = compute_bin_id(ra_min_deg, dec_min_deg, header_.ra_bins, header_.dec_bins) % header_.ra_bins;
    const int ra_bin_max = compute_bin_id(ra_max_deg, dec_min_deg, header_.ra_bins, header_.dec_bins) % header_.ra_bins;

    const int dec_bin_min = std::max(0, static_cast<int>((dec_min_deg + 90.0f) / (180.0f / header_.dec_bins)));
    const int dec_bin_max = std::min(static_cast<int>(header_.dec_bins) - 1,
                                     static_cast<int>((dec_max_deg + 90.0f) / (180.0f / header_.dec_bins)));

    // Simple version: assume no RA wrap for now
    for (int dec_bin = dec_bin_min; dec_bin <= dec_bin_max; ++dec_bin) {
        for (int ra_bin = ra_bin_min; ra_bin <= ra_bin_max; ++ra_bin) {
            const uint32_t bin_id = dec_bin * header_.ra_bins + ra_bin;

            BinIndex idx{};
            if (read_bin_index(bin_id, idx) != 0) {
                return -1;
            }

            if (idx.count == 0) {
                continue;
            }

            if (f_lseek(&file_, idx.offset) != FR_OK) {
                return -1;
            }

            for (uint32_t i = 0; i < idx.count; ++i) {
                BinObjectRecord rec{};
                if (read_object_at_current_pos(rec) != 0) {
                    return -1;
                }

                if (rec.ra_deg >= ra_min_deg && rec.ra_deg <= ra_max_deg &&
                    rec.dec_deg >= dec_min_deg && rec.dec_deg <= dec_max_deg) {
                    DSO obj{};
                    obj.name = std::to_string(rec.id); // replace with real name lookup later
                    obj.pos.right_ascension = rec.ra_deg / 15.0f; // convert deg -> hours
                    obj.pos.declination = rec.dec_deg;
                    obj.brightness = rec.mag;
                    out_objects.emplace_back(obj);
                }
            }
        }
    }

    return 0;
}
} // namespace telescope