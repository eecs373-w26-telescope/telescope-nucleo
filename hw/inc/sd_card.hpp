/*
Interface to SD card
*/ 

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <cmath>

#include "ff.h"
#include <astro/inc/units.hpp>

namespace telescope {

class SDCard {
public:
    struct FileHeader {
        uint32_t magic;
        uint16_t ra_bins;
        uint16_t dec_bins;
        uint32_t total_bins;
    };

    struct BinIndex {
        uint32_t offset;
        uint32_t count;
    };

    struct BinObjectRecord {
        uint16_t id;
        uint16_t type; // 0=M, 1=N
        float ra_deg;
        float dec_deg;
        float mag;
    };

    SDCard() = default;

    int mount();
    int unmount();

    int open_catalogue(const char* path);
    int close_catalogue();

    bool is_open() const {return file_open_;}
    int last_open_error() const {return last_open_err_;}

    int read_header(FileHeader& header);

    int search_objects_in_bounds(float ra_min_deg, float ra_max_deg,
                                  float dec_min_deg, float dec_max_deg,
                                   std::vector<DSO>& out_objects,
                                   size_t max_count = 0);

    bool find_object_by_id(uint16_t id, uint16_t type, DSO& out_obj);

private:
    int read_bin_index(uint32_t bin_id, BinIndex& out_index);
    int read_object_at_current_pos(BinObjectRecord& out_record);
    static int compute_bin_id(float ra_deg, float dec_deg, uint16_t ra_bin, uint16_t dec_bins);

    FIL file_{};
    bool mounted_{false};
    bool file_open_{false};

    FileHeader header_{};
    bool header_valid_{false};
    int last_open_err_{0};
};

} //namespace telescope
