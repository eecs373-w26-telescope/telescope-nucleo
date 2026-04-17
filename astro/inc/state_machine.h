#pragma once

#include <algorithm>
#include <cstring>
#include <string>

#include <hw/inc/sd_card.hpp>
#include <hw/inc/touchscreen.hpp>
#include <hw/inc/raspi.hpp>
#include <hw/inc/protocol.h>

#include <astro/inc/astronomy.hpp>

namespace telescope {

class StateMachine {
public:
    enum class TelescopeState : uint8_t {
        INIT,
        IDLE,
        SEARCH,
        FOUND
    };

    StateMachine(Touchscreen& touchscreen,
                 SDCard& sdcard,
                 RasPi& raspi)
        : touchscreen_(touchscreen),
          sdcard_(sdcard),
          raspi_(raspi),
          telescope_cfg_{},
          astronomy_(telescope_cfg_, sdcard_) {}

    void init() {
        current_state_ = TelescopeState::INIT;
        selected_messier_id_ = -1;
        last_search_status_ = false;
        has_selected_object_ = false;
        selected_object_ = DSO{};
        state_sequence_ = 0;
        target_full_sent_ = false;
    }

    void update_sensors(float altitude_deg, float azimuth_deg,
                        float latitude, float longitude,
                        const UTC& time) {
        astronomy_.update_pose(altitude_deg, azimuth_deg,
                               latitude, longitude, time);
    }

    void tick() {
        switch (current_state_) {
        case TelescopeState::INIT:
            handle_init();
            break;
        case TelescopeState::IDLE:
            handle_idle();
            break;
        case TelescopeState::SEARCH:
            handle_search();
            break;
        case TelescopeState::FOUND:
            handle_found();
            break;
        }
    }

    TelescopeState current_state() const { return current_state_; }
    int selected_messier_id() const { return selected_messier_id_; }
    bool has_selected_object() const { return has_selected_object_; }
    const DSO& selected_object() const { return selected_object_; }
    int fov_object_count() const { return static_cast<int>(astronomy_.get_current_fov().objects.size()); }
    EquatorialCoordinates current_eqc() const { return astronomy_.get_equatorial_coordinates(); }
    const FOV& current_fov() const { return astronomy_.get_current_fov(); }
    int last_search_result() const { return last_search_result_; }
    HorizontalCoordinates current_hc() const {return astronomy_.get_horizontal();
}

private:
    void sample_search_events() {
        const bool search_now = touchscreen_.get_search_status();

        if (!last_search_status_ && search_now) {
            search_requested_ = true;
            selected_messier_id_ = touchscreen_.get_selected_messier_id();
        }

        if (last_search_status_ && !search_now) {
            cancel_requested_ = true;
        }

        last_search_status_ = search_now;
    }

    void update_fov_and_objects() {
        astronomy_.convert_hc_to_eqc();
        astronomy_.calculate_FOV();
        last_search_result_ = astronomy_.find_objects_within_FOV();
    }

    bool try_select_target_from_current_fov(int messier_id) {
        const auto& objs = astronomy_.get_current_fov().objects;
        const std::string target_name = "M" + std::to_string(messier_id);

        for (const auto& obj : objs) {
            if (obj.name == target_name) {
                selected_object_ = obj;
                has_selected_object_ = true;
                return true;
            }
        }

        has_selected_object_ = false;
        return false;
    }

    bool lookup_target_globally(int messier_id) {
        const std::string target_name = "M" + std::to_string(messier_id);
        std::vector<DSO> results;
        if (sdcard_.search_objects_in_bounds(0.0f, 359.99f, -90.0f, 90.0f, results) != 0) {
            return false;
        }
        for (const auto& obj : results) {
            if (obj.name == target_name) {
                selected_object_ = obj;
                has_selected_object_ = true;
                return true;
            }
        }
        return false;
    }

    void send_dso_target_packet() {
        DSOTargetPayload pkt{};
        pkt.status         = DSO_OK;
        pkt.catalog_number = static_cast<uint16_t>(selected_messier_id_);
        if (has_selected_object_) {
            pkt.ra_mas       = static_cast<int32_t>(selected_object_.pos.right_ascension * 54000000.0f);
            pkt.dec_mas      = static_cast<int32_t>(selected_object_.pos.declination * 3600000.0f);
            pkt.magnitude_e2 = static_cast<int16_t>(selected_object_.brightness * 100.0f);
            const size_t n = std::min(selected_object_.name.size(), sizeof(pkt.name) - 1);
            std::memcpy(pkt.name, selected_object_.name.c_str(), n);
        } else {
            const std::string nm = "M" + std::to_string(selected_messier_id_);
            const size_t n = std::min(nm.size(), sizeof(pkt.name) - 1);
            std::memcpy(pkt.name, nm.c_str(), n);
        }
        raspi_.send_dso_target(pkt);
    }

    void transition(TelescopeState next) {
        current_state_ = next;
        StateSyncPayload payload{};
        payload.state = static_cast<uint8_t>(current_state_);
        payload.flags = 0;
        payload.sequence = state_sequence_++;
        raspi_.send_state_sync(payload);
    }

    void handle_init() {
        transition(TelescopeState::IDLE);
    }

    void handle_idle() {
        sample_search_events();
        update_fov_and_objects();

        if (search_requested_) {
            search_requested_ = false;
            cancel_requested_ = false;
            if (!try_select_target_from_current_fov(selected_messier_id_)) {
                lookup_target_globally(selected_messier_id_);
            }
            send_dso_target_packet();
            target_full_sent_ = has_selected_object_;
            transition(TelescopeState::SEARCH);
        }
    }

    void handle_search() {
        sample_search_events();
        update_fov_and_objects();

        if (cancel_requested_) {
            cancel_requested_ = false;
            search_requested_ = false;
            has_selected_object_ = false;
            selected_messier_id_ = -1;
            transition(TelescopeState::IDLE);
            return;
        }

        if (!has_selected_object_ && selected_messier_id_ >= 0) {
            bool found = try_select_target_from_current_fov(selected_messier_id_);
            if (found && !target_full_sent_) {
                send_dso_target_packet();
                target_full_sent_ = true;
            }
        }

        bool in_fov = has_selected_object_ &&
            astronomy_.is_object_in_FOV(selected_object_, astronomy_.get_current_fov());

        if (in_fov) {
            transition(TelescopeState::FOUND);
        }
    }

    void handle_found() {
        sample_search_events();
        update_fov_and_objects();

        if (cancel_requested_) {
            cancel_requested_ = false;
            search_requested_ = false;
            has_selected_object_ = false;
            selected_messier_id_ = -1;
            transition(TelescopeState::IDLE);
            return;
        }

        bool in_fov = has_selected_object_ &&
            astronomy_.is_object_in_FOV(selected_object_, astronomy_.get_current_fov());

        if (!in_fov) {
            transition(TelescopeState::SEARCH);
        }
    }

    Touchscreen& touchscreen_;
    SDCard& sdcard_;
    RasPi& raspi_;

    Telescope telescope_cfg_;
    Astronomy astronomy_;

    TelescopeState current_state_{TelescopeState::INIT};

    bool last_search_status_{false};
    bool search_requested_{false};
    bool cancel_requested_{false};
    int selected_messier_id_{-1};

    uint16_t state_sequence_{0};
    int last_search_result_{0};
    bool has_selected_object_{false};
    bool target_full_sent_{false};
    DSO selected_object_{};
};

} // namespace telescope
