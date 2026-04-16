#pragma once

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
        SETUP,
        READY,
        SEARCH,
        FOUND
    };

    struct Event {
        bool configuration_selected{false};
        bool object_selected{false};
        bool object_found{false};
        bool request_cancelled{false};
        bool display_objects{true};
    };

    StateMachine(Touchscreen& touchscreen,
                 SDCard::SDCard& sdcard)
        : touchscreen_(touchscreen),
          sdcard_(sdcard),
          telescope_cfg_{},
          astronomy_(telescope_cfg_, sdcard_) {}

    void init() {
        current_state_ = TelescopeState::INIT;
        events_ = Event{};
        selected_messier_id_ = -1;
        last_search_status_ = false;
        has_selected_object_ = false;
        selected_object_ = DSO{};
    }

    void update_sensors(float altitude_deg, float azimuth_deg,
                        float latitude, float longitude,
                        const UTC& time) {
        astronomy_.update_pose(altitude_deg, azimuth_deg,
                               latitude, longitude, time);
    }

    void tick() {
        clear_edge_events();
        sample_ui_state();

        switch (current_state_) {
        case TelescopeState::INIT:
            handle_init();
            break;
        case TelescopeState::SETUP:
            handle_setup();
            break;
        case TelescopeState::READY:
            handle_ready();
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
    const Event& events() const { return events_; }
    int selected_messier_id() const { return selected_messier_id_; }

private:
    void clear_edge_events() {
        const bool display = events_.display_objects;
        events_ = Event{};
        events_.display_objects = display;
    }

    void sample_ui_state() {
        events_.display_objects = touchscreen_.get_view_status();

        const bool search_now = touchscreen_.get_search_status();

        if (!last_search_status_ && search_now) {
            events_.object_selected = true;
            selected_messier_id_ = touchscreen_.get_selected_messier_id();
        }

        if (last_search_status_ && !search_now &&
            (current_state_ == TelescopeState::SEARCH || current_state_ == TelescopeState::FOUND)) {
            events_.request_cancelled = true;
        }

        if (touchscreen_.get_main()) {
            events_.configuration_selected = true;
        }

        last_search_status_ = search_now;
    }

    void update_fov_and_objects() {
        astronomy_.convert_hc_to_eqc();
        astronomy_.calculate_FOV();
        astronomy_.find_objects_within_FOV();
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

    void send_state_debug() {
        DebugPayload dbg{};
        const char* label = nullptr;

        switch (current_state_) {
        case TelescopeState::INIT:   label = "INIT"; break;
        case TelescopeState::SETUP:  label = "SETUP"; break;
        case TelescopeState::READY:  label = "READY"; break;
        case TelescopeState::SEARCH: label = "SEARCH"; break;
        case TelescopeState::FOUND:  label = "FOUND"; break;
        }

        for (int i = 0; i < 6 && label[i] != '\0'; ++i) {
            dbg.data[i] = static_cast<uint8_t>(label[i]);
        }
        raspi::send_debug(dbg);
    }

    void handle_init() {
        current_state_ = TelescopeState::SETUP;
        send_state_debug();
    }

    void handle_setup() {
        update_fov_and_objects();

        if (events_.configuration_selected) {
            current_state_ = TelescopeState::READY;
            send_state_debug();
        }
    }

    void handle_ready() {
        update_fov_and_objects();

        if (events_.object_selected) {
            try_select_target_from_current_fov(selected_messier_id_);
            current_state_ = TelescopeState::SEARCH;
            send_state_debug();
        }
    }

    void handle_search() {
        update_fov_and_objects();

        if (events_.request_cancelled) {
            has_selected_object_ = false;
            selected_messier_id_ = -1;
            current_state_ = TelescopeState::READY;
            send_state_debug();
            return;
        }

        if (!has_selected_object_ && selected_messier_id_ >= 0) {
            try_select_target_from_current_fov(selected_messier_id_);
        }

        if (has_selected_object_) {
            events_.object_found =
                astronomy_.is_object_in_FOV(selected_object_, astronomy_.get_current_fov());
        } else {
            events_.object_found = false;
        }

        if (events_.object_found) {
            current_state_ = TelescopeState::FOUND;
            send_state_debug();
        }
    }

    void handle_found() {
        update_fov_and_objects();

        if (events_.request_cancelled) {
            has_selected_object_ = false;
            selected_messier_id_ = -1;
            current_state_ = TelescopeState::READY;
            send_state_debug();
            return;
        }

        if (has_selected_object_) {
            events_.object_found =
                astronomy_.is_object_in_FOV(selected_object_, astronomy_.get_current_fov());
        } else {
            events_.object_found = false;
        }

        if (!events_.object_found) {
            current_state_ = TelescopeState::SEARCH;
            send_state_debug();
        }
    }

    Touchscreen& touchscreen_;
    SDCard::SDCard& sdcard_;

    Telescope telescope_cfg_;
    Astronomy astronomy_;

    TelescopeState current_state_{TelescopeState::INIT};
    Event events_{};

    bool last_search_status_{false};
    int selected_messier_id_{-1};

    bool has_selected_object_{false};
    DSO selected_object_{};
};

} // namespace telescope
