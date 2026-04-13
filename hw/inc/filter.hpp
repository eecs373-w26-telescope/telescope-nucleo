#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace filter {

template <typename T, std::size_t N>
class Filter {
public:
    Filter(T full_circle)
        : full_circle(full_circle) {}

    T update(T value) {
        constexpr float TWO_PI = 6.28318530717958647692f;

        float rad = (static_cast<float>(value) / static_cast<float>(full_circle)) * TWO_PI;
        float s = sinf(rad);
        float c = cosf(rad);

        sin_sum -= sin_buf[idx];
        cos_sum -= cos_buf[idx];
        sin_buf[idx] = s;
        cos_buf[idx] = c;
        sin_sum += s;
        cos_sum += c;

        idx = (idx + 1) % N;

        float avg_rad = atan2f(sin_sum, cos_sum);
        if (avg_rad < 0.0f) avg_rad += TWO_PI;

        return static_cast<T>((avg_rad / TWO_PI) * static_cast<float>(full_circle));
    }

private:
    T full_circle;
    std::array<float, N> sin_buf{};
    std::array<float, N> cos_buf{};
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    std::size_t idx = 0;
};

} // namespace filter
