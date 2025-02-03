#ifndef COLOR_SCALE_HPP
#define COLOR_SCALE_HPP

#include <opencv2/core.hpp>

cv::Vec3b get_color(float z, float mn=-5.f, float mx=15.f)
{
    // convert the z value in the range [mn, mx] to the rgb values for a rainbow color map
    /*float t = (z - mn) / (mx - mn);
    float r = std::max(0.f, 1.5f - std::abs(3.f - 4.f*t));
    float g = std::max(0.f, 1.5f - std::abs(2.f - 4.f*t));
    float b = std::max(0.f, 1.5f - std::abs(1.f - 4.f*t));

    return cv::Vec3b(255*r, 255*g, 255*b); */

    //convert hsv value to rgb
    float h = (z - mn) / (mx - mn) * 240.f;
    float s = 1.f;
    float v = 1.f;

    float c = v * s;
    float x = c * (1 - std::abs(std::fmod(h / 60.f, 2) - 1));
    float m = v - c;

    float r, g, b;

    if( h < 60 ) { r = c; g = x; b = 0; }
    else if( h < 120 ) { r = x; g = c; b = 0; }
    else if( h < 180 ) { r = 0; g = c; b = x; }
    else if( h < 240 ) { r = 0; g = x; b = c; }
    else if( h < 300 ) { r = x; g = 0; b = c; }
    else { r = c; g = 0; b = x; }

    return cv::Vec3b(255*(r+m), 255*(g+m), 255*(b+m));
}

#endif // COLOR_SCALE_HPP
