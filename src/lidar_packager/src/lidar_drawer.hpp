#include <cairo/cairo.h>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <string>
#include "lidarpoint.hpp"
#include "rclcpp/rclcpp.hpp"

void drawPointCloud(LidarPoint* lidarPoints, int pointCount, rclcpp::Logger logger)
{
    static int i = 0;

    cairo_surface_t* surface;
    cairo_t* cr;

    surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 800, 800);
    cr = cairo_create(surface);

    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    cairo_translate(cr, 250, 250);

    for(int i = 0; i < pointCount; i++)
    {
        cairo_arc(cr, lidarPoints[i].x * 100, lidarPoints[i].y * 100, 2, 0, 2*M_PI);
        cairo_stroke(cr);
    }

    std::string fileName = "imgs/lidar-data-" + std::to_string(i) + ".png";

    cairo_surface_write_to_png(surface, fileName.c_str());
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    RCLCPP_INFO(logger, "Wrote image %i to src folder", i);

    i++;
}
