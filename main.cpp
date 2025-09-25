#include "libraries/geometry.h"
#include <fstream>
#include <iostream>
using namespace std;

const Vec3f Background_Color = Vec3f(0.0, 0.0, 0.0);
const Vec3f Object_Color = Vec3f(0.4, 0.4, 0.3);
const int FOV = 1.05; // 60 Deg FOV

struct Sphere {
    Vec3f center;
    float radius;
    Sphere(const Vec3f &c, const float &r) : center(c), radius(r) {}

    bool ray_intersect(const Vec3f &origin, const Vec3f &direction, float &t0) const {
        Vec3f L = center - origin;
        float projection = L * direction;
        float d2 = L*L - projection*projection;
        float r2 = radius*radius;
        
        // Ray misses sphere
        if (d2 > r2) return false;

        float half_chord = sqrtf(r2 - d2);
        t0 = projection - half_chord;
        float t1 = projection + half_chord;

        // Check distance along ray
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;

        return true;
    }
};

Vec3f cast_ray(const Vec3f &origin, const Vec3f &direction, const Sphere &sphere) {
    float sphere_dist = numeric_limits<float>::max();
    if (!sphere.ray_intersect(origin, direction, sphere_dist)) return Background_Color;
    return Object_Color;
}

void render(const Sphere &sphere) {
    const int width = 1024;
    const int height = 768;
    
    // Initialize buffer frame
    vector<Vec3f> framebuffer(width * height);

    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            // Calculate field of view
            float x =  (2*(i + 0.5)/(float)width - 1) * tan(FOV/2.)*width/(float)height;
            float y = -(2*(j + 0.5)/(float)height - 1) * tan(FOV/2.);
            Vec3f dir = Vec3f(x, y, -1).normalize();

            framebuffer[i + j*width] = cast_ray(Vec3f(0,0,0), dir, sphere);
        }
    }

    // Save buffer frame
    ofstream ofs;
    ofs.open("./output/out.ppm");
    ofs << "P6\n" << width << " " << height << "\n255\n";

    for (size_t i = 0; i < height*width; ++i) {
        for (size_t j = 0; j < 3; j++) {
            float clamped_color = max(0.f, min(1.f, framebuffer[i][j]));
            unsigned char final_rgb_color = 255 * clamped_color;

            ofs << final_rgb_color;
        }
    }

    ofs.close();

}

int main() {
    Sphere sphere(Vec3f(-3,0,-16), 2.0f);
    render(sphere);
    return 0;
}