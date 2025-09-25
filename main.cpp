#include "libraries/geometry.h"
#include <fstream>
#include <iostream>
using namespace std;

struct Material {
    Material(const Vec3f &color) : diffuse_color(color) {}
    Material() : diffuse_color() {}
    Vec3f diffuse_color;
};

const Material ivory(Vec3f(0.4, 0.4, 0.3));
const Material plastic = Vec3f(0.3, 0.1, 0.1);

const Vec3f Background_Color = Vec3f(0.0, 0.0, 0.0);
const int FOV = 1.05; // 60 Deg FOV

struct Sphere {
    Vec3f center;
    float radius;
    Material material;
    Sphere(const Vec3f &c, const float &r, const Material &m) : center(c), radius(r), material(m) {}

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

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material) {
    float dist_i;
    float spheres_dist = numeric_limits<float>::max();

    for (size_t i=0; i < spheres.size(); i++) {
        // Account for overlapping Sphere
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }

    // Set to background color if too far
    return spheres_dist<1000;
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const vector<Sphere> &spheres) {
    Vec3f point, N;
    Material material;

    if (!scene_intersect(orig, dir, spheres, point, N, material)) return Background_Color;
    return material.diffuse_color;
}

void render(const std::vector<Sphere> &spheres) {
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

            framebuffer[i + j*width] = cast_ray(Vec3f(0,0,0), dir, spheres);
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
    vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(-3,0,-16), 2.0f, plastic));
    spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2.0f, ivory));
    spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 2.0f, ivory));
    spheres.push_back(Sphere(Vec3f(7.0, 5.0, -18.0), 2.0f, plastic));

    render(spheres);
    return 0;
}