#include "libraries/geometry.h"
#include <fstream>
#include <iostream>
using namespace std;

struct Material {
    Material(const Vec3f &a, const Vec3f &color, const float &spec) : albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : albedo(1,0,0), diffuse_color(), specular_exponent() {}
    Vec3f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

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

struct Light {
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

Material ivory(Vec3f(0.6,  0.3, 0.1), Vec3f(0.4, 0.4, 0.3), 50.0);
Material plastic(Vec3f(0.9,  0.1, 0.0), Vec3f(0.3, 0.1, 0.1), 10.0);
Material mirror(Vec3f(0.0, 10.0, 0.8), Vec3f(1.0, 1.0, 1.0), 1425.0);

const Vec3f Background_Color = Vec3f(0.5, 0.5, 0.5);
const int FOV = 1.05; // 60 Deg FOV

Vec3f reflect(const Vec3f &I, const Vec3f &N) {return I - N*2.f*(I*N);} // Direction vector

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

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, size_t depth=0) {
    Vec3f point, N;
    Material material;

    // Reflection
    if (depth > 4 || !scene_intersect(orig, dir, spheres, point, N, material)) return Background_Color;
    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f reflect_orig = reflect_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // Offset for no self-occlusion
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0;
    float specular_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir = (lights[i].position - point).normalize();
        float light_distance = (lights[i].position - point).norm();
        
        // Check if point in shadow of lights[i]
        Vec3f shadow_orig = light_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
        specular_light_intensity += powf(max(0.f, reflect(light_dir, N)*dir), material.specular_exponent)*lights[i].intensity;
    }

    // See https://en.wikipedia.org/wiki/Phong_reflection_model#Concepts
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2];
}

void render(const std::vector<Sphere> &spheres, const vector<Light> &lights) {
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

            framebuffer[i + j*width] = cast_ray(Vec3f(0,0,0), dir, spheres, lights);
        }
    }

    // Save buffer frame
    ofstream ofs;
    ofs.open("./output/out.ppm", ofstream::out | ofstream::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";

    for (size_t i = 0; i < height*width; ++i) {
        Vec3f &c = framebuffer[i];
        float maximum = max(c[0], max(c[1], c[2]));
        if (maximum > 1) c = c*(1./maximum);

        for (size_t j = 0; j < 3; j++) {
            float clamped_color = max(0.f, min(1.f, c[j]));
            unsigned char final_rgb_color = 255 * clamped_color;

            ofs << final_rgb_color;
        }
    }
    ofs.close();
}

int main() {
    vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(-3,0,-16), 2.0f, plastic));
    spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2.0f, mirror));
    spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 2.0f, ivory));
    spheres.push_back(Sphere(Vec3f(7.0, 5.0, -18.0), 4.0f, mirror));

    vector<Light> lights;
    lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
    lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f(30, 20, 30), 1.7));

    render(spheres, lights);
    return 0;
}