#pragma once
#define STB_IMAGE_IMPLEMENTATION
#include <geometry.h>
#include <algorithm>
#include <map>
#include <stb_image.h>

using namespace std;

struct Material {
    Material(const float &r, const Vec4f &a, const Vec3f &color, const float &spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
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

struct Scene {
    Scene(const vector<Sphere> &s, const vector<Light> &l, const map<string, Material> &m, const float &f):
    spheres(s), lights(l), materials(m), FOV(f) {}

    vector<Sphere> spheres;
    vector<Light> lights;
    map<string, Material> materials;
    float FOV;
    unsigned char* bg_data = nullptr;

    ~Scene() {if (bg_data) stbi_image_free(bg_data);}
};
