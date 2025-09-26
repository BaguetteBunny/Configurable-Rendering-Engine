#define STB_IMAGE_IMPLEMENTATION
#include <fstream>
#include <iostream>
#include <map>
#include "include/geometry.h"
#include "include/stb_image.h"

using namespace std;

int bg_width, bg_height, bg_channels;
const float PI = 3.14159265358979323846;

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

// Direction vector --- See Phong's algorithm
Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);
}

// Refraction formula using Snell's law --- https://en.wikipedia.org/wiki/Snell%27s_law
Vec3f refract(const Vec3f &I, const Vec3f &N, const float &refractive_index) {
    float cosi = - max(-1.f, min(1.f, I*N));
    float etai = 1, etat = refractive_index;
    Vec3f n = N;

    // Swap indices if ray in object
    if (cosi < 0) {
        cosi = -cosi;
        swap(etai, etat); n = -N;
    }

    float eta = etai / etat;
    float k = 1 - eta*eta*(1 - cosi*cosi);
    return k < 0 ? Vec3f(0,0,0) : I*eta + n*(eta * cosi - sqrtf(k));
}

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const Scene &scene, Vec3f &hit, Vec3f &N, Material &material) {
    float dist_i;
    float spheres_dist = numeric_limits<float>::max();

    for (size_t i=0; i < scene.spheres.size(); i++) {
        // Account for overlapping Sphere
        if (scene.spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - scene.spheres[i].center).normalize();
            material = scene.spheres[i].material;
        }
    }

    // Set to background color if too far
    return spheres_dist<1000;
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const Scene &scene, size_t depth=0) {
    Vec3f point, N;
    Material material;

    // Compute background texture & pixel coords
    if (depth > 4 || !scene_intersect(orig, dir, scene, point, N, material)) {
        float u = 0.5f + atan2f(dir.z, dir.x) / (2 * PI);
        float v = 0.5f - asinf(dir.y) / PI;

        int px = min(bg_width - 1, max(0, int(u * bg_width)));
        int py = min(bg_height - 1, max(0, int(v * bg_height)));

        int index = (py * bg_width + px) * 3;
        float r = scene.bg_data[index] / 255.0f;
        float g = scene.bg_data[index + 1] / 255.0f;
        float b = scene.bg_data[index + 2] / 255.0f;
        return Vec3f(r, g, b);
    };

    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();

    Vec3f reflect_orig = reflect_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // Offset for no self-occlusion
    Vec3f refract_orig = refract_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;

    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, scene, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, scene, depth + 1);

    float diffuse_light_intensity = 0;
    float specular_light_intensity = 0;
    for (size_t i=0; i<scene.lights.size(); i++) {
        Vec3f light_dir = (scene.lights[i].position - point).normalize();
        float light_distance = (scene.lights[i].position - point).norm();
        
        // Check if point in shadow of lights[i]
        Vec3f shadow_orig = light_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_dir, scene, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity  += scene.lights[i].intensity * max(0.f, light_dir*N);
        specular_light_intensity += powf(max(0.f, reflect(light_dir, N)*dir), material.specular_exponent)*scene.lights[i].intensity;
    }

    // See https://en.wikipedia.org/wiki/Phong_reflection_model#Concepts
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2] + refract_color*material.albedo[3];
}

void render(const Scene &scene) {
    const int width = 1024;
    const int height = 768;
    
    // Initialize buffer frame
    vector<Vec3f> framebuffer(width * height);

    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            // Calculate field of view
            float x =  (2*(i + 0.5)/(float)width - 1) * tan(scene.FOV/2.)*width/(float)height;
            float y = -(2*(j + 0.5)/(float)height - 1) * tan(scene.FOV/2.);
            Vec3f dir = Vec3f(x, y, -1).normalize();

            framebuffer[i + j*width] = cast_ray(Vec3f(0,0,0), dir, scene);
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
    map<string, Material> materials;
    materials["ivory"] = Material(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3), 50.0);
    materials["plastic"] = Material(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.0);
    materials["mirror"] = Material(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.0);
    materials["glass"] = Material(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125.0);

    vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(-3,0,-16), 2.0f, materials["plastic"]));
    spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2.0f, materials["glass"]));
    spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 2.0f, materials["ivory"]));
    spheres.push_back(Sphere(Vec3f(7.0, 5.0, -18.0), 4.0f, materials["mirror"]));

    vector<Light> lights;
    lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
    lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f(30, 20, 30), 1.7));

    Scene scene(spheres, lights, materials, 1.05); // 60 Deg FOV (Default)
    scene.bg_data = stbi_load("assets/church_of_lutherstadt.jpg", &bg_width, &bg_height, &bg_channels, 3);

    render(scene);
    return 0;
}