#include <fstream>
#include <iostream>
#include <map>
#include <algorithm>
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <geometry.h>
#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include "imgui/backends/imgui_impl_sdl3.h"
#include "main_struct.h"
// g++ -Iinclude -Iimgui -Iimgui/backends -Iinclude/SDL3 main.cpp imgui/imgui.cpp imgui/imgui_draw.cpp imgui/imgui_tables.cpp imgui/imgui_widgets.cpp imgui/backends/imgui_impl_sdl3.cpp imgui/backends/imgui_impl_opengl3.cpp -Llib -lSDL3 -lmingw32 -lopengl32 -lgdi32 -o main.exe
using namespace std;

int bg_width, bg_height, bg_channels;
const float PI = 3.14159265358979323846;
const int frame_width = 1920;
const int frame_height = 1080;

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
    const float inv_pi = 1/PI;
    const float inv_maxcol = 1/255.0f;

    // Compute background texture & pixel coords
    if (depth > 4 || !scene_intersect(orig, dir, scene, point, N, material)) {
        float u = 0.5f + atan2f(dir.z, dir.x) * inv_pi * 0.5f;
        float v = 0.5f - asinf(dir.y) * inv_pi;

        int px = min(bg_width - 1, max(0, int(u * bg_width)));
        int py = min(bg_height - 1, max(0, int(v * bg_height)));

        int index = (py * bg_width + px) * 3;
        float r = scene.bg_data[index] * inv_maxcol;
        float g = scene.bg_data[index + 1] * inv_maxcol;
        float b = scene.bg_data[index + 2] * inv_maxcol;
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
        Vec3f to_light = scene.lights[i].position - point;
        float light_distance = to_light.norm();
        Vec3f light_dir = to_light * (1 / light_distance);
        
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

vector<unsigned char> render(const Scene &scene) {
    const int width = frame_width;
    const int height = frame_height;
    const float scale = tan(scene.FOV/2.0f);
    const float scale_aspect_prod = scale * frame_width / float(frame_height);
    vector<unsigned char> framebuffer(width * height * 3);

    const float inv_w = (1.0/width);
    const float inv_h = (1.0/height);
    
    // Multi-threaded rendering
    #pragma omp parallel for
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            // Calculate field of view
            float x =  (2*(i + 0.5) * inv_w - 1) * scale_aspect_prod;
            float y = -(2*(j + 0.5) * inv_h - 1) * scale;
            Vec3f dir = Vec3f(x, y, -1).normalize();

            // Create bytearray
            Vec3f c = cast_ray(Vec3f(0,0,0), dir, scene);

            float maxVal = max(c[0], max(c[1], c[2]));
            if (maxVal > 1.f) c = c * (1.f / maxVal);

            size_t idx = (i + j*width) * 3;
            framebuffer[idx+0] = static_cast<unsigned char>(clamp(c[0], 0.f, 1.f) * 255.f);
            framebuffer[idx+1] = static_cast<unsigned char>(clamp(c[1], 0.f, 1.f) * 255.f);
            framebuffer[idx+2] = static_cast<unsigned char>(clamp(c[2], 0.f, 1.f) * 255.f);
        }
    }

    return framebuffer;
}

int main() {
    // Initialize
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetSwapInterval(1);
    SDL_Window* window = SDL_CreateWindow("GUI", frame_width, frame_height, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");
    
    // Materials Shapes Lights Backgrounds
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

    // Framebuffer
    vector<unsigned char> framebuffer;
    framebuffer = render(scene);

    // Dynamic Rendering
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame_width, frame_height, 0, GL_RGB, GL_UNSIGNED_BYTE, framebuffer.data());

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    bool done = false;
    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) {
                done = true;
            }
            else if (event.type == SDL_EVENT_WINDOW_DESTROYED) {
                if (event.window.windowID == SDL_GetWindowID(window)) {
                    done = true;
                }
            }
        }
        framebuffer = render(scene);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, frame_width, frame_height, GL_RGB, GL_UNSIGNED_BYTE, framebuffer.data());

        // Start a new ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Hello");
        ImGui::GetBackgroundDrawList()->AddImage(textureID, ImVec2(0, 0), ImGui::GetIO().DisplaySize);
        ImGui::SliderFloat("Sphere Radius", &scene.spheres[3].radius, 0.1f, 10.0f);
        ImGui::End();

        ImGui::Render();
        int display_w, display_h;
        SDL_GetWindowSizeInPixels(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}