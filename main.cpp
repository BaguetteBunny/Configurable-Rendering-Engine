#include "libraries/geometry.h"
#include <fstream>
#include <iostream>
using namespace std;

void render() {
    const int width = 1024;
    const int height = 768;
    
    // Initialize buffer frame
    vector<Vec3f> framebuffer(width * height);

    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            framebuffer[i + j*width] = Vec3f(j/float(height), i/float(width), 0);
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
    render();
    return 0;
}