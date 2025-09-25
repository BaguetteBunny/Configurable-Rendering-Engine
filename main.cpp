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
}

int main() {
    render();
    return 0;
}