//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <thread>
#include <mutex>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++){
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}

struct Payload
{
    std::vector<Vector3f> *framebuffer;
    const Scene *scene;
    Vector3f eye_pos;
    int spp;

    int offset;
    int length;

    int *progress;
    std::mutex *progressMutux;
};

static void renderPartial(Payload p)
{
    float scale = tan(deg2rad(p.scene->fov * 0.5));
    float imageAspectRatio = p.scene->width / (float)p.scene->height;
    
    for (int i = 0; i < p.length; ++i) {
        int index = p.offset + i;
        int iy = index / p.scene->width;
        int ix = index % p.scene->width;
        float x = (2 * (ix + 0.5) / (float)p.scene->width - 1) *
                    imageAspectRatio * scale;
        float y = (1 - 2 * (iy + 0.5) / (float)p.scene->height) * scale;

        Vector3f dir = normalize(Vector3f(-x, y, 1));
        for (int k = 0; k < p.spp; k++){
            (*p.framebuffer)[index] += p.scene->castRay(Ray(p.eye_pos, dir), 0) / p.spp;  
        }

        p.progressMutux->lock();
        (*p.progress)++;
        if (*p.progress % p.scene->width == 0) {
            UpdateProgress(*p.progress / (float)p.framebuffer->size());
        }
        p.progressMutux->unlock();
    }
}

void Renderer::MultithreadRender(const Scene& scene)
{
    int nthread = std::thread::hardware_concurrency() / 2;
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
    Vector3f eye_pos(278, 273, -800);

    if (framebuffer.size() <= nthread) {
        return Render(scene);
    }

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << " nthread: " << nthread << "\n";
    std::vector<std::thread> threads(nthread);
    int progress = 0;
    std::mutex progressMutex;
    int length = framebuffer.size() / nthread;
    for (int i = 0; i < nthread; ++i) {
        Payload p;
        p.framebuffer = &framebuffer;
        p.scene = &scene;
        p.eye_pos = eye_pos;
        p.spp = spp;
        p.offset = i * length;
        p.length = i == nthread - 1 ? framebuffer.size() - p.offset : length;
        p.progress = &progress;
        p.progressMutux = &progressMutex;
        threads[i] = std::thread(renderPartial, p);
    }
    for (auto& thread : threads) {
        thread.join();
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
