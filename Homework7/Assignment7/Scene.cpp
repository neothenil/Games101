//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }
    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);
    if (!inter.happened)
        return Vector3f(0.f);
    return shade(inter, ray.direction);
}

Vector3f Scene::shade(Intersection &inter, Vector3f wo) const
{
    // Contribution if the object is light source.
    Vector3f emissionLight(0.f);
    if (inter.m->hasEmission()) {
        emissionLight = inter.m->getEmission();
    }

    // Contribution from the light source.
    Vector3f directLight(0.f);
    float lightPdf;
    Intersection lightPos;
    sampleLight(lightPos, lightPdf);
    Vector3f rayOrig = (dotProduct(inter.normal, lightPos.coords - inter.coords) < 0) ?
               inter.coords - inter.normal * EPSILON :
               inter.coords + inter.normal * EPSILON;
    Vector3f xx = lightPos.coords - rayOrig;
    Vector3f wi = normalize(xx);
    Ray xxRay(rayOrig, wi);
    Intersection xxInter = intersect(xxRay);
    // If the ray is not blocked in the middle.
    if (xxInter.happened && (xxInter.coords - lightPos.coords).norm() <= EPSILON * 100.f) {
        Vector3f fr = inter.m->eval(-wi, -wo, inter.normal);
        if (dotProduct(lightPos.normal, -wi) > 0.f) {
            directLight = lightPos.emit * fr * fabs(dotProduct(inter.normal, wi)) *
                          dotProduct(lightPos.normal, -wi) / dotProduct(xx, xx) / lightPdf;
        }
    }

    // Contribution from other reflectors.
    Vector3f indirectLight(0.f);
    float ksi = get_random_float();
    if (ksi > RussianRoulette)
        return emissionLight + directLight;
    
    wi = inter.m->sample(wo, inter.normal);
    rayOrig = ((dotProduct(inter.normal, wi) < 0) ?
              inter.coords - inter.normal * EPSILON :
              inter.coords + inter.normal * EPSILON);
    Ray indirectRay(rayOrig, wi);
    Intersection reflectorPos = intersect(indirectRay);
    if (reflectorPos.happened && !reflectorPos.m->hasEmission()) {
        Vector3f fr = inter.m->eval(-wi, -wo, inter.normal);
        float pdf = inter.m->pdf(wo, wi, inter.normal);
        Vector3f reflected = shade(reflectorPos, wi);
        if (pdf > 0.f) {
            indirectLight = reflected * fr * fabs(dotProduct(inter.normal, wi)) /
                            pdf / RussianRoulette;
        }
    }

    return emissionLight + directLight + indirectLight;
}