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
            if (p <= emit_area_sum){
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
    Intersection inter1 = intersect(ray);
    if (!inter1.happened)
        return Vector3f(0.0);

    if (inter1.obj->hasEmit())
    {
        if (depth == 0)
            return inter1.m->getEmission();
        else
            return Vector3f(0.0);
    }

    Vector3f emission(0.0);

    Intersection light;
    float pdfLight;
    sampleLight(light, pdfLight);
	
    Vector3f wo;
    wo = normalize(light.coords - inter1.coords);
    float dis;
    dis = (light.coords - inter1.coords).norm();
    float dissqure = dis * dis;
    Intersection interLightDir = intersect(Ray(inter1.coords, wo));

    if (interLightDir.happened && (interLightDir.coords-light.coords).norm()<1e-2)
    {
        emission = light.emit * inter1.m->eval(-ray.direction, wo, inter1.normal)
            * dotProduct(wo, inter1.normal) * dotProduct(-wo, light.normal)
            / (dissqure * pdfLight);
    }

    
    
    if (get_random_float() <= RussianRoulette)
    {
		wo = inter1.m->sample(ray.direction, inter1.normal).normalized();
        Ray bouns(inter1.coords, wo);
        Vector3f indir = castRay(bouns, depth + 1);
        emission += indir * inter1.m->eval(-ray.direction, wo, inter1.normal)
           * dotProduct(wo, inter1.normal)
            / inter1.m->pdf(ray.direction, wo, inter1.normal)
            /RussianRoulette;
    }

    return emission;
}