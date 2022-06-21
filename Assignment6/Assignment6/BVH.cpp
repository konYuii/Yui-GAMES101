#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    //const int n = objects.size();
    // Compute bounds of all primitives in BVH node
    Bounds3 bound;
    for (int i = 0; i < objects.size(); ++i)
        bound = Union(bound, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds .maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        Bounds3 bounds;
        svhBound.clear();
        svhBound.reserve(objects.size());
        for (int i = 0; i < objects.size() - 1; i++)
        {
            bounds = Union(bounds, objects[i]->getBounds());
            svhBound.push_back(bounds);

        }


        Bounds3 rightBound;
        
        double sur = bound.SurfaceArea();
        double cost = 2*objects.size();
        int index = objects.size() / 2;
        for (int i = objects.size() - 1; i > 0; i--)
        {
            rightBound = Union(rightBound, objects[i]->getBounds());
            float c = (rightBound.SurfaceArea() / sur * (objects.size() - i)) + (svhBound[i - 1].SurfaceArea() / sur * (i - 1));
            if (cost >= c)
            {
                cost = c;
                index = i;
            }
        }
        auto beginning = objects.begin();
        auto middling = objects.begin() + index;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection interL,interR;
    if (!node  ||  !(node->bounds.IntersectP(ray,ray.direction_inv)))
        return interL;
    if (node->left == nullptr && node->right == nullptr)
    {
        return node->object->getIntersection(ray);
    }
    if (node->left)
        interL = getIntersection(node->left, ray);
    if (node->right)
        interR = getIntersection(node->right, ray);
    if (interL.obj == nullptr)
        return interR;
    if (interR.obj == nullptr)
        return interL;

    return interL.distance < interR.distance ? interL : interR;

}