#include <algorithm>
#include <cassert>
#include <chrono>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    // time_t start, stop;
    // time(&start);
    auto start = std::chrono::high_resolution_clock::now();
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    auto stop = std::chrono::high_resolution_clock::now();
    // time(&stop);
    // double diff = difftime(stop, start);
    long hrs = std::chrono::duration_cast<std::chrono::hours>(stop - start).count();
    long mins = std::chrono::duration_cast<std::chrono::minutes>(stop - start).count();
    long secs = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count();
    long msecs = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

    printf(
        "\rBVH Generation complete: \nTime Taken: %li hrs, %li mins, %li secs, %li msecs\n\n",
        hrs, mins, secs, msecs);
}

BVHBuildNode* recursiveBuildNaive(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildNaive(std::vector{objects[0]});
        node->right = recursiveBuildNaive(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
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

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildNaive(leftshapes);
        node->right = recursiveBuildNaive(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* recursiveBuildSAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode;

    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildSAH(std::vector{objects[0]});
        node->right = recursiveBuildSAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }

    int splitAxis = 0, splitPos = 0;
    double minCost = std::numeric_limits<double>::max();

    for (int axis = 0; axis < 3; ++axis) {
        std::sort(objects.begin(), objects.end(), [=](auto f1, auto f2) {
            return f1->getBounds().Centroid()[axis] <
                   f2->getBounds().Centroid()[axis];
        });
        std::vector<Bounds3> leftBounds(objects.size()), rightBounds(objects.size());
        for (int i = 1; i < objects.size(); ++i) {
            leftBounds[i] = Union(leftBounds[i-1], objects[i-1]->getBounds());
            rightBounds[i] = Union(rightBounds[i-1], objects[objects.size()-i]->getBounds());
        }
        for (int i = 1; i < objects.size(); ++i) {
            int leftN = i, rightN = objects.size()-i;
            double leftArea = leftBounds[leftN].SurfaceArea();
            double rightArea = rightBounds[rightN].SurfaceArea();
            double cost = leftArea * leftN + rightArea * rightN;
            if (cost < 0)
                continue;
            if (cost < minCost) {
                splitAxis = axis;
                splitPos = i;
                minCost = cost;
            }
        }
    }

    std::sort(objects.begin(), objects.end(), [=](auto f1, auto f2) {
        return f1->getBounds().Centroid()[splitAxis] <
                f2->getBounds().Centroid()[splitAxis];
    });

    auto beginning = objects.begin();
    auto middling = objects.begin() + splitPos;
    auto ending = objects.end();

    auto leftshapes = std::vector<Object*>(beginning, middling);
    auto rightshapes = std::vector<Object*>(middling, ending);

    assert(objects.size() == (leftshapes.size() + rightshapes.size()));

    node->splitAxis = splitAxis;
    node->left = recursiveBuildSAH(leftshapes);
    node->right = recursiveBuildSAH(rightshapes);
    node->bounds = Union(node->left->bounds, node->right->bounds);
    
    return node;
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    switch (splitMethod)
    {
    case SplitMethod::NAIVE:
        return recursiveBuildNaive(objects);
    case SplitMethod::SAH:
        return recursiveBuildSAH(objects);
    }
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
    Intersection isect;
    std::array<int, 3> dirIsNeg{int(ray.direction.x < 0), int(ray.direction.y < 0), int(ray.direction.z < 0)};
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
        return isect;
    }
    if (node->object) {
        return node->object->getIntersection(ray);
    }
    Intersection isect2;
    isect = getIntersection(node->left, ray);
    isect2 = getIntersection(node->right, ray);
    if (isect.happened && isect2.happened) {
        if (isect.distance < isect2.distance) {
            return isect;
        }
        else {
            return isect2;
        }
    }
    else if (isect.happened) {
        return isect;
    }
    else if (isect2.happened) {
        return isect2;
    }
    else {
        return isect;
    }
}