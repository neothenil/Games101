//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        if (u < 0.f) u = 0;
        if (u > 1.f) u = 1;
        if (v < 0.f) v = 0;
        if (v > 1.f) v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    float h(float u, float v)
    {
        Eigen::Vector3f color = getColor(u, v);
        float result = color[0]*0.299+color[1]*0.587+color[2]*0.114;
        return result;
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        if (u < 0.f) u = 0;
        if (u > 1.f) u = 1;
        if (v < 0.f) v = 0;
        if (v > 1.f) v = 1;
        float u1 = int(u * width) / float(width);
        float u2 = (int(u * width) + 1) / float(width);
        float v1 = int(v * height) / float(height);
        float v2 = (int(v * height) + 1) / float(height);
        Eigen::Vector3f c11, c12, c21, c22, c1, c2, c;
        c11 = getColor(u1, v1);
        c12 = getColor(u1, v2);
        c21 = getColor(u2, v1);
        c22 = getColor(u2, v2);
        c1 = (u - u1) / (u2 - u1) * (c21 - c11) + c11;
        c2 = (u - u1) / (u2 - u1) * (c22 - c12) + c12;
        c = (v - v1) / (v2 - v1) * (c2 - c1) + c1;
        return c;
    }
};
#endif //RASTERIZER_TEXTURE_H
