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
        u = std::clamp(u, 0.00001f, 0.9999f);
        v = std::clamp(v, 0.00001f, 0.9999f);
        int u_img = u * width;
        int v_img = (1 - v) * height;

        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u = std::clamp(u, 0.00001f, 0.9999f);
        v = std::clamp(v, 0.00001f, 0.9999f);

        int uMin, vMin, uMax, vMax;
        float u_img = u * width;
        float v_img = (1 - v) * height;
        int ujud = floor(u_img),vjud=floor(v_img);
        if ((u_img - ujud) <= 0.5f)
        {
            uMin = ujud - 1;
            uMax = ceil(u_img);
        }
        else
        {
            uMin = ujud;
            uMax = ceil(u_img) + 1;
        }
        if ((v_img - vjud) <= 0.5f)
        {
            vMin = vjud - 1;
            vMax = ceil(v_img);
        }
        else
        {
            vMin = vjud;
            vMax = ceil(v_img);
        }

        uMax = std::clamp(uMax, 0, width);
        uMin = std::clamp(uMin, 0, width);
        vMax = std::clamp(vMax, 0, height);
        vMin = std::clamp(vMin, 0, height);

        auto p1 = image_data.at<cv::Vec3b>(vMax, uMin);
        auto p2 = image_data.at<cv::Vec3b>(vMax, uMax);
        auto p3 = image_data.at<cv::Vec3b>(vMin, uMin);
        auto p4 = image_data.at<cv::Vec3b>(vMin, uMax);

        auto c1 = p1 + (u_img - uMin) / (uMax - uMin) * (p2 - p1);
        auto c2 = p3 + (u_img - uMin) / (uMax - uMin) * (p4 - p3);
        auto color = c2 + (c1 - c2) * (v_img - vMin) / (vMax - vMin);

        return Eigen::Vector3f(color[0], color[1], color[2]);

    }

};
#endif //RASTERIZER_TEXTURE_H
