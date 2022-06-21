#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;


        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    //递归实现
    
    /*if (control_points.size() == 2)
    {
        return  control_points[0] + (control_points[1] - control_points[0]) * t;
    }
    
    std::vector<cv::Point2f> nextLevel;
    for (int i = 0; i < control_points.size() - 1; i++)
    {
        nextLevel.push_back(control_points[i] + (control_points[i + 1] - control_points[i]) * t);
    }
    return recursive_bezier(nextLevel, t);*/

    int level = control_points.size() - 1;
    int ck=1;
    cv::Point2f p(0,0);
    for (int i = 0; i <= level; i++)
    {
        p += (pow(t, i) * control_points[i] * pow(1 - t, level - i) * ck);
        ck *= (level - i);
        ck /= (i + 1);
    }
    return p;
}

void Alligze(cv::Mat& window, cv::Point2f p)
{
	float x = floor(p.x) + 0.5;
	float y = ceil(p.y) + 0.5;
    window.at<cv::Vec3b>(p.y, p.x)[2] = 255 * (1 - abs(p.x - x) - abs(p.y - y));
    if (p.x == x && p.y == y)
        return;

	float x1 = x + 1;
	float y1 = y + 1;



    if (p.x > x)
        window.at<cv::Vec3b>(p.y, p.x + 1)[2] = 255 * abs(x1 - p.x);
    else
        window.at<cv::Vec3b>(p.y, p.x - 1)[2] = 255 * abs(x - p.x);
    if (p.y > y)
        window.at<cv::Vec3b>(p.y + 1, p.x)[2] = 255 * abs(y1 - p.x);
    else
        window.at<cv::Vec3b>(p.y - 1, p.x)[2] = 255 * abs(y - p.x);

    if (p.x >= x && p.y >= y)
        window.at<cv::Vec3b>(p.y + 1, p.x + 1)[2] = window.at<cv::Vec3b>(p.y, p.x + 1)[2] * 0.5f + window.at<cv::Vec3b>(p.y + 1, p.x)[2] * 0.5f;
    else if(p.x < x)
        window.at<cv::Vec3b>(p.y + 1, p.x - 1)[2] = window.at<cv::Vec3b>(p.y, p.x - 1)[2] * 0.5f + window.at<cv::Vec3b>(p.y + 1, p.x)[2] * 0.5f;
    else if (p.y < y)
        window.at<cv::Vec3b>(p.y - 1, p.x + 1)[2] = window.at<cv::Vec3b>(p.y, p.x + 1)[2] * 0.5f + window.at<cv::Vec3b>(p.y - 1, p.x)[2] * 0.5f;
    else
        window.at<cv::Vec3b>(p.y - 1, p.x - 1)[2] = window.at<cv::Vec3b>(p.y, p.x - 1)[2] * 0.5f + window.at<cv::Vec3b>(p.y - 1, p.x)[2] * 0.5f;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    for (double t = 0; t <= 1; t += 0.001)
    {
        cv::Point2f p = recursive_bezier(control_points, t);


        //window.at<cv::Vec3b>(p.y, p.x)[2] = 255;
        //抗锯齿，感觉写的不好，有改进空间
        Alligze(window, p);
    }



}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve2.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
