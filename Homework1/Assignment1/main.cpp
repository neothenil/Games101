#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

inline double deg2rad(double degree)
{
    return degree / double(180) * MY_PI;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rad = deg2rad(rotation_angle);
    Eigen::Matrix4f rotatez;
    rotatez << std::cos(rad), -std::sin(rad), 0, 0,
               std::sin(rad), std::cos(rad),  0, 0,
               0,             0,              1, 0,
               0,             0,              0, 1;
    model = rotatez * model;
    return model;
}

Eigen::Matrix4f get_model_matrix(Eigen::Vector3f n, float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rad = deg2rad(rotation_angle);
    n.normalized();
    Eigen::Matrix3f rotate, N;
    N << 0, -n[2], n[1],
         n[2], 0, -n[0],
         -n[1], n[0], 0;
    rotate = cos(rad)*Eigen::Matrix3f::Identity() + (1.0 - cos(rad))*n*n.transpose()
             + sin(rad)*N;
    Eigen::Matrix4f rotate_homo;
    rotate_homo.row(0) = Eigen::Vector4f(rotate.coeff(0, 0), rotate.coeff(0, 1), rotate.coeff(0, 2), 0.0);
    rotate_homo.row(1) = Eigen::Vector4f(rotate.coeff(1, 0), rotate.coeff(1, 1), rotate.coeff(1, 2), 0.0);
    rotate_homo.row(2) = Eigen::Vector4f(rotate.coeff(2, 0), rotate.coeff(2, 1), rotate.coeff(2, 2), 0.0);
    rotate_homo.row(3) = Eigen::Vector4f(0.0, 0.0, 0.0, 1.0);
    model = rotate_homo * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float l, r, b, t, n, f;
    n = -zNear;  // Why is zNear and zFar > 0?
    f = -zFar;
    t = fabs(n) * tan(eye_fov / 2.0f);
    r = aspect_ratio * t;
    b = -t;
    l = -r;
    Eigen::Matrix4f ortho_project, ortho_translate, ortho_scale;
    ortho_translate << 1, 0, 0, -(r+l)/2.0,
                       0, 1, 0, -(t+b)/2.0,
                       0, 0, 1, -(n+f)/2.0,
                       0, 0, 0, 1;
    ortho_scale << 2.0/(r-l), 0,         0,         0,
                   0,         2.0/(t-b), 0,         0,
                   0,         0,         2.0/(n-f), 0,
                   0,         0,         0,         1;
    ortho_project = ortho_scale * ortho_translate;

    Eigen::Matrix4f persp_project;
    persp_project << n, 0, 0,   0,
                     0, n, 0,   0,
                     0, 0, n+f, -n*f,
                     0, 0, 1,   0;
    
    projection = ortho_project * persp_project;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(Eigen::Vector3f(0.0, 1.0, 0.0), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
