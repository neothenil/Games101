// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// z value in _v must be 0.
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f ab, bc, ca;
    Vector3f ap, bp, cp;
    Vector3f p(x, y, 0.0);
    ab = _v[1] - _v[0];
    bc = _v[2] - _v[1];
    ca = _v[0] - _v[2];
    ap = p - _v[0];
    bp = p - _v[1];
    cp = p - _v[2];
    Vector3f ab_x_ap, bc_x_bp, ca_x_cp;
    ab_x_ap = ab.cross(ap);
    bc_x_bp = bc.cross(bp);
    ca_x_cp = ca.cross(cp);
    if (ab_x_ap[2]*bc_x_bp[2] > 0 && bc_x_bp[2]*ca_x_cp[2] > 0)
        return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = -vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

// Screen space rasterization
// void rst::rasterizer::rasterize_triangle(const Triangle& t) {
//     auto v = t.toVector4();
    
//     // TODO : Find out the bounding box of current triangle.
//     Vector4f a(v[0]), b(v[1]), c(v[2]);
//     std::array<Vector3f, 3> screenPoints;
//     std::transform(std::begin(v), std::end(v), screenPoints.begin(), [](auto& vec) {
//         return Vector3f(vec.x(), vec.y(), 0.0f);
//     });
//     float xmin, ymin, xmax, ymax;
//     xmin = std::min({a[0], b[0], c[0]});
//     ymin = std::min({a[1], b[1], c[1]});
//     xmax = std::max({a[0], b[0], c[0]});
//     ymax = std::max({a[1], b[1], c[1]});
//     // iterate through the pixel and find if the current pixel is inside the triangle
//     for (int i = floor(xmin); i <= floor(xmax); i++) {
//         for (int j = floor(ymin); j <= floor(ymax); j++) {
//             float x, y;
//             x = i + 0.5f;
//             y = j + 0.5f;
//             if (!insideTriangle(x, y, screenPoints.data()))
//                 continue;
//             // If so, use the following code to get the interpolated z value.
//             auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
//             float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//             z_interpolated *= w_reciprocal;
//             // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
//             if (z_interpolated < depth_buf[get_index(i, j)]) {
//                 depth_buf[get_index(i, j)] = z_interpolated;
//                 set_pixel({float(i), float(j), 0.0}, t.getColor());
//             }
//         }
//     }
// }

static auto getSamplePoints(int i, int j) {
    std::array<Vector3f, 4> res = {
        Vector3f(float(i) + 0.25f, float(j) + 0.25f, 0.0),
        Vector3f(float(i) + 0.75f, float(j) + 0.25f, 0.0),
        Vector3f(float(i) + 0.25f, float(j) + 0.75f, 0.0),
        Vector3f(float(i) + 0.75f, float(j) + 0.75f, 0.0)
    };
    return res;
}

void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    Vector4f a(v[0]), b(v[1]), c(v[2]);
    std::array<Vector3f, 3> screenPoints;
    std::transform(std::begin(v), std::end(v), screenPoints.begin(), [](auto& vec) {
        return Vector3f(vec.x(), vec.y(), 0.0f);
    });
    float xmin, ymin, xmax, ymax;
    xmin = std::min({a[0], b[0], c[0]});
    ymin = std::min({a[1], b[1], c[1]});
    xmax = std::max({a[0], b[0], c[0]});
    ymax = std::max({a[1], b[1], c[1]});
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int i = floor(xmin); i <= floor(xmax); i++) {
        for (int j = floor(ymin); j <= floor(ymax); j++) {
            auto samplePoints = getSamplePoints(i, j);
            int subindex = 0;
            Vector2i subpixels[4] = {
                {2*i, 2*j}, {2*i+1, 2*j}, {2*i, 2*j+1}, {2*i+1, 2*j+1}
            };
            bool haveSetSubpixel = false;
            for (const auto& samplePoint : samplePoints) {
                float x = samplePoint.x();
                float y = samplePoint.y();
                int subi, subj;
                subi = subpixels[subindex][0];
                subj = subpixels[subindex][1];
                subindex += 1;
                if (!insideTriangle(x, y, screenPoints.data()))
                    continue;
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                if (z_interpolated < msaa_depth_buf[get_sub_index(subi, subj)]) {
                    msaa_depth_buf[get_sub_index(subi, subj)] = z_interpolated;
                    set_sub_pixel({float(subi), float(subj), 0.0}, t.getColor());
                    haveSetSubpixel = true;
                }
            }
            if (haveSetSubpixel) {
                Vector3f blendedColor = {0.0f, 0.0f, 0.0f};
                for (const auto& subpixel : subpixels) {
                    blendedColor += msaa_frame_buf[get_sub_index(subpixel[0], subpixel[1])];
                }
                blendedColor /= 4.0f;
                set_pixel({float(i), float(j), 0.0f}, blendedColor);
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(msaa_frame_buf.begin(), msaa_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(msaa_depth_buf.begin(), msaa_depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    msaa_frame_buf.resize(w * h * 4);
    msaa_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_sub_index(int x, int y)
{
    return (2*height-1-y)*2*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

void rst::rasterizer::set_sub_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    auto ind = (2*height-1-point.y())*2*width + point.x();
    msaa_frame_buf[ind] = color;
}

// clang-format on