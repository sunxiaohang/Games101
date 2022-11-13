#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;//这里只有平移变化
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1,-eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

//    {1,0,0}
//    {0,1,0}
//    {0,0,1}
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f N, R;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

    N << 0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1], axis[0], 0;

    float Ca = cos(angle/180 * MY_PI);
    float Sa = sin(angle/180 * MY_PI);
    R = Ca * I + (1 - Ca)*axis*axis.transpose() + Sa * N;

    rotation << R(0,0),R(0,1),R(0,2),0,
                R(1,0),R(1,1),R(1,2),0,
                R(2,0),R(2,1),R(2,2),0,
                0, 0, 0, 1;

    return rotation;
}

Eigen::Matrix4f get_model_matrix(float rotation_angleX, float rotation_angleY, float rotation_angleZ)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translateX, translateY, translateZ;
    translateX << 1, 0, 0, 0,
                 0, cos(rotation_angleX/180*acos(-1)), -sin(rotation_angleX/180*acos(-1)), 0,
                 0, sin(rotation_angleX/180*acos(-1)), cos(rotation_angleX/180*acos(-1)), 0,
                 0,0,0,1;
    translateY << cos(rotation_angleY/180*acos(-1)), 0, sin(rotation_angleY/180*acos(-1)), 0,
                 0, 1, 0, 0,
                 -sin(rotation_angleY/180*acos(-1)), 0, cos(rotation_angleY/180*acos(-1)), 0,
                 0,0,0,1;
    translateZ << cos(rotation_angleZ/180*acos(-1)), -sin(rotation_angleZ/180*acos(-1)), 0, 0,
                 sin(rotation_angleZ/180*acos(-1)), cos(rotation_angleZ/180*acos(-1)),0 , 0,
                 0,0,1,0,
                 0,0,0,1;
    model = translateX * translateY * translateZ * model;
    return model;
}

//fov = 45, 1, 0.1, 50
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    projection << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear*zFar,
        0,0,1,0;

    float halve = tan(eye_fov*MY_PI/2/180);
    float top = zNear * halve;
    float bottom = - top;
    float right = top * aspect_ratio;
    float left = - right;

    Eigen::Matrix4f translate;
    translate << 2/(right - left), 0, 0, -(right + left)/2,
         0, 2/(top - bottom), 0, -(top + bottom)/2,
         0, 0, 2/(zNear - zFar), -(zNear + zFar)/2,
         0, 0, 0, 1;

    projection = translate * projection;
    return projection;
}

int main(int argc, const char** argv)
{
    float angleX = 0;
    float angleY = 0;
    float angleZ = 0;

    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angleX = std::stof(argv[2]); // -r by default
        angleY = std::stof(argv[2]); // -r by default
        angleZ = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700); //定义屏幕的宽高

    Eigen::Vector3f eye_pos = {0, 0, 5};//定义人眼位置

    std::vector<Eigen::Vector3f> pos{{0.5, 0, -0.5}, {0, 0.5, -0.5}, {-0.5, 0, -0.5}}; //定义三角形顶点坐标

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};//三角形顶点索引

    auto pos_id = r.load_positions(pos);//加载顶点位置到光栅化器
    auto ind_id = r.load_indices(ind);//加载顶点索引到光栅化器

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angleX, angleY, angleZ));
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

        r.set_model(get_model_matrix(angleX, angleY, angleZ));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        angleX += 10;
        angleY += 10;
        angleZ += 10;

        if (key == 's') {
            angleX += 10;
        }
        else if (key == 'w') {
            angleX -= 10;
        }

        if (key == 'd') {
            angleY += 10;
        }
        else if (key == 'a') {
            angleY -= 10;
        }

        if (key == 'q') {
            angleZ += 10;
        }
        else if (key == 'e') {
            angleZ -= 10;
        }
    }

    return 0;
}
