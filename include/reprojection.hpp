#pragma once 
#include <array>

using Matrix3x3 = std::array<std::array<double,3>,3>;
//ai说这样更符合现在cpp，二维数组是c语言写法
using Vector3 = std ::array<double,3>;

struct Point3D
{   std::array<double,3> data;//三维点x,y,z
    
};

struct Point2D
{   std::array<double,2> data;//二维点，u，v

};

struct CameraIntrinsics//相机内参
{
    Matrix3x3 K;//相机系转平面
};

struct CameraExtrinsics//相机外参
{
    Matrix3x3 R;//旋转矩阵将世界系转相机系
    Vector3 t;//原点平移至新原点
};

bool reproject(const Point3D& pw, const CameraIntrinsics& K, const CameraExtrinsics& ext, Point2D& out_proj);
//pw是三维点坐标，
//K是内参，
//ext包括旋转矩阵和平移向量，
//out_proj即为计算出的点

double calcPixelDistance(const Point2D& p1, const Point2D& p2);
//计算两像素点的欧氏距离
