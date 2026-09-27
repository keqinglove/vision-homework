#include "reprojection.hpp"
#include <cmath>

bool reproject(const Point3D& pw, const CameraIntrinsics& K, const CameraExtrinsics& ext, Point2D& out_proj)
{
    Vector3 Pc ={0.0,0.0,0.0};//初始化转到相机系的三维点

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Pc[i] += ext.R[i][j] * pw.data[j];//转换坐标系后的新坐标
        }
        Pc[i] += ext.t[i];//再平移
    }


 if (Pc[2] <= 1e-6)//ai建议用这个数字避免精度误差
 {
    return false;//非正深度，投影失败
 }

 double x_norm = Pc[0] / Pc[2];
 double y_norm = Pc[1] / Pc[2];//将相机系点归一至一个平面

 double normlized[3] = {x_norm, y_norm, 1.0};//归一点在相机系下的坐标
 out_proj.data = {0.0, 0.0};//初始化计算点

 for (int i=0; i < 2; ++i)
 {
    for (int j = 0; j < 3; ++j)
    {
        out_proj.data[i] += K.K[i][j] * normlized[j];
        //归一点通过相机内参转换到照片上
    }
 }
 return true; //投影成功
}

double calcPixelDistance(const Point2D& p1, const Point2D& p2)
{
    double dx = p1.data[0] - p2.data[0];
    double dy = p1.data[1] - p2.data[1];

    return std::hypot(dx,dy);
    //等同于sqrt（dx*dx+ dy*dy)
}

