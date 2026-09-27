#include "reprojection.hpp"
#include <iostream>
#include <vector>
#include <iomanip>
#include <random>


int main()
{    //动态内参
    CameraIntrinsics K;
    std::cout << "步骤1:输入相机内参3x3矩阵 K" << std::endl;
    std::cout << "请按行从左到右,从上到下输入9个数字(用空格分隔)" << std::endl;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            std::cin  >> K.K[i][j];
        }
    }


     //动态外参
     CameraExtrinsics ext;
     std::cout << "步骤2:输入相机外参" << std::endl;
     std::cout << "请输入3x3旋转矩阵 R(按行输入,9个数字,空格分隔)" << std::endl;
    
     for (int i = 0; i < 3; ++i)
     {
        for (int j = 0; j < 3; ++j)
        {
            std::cin >> ext.R[i][j];
        }

     }
     
     std::cout << "请输入3x1平移向量 t(输入三个数字,空格分隔)"<< std::endl;

     for (int i = 0; i < 3; ++i)
     {
        std::cin >> ext.t[i];
     }



     //动态三维点
     std::vector<Point3D> world_points;
     int num_points;

     std::cout << "步骤3:输入世界系的三维点"<< std::endl;
     std::cout << "请输入你要测试的三维点数量"<< std::endl;
     std::cin >> num_points;


     for (int i = 0; i < num_points; ++i)
     {
        Point3D pw;
        std::cout << "请输入第" << i + 1 << "个点的世界坐标(x,y,z)"<<std::endl;
        std::cin >> pw.data[0]  >> pw.data[1] >> pw.data[2];
        world_points.push_back(pw);//存入vector 
     }


     //重投影计算和误差分析
     std:: cout << "步骤4:执行重投影与误差计算"<< std::endl;


     //构造随机数生成器，模拟真实观测噪声（均值0，标准差1像素）
     std::default_random_engine generater;
     std::normal_distribution<double> noise_dist(0.0, 1.0);

     //遍历 vector 中所有点
     for (size_t i = 0; i < world_points.size(); ++i)
     {
        const auto& pw = world_points[i];
        Point2D proj;

        std::cout << "处理第"<< i + 1 << "个点" << std::endl;
        std::cout << "世界坐标 Pw:("<< pw.data[0] << ","<< pw.data[1] << "," << pw.data[2]<< ")"<< std::endl;

        if (!reproject(pw, K, ext, proj))
        {
            std::cout << "[warning]深度 Z <= 0,该点无法投影,已跳过！"<< std::endl;
            continue;
        }

        std::cout << "理论计算的像素坐标:(" << std::fixed << std::setprecision(2) << proj.data[0] << "," << proj.data[1] << ")" << std::endl;

        Point2D obs_noisy;
        obs_noisy.data[0] = proj.data[0] + noise_dist(generater);
        obs_noisy.data[1] = proj.data[1] + noise_dist(generater);

        double dist_noisy = calcPixelDistance(proj, obs_noisy);
        std::cout << "【构造的观测点】坐标: (" 
                  << std::fixed << std::setprecision(2) << obs_noisy.data[0] << ", " << obs_noisy.data[1] << ")" << std::endl;
        std::cout << "【重投影误差】像素欧氏距离: " 
                  << std::fixed << std::setprecision(4) << dist_noisy << " px" << std::endl;
            
    }
    std::cout << "测试结束"<< std::endl;
    return 0;
}        

    
    

     




        
// 第一次培训作业 - 重投影计算
