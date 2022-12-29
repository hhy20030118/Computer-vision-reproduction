#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/xfeatures2d.hpp>
#include<opencv2/stitching.hpp>
#include "core/core.hpp"
#include "imgproc/imgproc.hpp"
#include "highgui/highgui.hpp"
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//1、使用特征检测算法找到两张图像中相似的点，计算变换矩阵
//2、将图像right透视变换后得到的图片与图像left拼接

//图像融合，消除拼接缝。输入一个权重集，一个变换集，进行全局加权 i是指定的中央图片，操作都在i上进行
bool OptimizeSeam(vector<Mat>dstimgs, vector<Mat>warpimgs, int i);
//输入照片数据集和一个变换后图像的数据集(空），bool基本不用，因为打印出来太多了
bool Image_Stitching(vector<Mat>images, vector<Mat>warpimgs, bool draw);
//输入照片数据集即可
bool OpenCV_Stitching(vector<Mat>images);
//测试变换矩阵是不是“病态的”
bool test_for_H(Mat H, int m, int n);
//衡量两个坐标之间的距离
double dst(Mat p, Mat q);