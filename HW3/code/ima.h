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

//1��ʹ����������㷨�ҵ�����ͼ�������Ƶĵ㣬����任����
//2����ͼ��right͸�ӱ任��õ���ͼƬ��ͼ��leftƴ��

//ͼ���ںϣ�����ƴ�ӷ졣����һ��Ȩ�ؼ���һ���任��������ȫ�ּ�Ȩ i��ָ��������ͼƬ����������i�Ͻ���
bool OptimizeSeam(vector<Mat>dstimgs, vector<Mat>warpimgs, int i);
//������Ƭ���ݼ���һ���任��ͼ������ݼ�(�գ���bool�������ã���Ϊ��ӡ����̫����
bool Image_Stitching(vector<Mat>images, vector<Mat>warpimgs, bool draw);
//������Ƭ���ݼ�����
bool OpenCV_Stitching(vector<Mat>images);
//���Ա任�����ǲ��ǡ���̬�ġ�
bool test_for_H(Mat H, int m, int n);
//������������֮��ľ���
double dst(Mat p, Mat q);