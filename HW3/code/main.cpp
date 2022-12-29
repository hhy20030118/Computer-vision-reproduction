#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/xfeatures2d.hpp>
#include<opencv2/stitching.hpp>
#include "ima.h"
#include<string>
#include<vector>
#define N 100
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

int main()
{
	vector<Mat> images;
	// 按照名字读，虽然比较规范但是重命名花费的时间太长了，改成读取所有jpg
	/*int k = 0;
	string s = "";
	for (register int i = 0; i < N; ++i) {
		s = to_string(i+1);
		images.push_back(imread("./imgs/2/0"+s+".jpg"));
		if (images[i].empty()) {
			k = i-1;
			break;
			
		}
	}
	images.pop_back();
	*/
	//中间文件夹的数字可以改，对应不同的数据集
	//1 是测试用的4个
	//2 照片间的关系不大
	//3 大量数据集
	//4 5 比较常规
	//1 4 5 的效果比较好，3 勉强能看， 2 比较爆炸
	String pattern = "./imgs/1/*.jpg";
	vector<String> fn;
	glob(pattern, fn, false);
	for (register int i = 0; i < fn.size(); ++i) {
		images.push_back(imread(fn[i]));
	}
	cout << "finish load" << endl;
	//如果只有一张或没有那么没法进行下去
	if (images.size() <=1) {
		cout << "scarce Image!" << endl;
		system("pause");
		return -1;
	}
	//Mat image_left = imread("./left.jpg");
	//Mat image_right = imread("./right.jpg");

	vector<Mat>warpimgs;
	if (!Image_Stitching(images, warpimgs, false))
	{
		cout << "can not stitching the image!" << endl;
		system("pause");
		return false;
	}


	if (!OpenCV_Stitching(images))
	{
		cout << "can not stitching the image!" << endl;
		system("pause");
		return false;
	}

	waitKey(0);
	destroyAllWindows();
	system("pause");
	return 0;
}
