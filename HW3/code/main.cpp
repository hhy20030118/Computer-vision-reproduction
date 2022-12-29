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
	// �������ֶ�����Ȼ�ȽϹ淶�������������ѵ�ʱ��̫���ˣ��ĳɶ�ȡ����jpg
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
	//�м��ļ��е����ֿ��Ըģ���Ӧ��ͬ�����ݼ�
	//1 �ǲ����õ�4��
	//2 ��Ƭ��Ĺ�ϵ����
	//3 �������ݼ�
	//4 5 �Ƚϳ���
	//1 4 5 ��Ч���ȽϺã�3 ��ǿ�ܿ��� 2 �Ƚϱ�ը
	String pattern = "./imgs/1/*.jpg";
	vector<String> fn;
	glob(pattern, fn, false);
	for (register int i = 0; i < fn.size(); ++i) {
		images.push_back(imread(fn[i]));
	}
	cout << "finish load" << endl;
	//���ֻ��һ�Ż�û����ôû��������ȥ
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
