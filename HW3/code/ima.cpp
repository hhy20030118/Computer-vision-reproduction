#include "ima.h"

//图像融合，消除拼接缝
bool OptimizeSeam(vector<Mat>dstimgs, vector<Mat>warpimgs, int m)
{
	//宽和高
auto p = warpimgs[0].rows, q = warpimgs[0].cols;
for (register int i = 0; i < p; ++i) {
	for (register int j = 0; j < q; ++j) {
		vector<int> overlapimgs;

		for (register int k = 0; k < warpimgs.size(); ++k) {
			if (warpimgs[k].at<Vec3b>(i, j) != Vec3b(0, 0, 0)) {
				overlapimgs.push_back(k);//统计在这个像素点有哪些图片重合了
			}
		}
		if (overlapimgs.size() == 1) {//只有一个，直接设为指定值
			warpimgs[m].at<Vec3b>(i, j) = warpimgs[overlapimgs[0]].at<Vec3b>(i, j);
		}
		else if (overlapimgs.size() > 1) {
			double weight = 0.0;
			vector<double> sum(3, 0.0);
			for (register int k = 0; k < overlapimgs.size(); ++k) {//加权，按照权重集中的权值
				for (register int c = 0; c < 3; ++c)sum[c] += dstimgs[overlapimgs[k]].at<float>(i, j) * warpimgs[overlapimgs[k]].at<Vec3b>(i, j)[c];
				weight += dstimgs[overlapimgs[k]].at<float>(i, j);
			}
			for (register int c = 0; c < 3; ++c) {
				sum[c] = sum[c] / weight; //归一化
				warpimgs[m].at<Vec3b>(i, j)[c] = (float)sum[c]; //重新设置加权后的图片
			}
		}
	}
}
	//原框架
	/*double Width = (end_x - start_x);//重叠区域的宽度  

	//图像加权融合，通过改变alpha修改DstImg与WarpImg像素权重，达到融合效果
	double alpha = 1.0;
	for (int i = 0; i < DstImg.rows; i++)
	{
		for (int j = start_x; j < end_x; j++)
		{
			for (int c = 0; c < 3; c++)
			{
				//如果图像WarpImg像素为0，则完全拷贝DstImg
				if (WarpImg.at<Vec3b>(i, j)[c] == 0)
				{
					alpha = 1.0;
				}
				else
				{
					double l = Width - (j - start_x); //重叠区域中某一像素点到拼接缝的距离
					alpha = l / Width;
				}
				DstImg.at<Vec3b>(i, j)[c] = DstImg.at<Vec3b>(i, j)[c] * alpha + WarpImg.at<Vec3b>(i, j)[c] * (1.0 - alpha);
			}
		}
	}
    */
	return true;
}
double dst(Mat p, Mat q) {
	//只算前两位（x,y)坐标，采用的是L1距离。为什么采用L1在报告中解释
	return abs(p.at<double>(0) - q.at<double>(0)) + abs(p.at<double>(1) - q.at<double>(1));
}
bool test_for_H(Mat H, int m, int n) {
	//检测是否正常
	if (H.dims <= 0)return false;
	auto l = (m + n) / 2;
	//四个顶点
	Mat a = (Mat_<double>(3, 1) << 0, m, 1);
	Mat b = (Mat_<double>(3, 1) << n, m, 1);
	Mat c = (Mat_<double>(3, 1) << n, 0, 1);
	Mat d = (Mat_<double>(3, 1) << 0, 0, 1);
	//中心点
	Mat e = (Mat_<double>(3, 1) << n/2, m/2, 1);
	//计算变换后的距离
	Mat p1 = H * a;
	Mat p2 = H * b;
	Mat p3 = H * c;
	Mat p4 = H * d;
	Mat p5 = H * e;
	//与原距离进行比较，相乘
	auto u = abs(dst(p1, p5)-l) * abs(dst(p5, p2)-l) * abs(dst(p3, p5)-l) * abs(dst(p5, p4)-l);
	cout << "u = " << u << endl;
	//1e12对于不同大小的图片要进行修改
	if (u > 1e+12)return false;
	return true;
}

bool Image_Stitching(vector<Mat>images, vector<Mat>warpimgs, bool draw)
{
	//创建SIFT特征检测器
	//10000比较大了，但是有的图片重合的不多，只能大一些
	int Hessian = 10000;
	Ptr<Feature2D>detector = SIFT::create(Hessian);

	//进行图像特征检测、特征描述
	//所有left和right都换成了vector
	vector<vector<KeyPoint>>keypoint(images.size());
	vector<Mat> descriptor(images.size());
	//两张图象之间的匹配点数
	vector<vector<int>> match_num(images.size(),vector<int>(images.size(),0));
	//计算匹配的“好图片”
	vector<int> goodimg;
	vector<vector<Mat>> H(images.size(), vector<Mat>(images.size()));
	for (register int i = 0; i < images.size(); ++i) {
		detector->detectAndCompute(images[i], Mat(), keypoint[i], descriptor[i]);
	}
	//detector->detectAndCompute(image_left, Mat(), keypoint_left, descriptor_left);
	//detector->detectAndCompute(image_right, Mat(), keypoint_right, descriptor_right);
	//每两个之间做一次
	for (register int i = 0; i < images.size(); ++i) {
		for (register int j = 0; j < images.size(); ++j) {
			if (i == j)continue;
			//使用FLANN算法进行特征描述子的匹配
			FlannBasedMatcher matcher;
			vector<DMatch>matches;
			matcher.match(descriptor[i], descriptor[j], matches);

			double Max = 0.0;
			for (int k = 0; k < matches.size(); k++)
			{
				//float distance –>代表这一对匹配的特征点描述符（本质是向量）的欧氏距离，数值越小也就说明两个特征点越相像。
				double dis = matches[k].distance;
				if (dis > Max)
				{
					Max = dis;
				}
			}

			//筛选出匹配程度高的关键点
			vector<DMatch>goodmatches;
			vector<Point2f>goodkeypoint_left, goodkeypoint_right;
			for (int k = 0; k < matches.size(); k++)
			{
				double dis = matches[k].distance;
				if (dis < 0.15 * Max)
				{
					/*
					以右图做透视变换
					左图->queryIdx:查询点索引（查询图像）
					右图->trainIdx:被查询点索引（目标图像）
					*/
					//注：对image_right图像做透视变换，故goodkeypoint_left对应queryIdx，goodkeypoint_right对应trainIdx
					//int queryIdx –>是测试图像的特征点描述符（descriptor）的下标，同时也是描述符对应特征点（keypoint)的下标。
					goodkeypoint_left.push_back(keypoint[i][matches[k].queryIdx].pt);
					//int trainIdx –> 是样本图像的特征点描述符的下标，同样也是相应的特征点的下标。
					goodkeypoint_right.push_back(keypoint[j][matches[k].trainIdx].pt);
					goodmatches.push_back(matches[k]);
				}
			}
			match_num[i][j] = min(goodkeypoint_left.size(), goodkeypoint_right.size());
			//绘制特征点    //暂时不能用，因为特征点是某两幅之间的，全显示出来会很乱
			if (draw)
			{
				Mat result;
				drawMatches(images[i], keypoint[i], images[j], keypoint[j], goodmatches, result);
				imshow("特征匹配", result);

				Mat temp_left = images[i].clone();
				for (int i = 0; i < goodkeypoint_left.size(); i++)
				{
					circle(temp_left, goodkeypoint_left[i], 3, Scalar(0, 255, 0), -1);
				}
				imshow("goodkeypoint_left", temp_left);

				Mat temp_right = images[j].clone();
				for (int i = 0; i < goodkeypoint_right.size(); i++)
				{
					circle(temp_right, goodkeypoint_right[i], 3, Scalar(0, 255, 0), -1);
				}
				imshow("goodkeypoint_right", temp_right);
			}

			//findHomography计算单应性矩阵至少需要4个点
			/*
			计算多个二维点对之间的最优单映射变换矩阵H（3x3），使用MSE或RANSAC方法，找到两平面之间的变换矩阵
			*/

			if (goodkeypoint_left.size() < 8 || goodkeypoint_right.size() < 8)continue;

			//获取图像right到图像left的投影映射矩阵，尺寸为3*3
			//注意顺序，srcPoints对应goodkeypoint_right，dstPoints对应goodkeypoint_left
			H[i][j] = findHomography(goodkeypoint_right, goodkeypoint_left, RANSAC);
			cout << "finish H   " << i << "  " << j << endl;
		}
	}
	int M = 8;   //阈值，剔除那些不合适的图像
	//寻找最合适的中心图像
	int max = 0, max_num = 0,maxw = 0;
	for (register int i = 0; i < images.size(); ++i) {
		int sum = 0;
		int k = 0;
		for (register int j = 0; j < images.size(); ++j) {
			//这是按照匹配点数确定好的H，已经废除
			/*if (match_num[i][j] > M) {
				sum += match_num[i][j];
				k++;
			}*/
			/*if (H[i][j].dims>0) {
				if (abs(H[i][j].at<double>(0, 2)) < 1000.0 && abs(H[i][j].at<double>(1, 2) )< 1000.0) {
					k++;
					sum += match_num[i][j];
				}
			}*/
			if (test_for_H(H[i][j], images[i].cols, images[i].rows)) {
				k++;
				sum += match_num[i][j];
			}
		}
		if (k > max) { //记录最合适的
			max = k;
			max_num = i;
			maxw = sum;
		}
		else if (k == max && sum > maxw) {
			max = k;
			max_num = i;
			maxw = sum;
		}
	}
	//剔除不合适的
	for (register int i = 0; i < images.size(); ++i) {
		if (test_for_H(H[max_num][i], images[i].cols, images[i].rows) && match_num[max_num][i] > M) {
			goodimg.push_back(i);
			//cout << H[max_num][i]<<endl;
		}
		cout << max_num<<"  " << i <<endl<< H[max_num][i] << endl;
	}
	//N是屏幕大小，i是中心图像
	int N = 1,i = max_num;
	//偏移矩阵，把图像放到中央
	Mat tr = (Mat_<double>(3, 3) << 1,0, N*images[i].cols,0,1, N*images[i].rows,0,0,1);
	Mat Endimg_origin;
	warpimgs.resize(goodimg.size() + 1);
	vector<Mat> dstImage(goodimg.size()+1);
	warpPerspective(images[i],warpimgs[goodimg.size()], tr, Size((2 * N + 1) * images[i].cols, (2 * N + 1) * images[i].rows));
	Endimg_origin = warpimgs[goodimg.size()].clone();

	Mat srcGray1;
	cvtColor(warpimgs[goodimg.size()], srcGray1, COLOR_RGB2GRAY);// 转换为灰度图像
	Mat srcBinary1;
	threshold(srcGray1, srcBinary1, 0, 255, cv::THRESH_BINARY);// 转换为二值图像
	distanceTransform(srcBinary1, dstImage[goodimg.size()], DIST_L2, DIST_MASK_PRECISE);//调用函数计算到零点的距离
	//normalize(dstImage[i], dstImage[i], 0, 1., cv::NORM_MINMAX);// 归一化，不需要
	//对于每一张好图片处理
	for (register int j = 0; j < goodimg.size(); ++j) {
		//对image进行透视变换
		warpPerspective(images[goodimg[j]], warpimgs[j], tr * H[i][goodimg[j]], Size((2 * N + 1) * images[i].cols, (2 * N + 1) * images[i].rows));

		//每个都显示太多了，非必要不用
		//namedWindow("透视变换", WINDOW_NORMAL);
		//imshow("透视变换", warpimgs[j]);

		//将image_right拷贝到一起，完成图像拼接

		Mat roi(Endimg_origin, Rect(0, 0, (2 * N + 1) * images[i].cols, (2 * N + 1) * images[i].rows));
		warpimgs[j].copyTo(roi, warpimgs[j]);
		//一样，生成权重集
		Mat srcGray;
		cvtColor(warpimgs[j], srcGray, COLOR_RGB2GRAY);
		Mat srcBinary;
		threshold(srcGray, srcBinary, 0, 255, cv::THRESH_BINARY);

		distanceTransform(srcBinary, dstImage[j], DIST_L2, DIST_MASK_PRECISE);
		//normalize(dstImage[j], dstImage[j], 0, 1., cv::NORM_MINMAX);

		//namedWindow("dst", WINDOW_NORMAL);   //显示距离图像 未归一化之前没有用
		//imshow("dst", dstImage[j]);
		//原框架
		/*//透视变换左上角(0,0,1)
		Mat V2 = (Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
		Mat V1 = H[i][j] * V2;
		Point left_top;
		left_top.x = V1.at<double>(0, 0) / V1.at<double>(2, 0);
		left_top.y = V1.at<double>(1, 0) / V1.at<double>(2, 0);
		if (left_top.x < 0)left_top.x = 0;

		//透视变换左下角(0,src.rows,1)
		V2 = (Mat_<double>(3, 1) << 0.0, images[i].rows, 1.0);
		V1 = H[i][j] * V2;
		Point left_bottom;
		left_bottom.x = V1.at<double>(0, 0) / V1.at<double>(2, 0);
		left_bottom.y = V1.at<double>(1, 0) / V1.at<double>(2, 0);
		if (left_bottom.x < 0)left_bottom.x = 0;

		int start_x = min(left_top.x, left_bottom.x);//重合区域起点
		int end_x = images[i].cols;//重合区域终点

		OptimizeSeam(start_x, end_x, WarpImg, DstImg); //图像融合*/

	}
	
	namedWindow("图像全景拼接", WINDOW_FREERATIO);
	resizeWindow("图像全景拼接", 640, 480);
	imshow("图像全景拼接", Endimg_origin);
	
	OptimizeSeam(dstImage,warpimgs, goodimg.size());
	namedWindow("图像融合", WINDOW_FREERATIO);
	resizeWindow("图像融合", 640, 480);
	imshow("图像融合", warpimgs[goodimg.size()]);
	imwrite("test1.jpg", warpimgs[goodimg.size()]);
	return true;
}


bool OpenCV_Stitching(vector<Mat>images)
{
	//将待拼接图片放进容器里面
	//vector<Mat>images;
	//images.push_back(image_left);
	//images.push_back(image_right);

	//创建Stitcher模型
	Ptr<Stitcher>stitcher = Stitcher::create();

	Mat result;
	Stitcher::Status status = stitcher->stitch(images, result);// 使用stitch函数进行拼接

	if (status != Stitcher::OK) return false;
	namedWindow("OpenCV图像全景拼接", WINDOW_NORMAL);  //适配窗口大小，要不显示不出来
	imshow("OpenCV图像全景拼接", result);
	imwrite("test.jpg",result);
	return true;
}

