#include "ima.h"

//ͼ���ںϣ�����ƴ�ӷ�
bool OptimizeSeam(vector<Mat>dstimgs, vector<Mat>warpimgs, int m)
{
	//��͸�
auto p = warpimgs[0].rows, q = warpimgs[0].cols;
for (register int i = 0; i < p; ++i) {
	for (register int j = 0; j < q; ++j) {
		vector<int> overlapimgs;

		for (register int k = 0; k < warpimgs.size(); ++k) {
			if (warpimgs[k].at<Vec3b>(i, j) != Vec3b(0, 0, 0)) {
				overlapimgs.push_back(k);//ͳ����������ص�����ЩͼƬ�غ���
			}
		}
		if (overlapimgs.size() == 1) {//ֻ��һ����ֱ����Ϊָ��ֵ
			warpimgs[m].at<Vec3b>(i, j) = warpimgs[overlapimgs[0]].at<Vec3b>(i, j);
		}
		else if (overlapimgs.size() > 1) {
			double weight = 0.0;
			vector<double> sum(3, 0.0);
			for (register int k = 0; k < overlapimgs.size(); ++k) {//��Ȩ������Ȩ�ؼ��е�Ȩֵ
				for (register int c = 0; c < 3; ++c)sum[c] += dstimgs[overlapimgs[k]].at<float>(i, j) * warpimgs[overlapimgs[k]].at<Vec3b>(i, j)[c];
				weight += dstimgs[overlapimgs[k]].at<float>(i, j);
			}
			for (register int c = 0; c < 3; ++c) {
				sum[c] = sum[c] / weight; //��һ��
				warpimgs[m].at<Vec3b>(i, j)[c] = (float)sum[c]; //�������ü�Ȩ���ͼƬ
			}
		}
	}
}
	//ԭ���
	/*double Width = (end_x - start_x);//�ص�����Ŀ��  

	//ͼ���Ȩ�ںϣ�ͨ���ı�alpha�޸�DstImg��WarpImg����Ȩ�أ��ﵽ�ں�Ч��
	double alpha = 1.0;
	for (int i = 0; i < DstImg.rows; i++)
	{
		for (int j = start_x; j < end_x; j++)
		{
			for (int c = 0; c < 3; c++)
			{
				//���ͼ��WarpImg����Ϊ0������ȫ����DstImg
				if (WarpImg.at<Vec3b>(i, j)[c] == 0)
				{
					alpha = 1.0;
				}
				else
				{
					double l = Width - (j - start_x); //�ص�������ĳһ���ص㵽ƴ�ӷ�ľ���
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
	//ֻ��ǰ��λ��x,y)���꣬���õ���L1���롣Ϊʲô����L1�ڱ����н���
	return abs(p.at<double>(0) - q.at<double>(0)) + abs(p.at<double>(1) - q.at<double>(1));
}
bool test_for_H(Mat H, int m, int n) {
	//����Ƿ�����
	if (H.dims <= 0)return false;
	auto l = (m + n) / 2;
	//�ĸ�����
	Mat a = (Mat_<double>(3, 1) << 0, m, 1);
	Mat b = (Mat_<double>(3, 1) << n, m, 1);
	Mat c = (Mat_<double>(3, 1) << n, 0, 1);
	Mat d = (Mat_<double>(3, 1) << 0, 0, 1);
	//���ĵ�
	Mat e = (Mat_<double>(3, 1) << n/2, m/2, 1);
	//����任��ľ���
	Mat p1 = H * a;
	Mat p2 = H * b;
	Mat p3 = H * c;
	Mat p4 = H * d;
	Mat p5 = H * e;
	//��ԭ������бȽϣ����
	auto u = abs(dst(p1, p5)-l) * abs(dst(p5, p2)-l) * abs(dst(p3, p5)-l) * abs(dst(p5, p4)-l);
	cout << "u = " << u << endl;
	//1e12���ڲ�ͬ��С��ͼƬҪ�����޸�
	if (u > 1e+12)return false;
	return true;
}

bool Image_Stitching(vector<Mat>images, vector<Mat>warpimgs, bool draw)
{
	//����SIFT���������
	//10000�Ƚϴ��ˣ������е�ͼƬ�غϵĲ��ֻ࣬�ܴ�һЩ
	int Hessian = 10000;
	Ptr<Feature2D>detector = SIFT::create(Hessian);

	//����ͼ��������⡢��������
	//����left��right��������vector
	vector<vector<KeyPoint>>keypoint(images.size());
	vector<Mat> descriptor(images.size());
	//����ͼ��֮���ƥ�����
	vector<vector<int>> match_num(images.size(),vector<int>(images.size(),0));
	//����ƥ��ġ���ͼƬ��
	vector<int> goodimg;
	vector<vector<Mat>> H(images.size(), vector<Mat>(images.size()));
	for (register int i = 0; i < images.size(); ++i) {
		detector->detectAndCompute(images[i], Mat(), keypoint[i], descriptor[i]);
	}
	//detector->detectAndCompute(image_left, Mat(), keypoint_left, descriptor_left);
	//detector->detectAndCompute(image_right, Mat(), keypoint_right, descriptor_right);
	//ÿ����֮����һ��
	for (register int i = 0; i < images.size(); ++i) {
		for (register int j = 0; j < images.size(); ++j) {
			if (i == j)continue;
			//ʹ��FLANN�㷨�������������ӵ�ƥ��
			FlannBasedMatcher matcher;
			vector<DMatch>matches;
			matcher.match(descriptor[i], descriptor[j], matches);

			double Max = 0.0;
			for (int k = 0; k < matches.size(); k++)
			{
				//float distance �C>������һ��ƥ�������������������������������ŷ�Ͼ��룬��ֵԽСҲ��˵������������Խ����
				double dis = matches[k].distance;
				if (dis > Max)
				{
					Max = dis;
				}
			}

			//ɸѡ��ƥ��̶ȸߵĹؼ���
			vector<DMatch>goodmatches;
			vector<Point2f>goodkeypoint_left, goodkeypoint_right;
			for (int k = 0; k < matches.size(); k++)
			{
				double dis = matches[k].distance;
				if (dis < 0.15 * Max)
				{
					/*
					����ͼ��͸�ӱ任
					��ͼ->queryIdx:��ѯ����������ѯͼ��
					��ͼ->trainIdx:����ѯ��������Ŀ��ͼ��
					*/
					//ע����image_rightͼ����͸�ӱ任����goodkeypoint_left��ӦqueryIdx��goodkeypoint_right��ӦtrainIdx
					//int queryIdx �C>�ǲ���ͼ�����������������descriptor�����±꣬ͬʱҲ����������Ӧ�����㣨keypoint)���±ꡣ
					goodkeypoint_left.push_back(keypoint[i][matches[k].queryIdx].pt);
					//int trainIdx �C> ������ͼ������������������±꣬ͬ��Ҳ����Ӧ����������±ꡣ
					goodkeypoint_right.push_back(keypoint[j][matches[k].trainIdx].pt);
					goodmatches.push_back(matches[k]);
				}
			}
			match_num[i][j] = min(goodkeypoint_left.size(), goodkeypoint_right.size());
			//����������    //��ʱ�����ã���Ϊ��������ĳ����֮��ģ�ȫ��ʾ���������
			if (draw)
			{
				Mat result;
				drawMatches(images[i], keypoint[i], images[j], keypoint[j], goodmatches, result);
				imshow("����ƥ��", result);

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

			//findHomography���㵥Ӧ�Ծ���������Ҫ4����
			/*
			��������ά���֮������ŵ�ӳ��任����H��3x3����ʹ��MSE��RANSAC�������ҵ���ƽ��֮��ı任����
			*/

			if (goodkeypoint_left.size() < 8 || goodkeypoint_right.size() < 8)continue;

			//��ȡͼ��right��ͼ��left��ͶӰӳ����󣬳ߴ�Ϊ3*3
			//ע��˳��srcPoints��Ӧgoodkeypoint_right��dstPoints��Ӧgoodkeypoint_left
			H[i][j] = findHomography(goodkeypoint_right, goodkeypoint_left, RANSAC);
			cout << "finish H   " << i << "  " << j << endl;
		}
	}
	int M = 8;   //��ֵ���޳���Щ�����ʵ�ͼ��
	//Ѱ������ʵ�����ͼ��
	int max = 0, max_num = 0,maxw = 0;
	for (register int i = 0; i < images.size(); ++i) {
		int sum = 0;
		int k = 0;
		for (register int j = 0; j < images.size(); ++j) {
			//���ǰ���ƥ�����ȷ���õ�H���Ѿ��ϳ�
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
		if (k > max) { //��¼����ʵ�
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
	//�޳������ʵ�
	for (register int i = 0; i < images.size(); ++i) {
		if (test_for_H(H[max_num][i], images[i].cols, images[i].rows) && match_num[max_num][i] > M) {
			goodimg.push_back(i);
			//cout << H[max_num][i]<<endl;
		}
		cout << max_num<<"  " << i <<endl<< H[max_num][i] << endl;
	}
	//N����Ļ��С��i������ͼ��
	int N = 1,i = max_num;
	//ƫ�ƾ��󣬰�ͼ��ŵ�����
	Mat tr = (Mat_<double>(3, 3) << 1,0, N*images[i].cols,0,1, N*images[i].rows,0,0,1);
	Mat Endimg_origin;
	warpimgs.resize(goodimg.size() + 1);
	vector<Mat> dstImage(goodimg.size()+1);
	warpPerspective(images[i],warpimgs[goodimg.size()], tr, Size((2 * N + 1) * images[i].cols, (2 * N + 1) * images[i].rows));
	Endimg_origin = warpimgs[goodimg.size()].clone();

	Mat srcGray1;
	cvtColor(warpimgs[goodimg.size()], srcGray1, COLOR_RGB2GRAY);// ת��Ϊ�Ҷ�ͼ��
	Mat srcBinary1;
	threshold(srcGray1, srcBinary1, 0, 255, cv::THRESH_BINARY);// ת��Ϊ��ֵͼ��
	distanceTransform(srcBinary1, dstImage[goodimg.size()], DIST_L2, DIST_MASK_PRECISE);//���ú������㵽���ľ���
	//normalize(dstImage[i], dstImage[i], 0, 1., cv::NORM_MINMAX);// ��һ��������Ҫ
	//����ÿһ�ź�ͼƬ����
	for (register int j = 0; j < goodimg.size(); ++j) {
		//��image����͸�ӱ任
		warpPerspective(images[goodimg[j]], warpimgs[j], tr * H[i][goodimg[j]], Size((2 * N + 1) * images[i].cols, (2 * N + 1) * images[i].rows));

		//ÿ������ʾ̫���ˣ��Ǳ�Ҫ����
		//namedWindow("͸�ӱ任", WINDOW_NORMAL);
		//imshow("͸�ӱ任", warpimgs[j]);

		//��image_right������һ�����ͼ��ƴ��

		Mat roi(Endimg_origin, Rect(0, 0, (2 * N + 1) * images[i].cols, (2 * N + 1) * images[i].rows));
		warpimgs[j].copyTo(roi, warpimgs[j]);
		//һ��������Ȩ�ؼ�
		Mat srcGray;
		cvtColor(warpimgs[j], srcGray, COLOR_RGB2GRAY);
		Mat srcBinary;
		threshold(srcGray, srcBinary, 0, 255, cv::THRESH_BINARY);

		distanceTransform(srcBinary, dstImage[j], DIST_L2, DIST_MASK_PRECISE);
		//normalize(dstImage[j], dstImage[j], 0, 1., cv::NORM_MINMAX);

		//namedWindow("dst", WINDOW_NORMAL);   //��ʾ����ͼ�� δ��һ��֮ǰû����
		//imshow("dst", dstImage[j]);
		//ԭ���
		/*//͸�ӱ任���Ͻ�(0,0,1)
		Mat V2 = (Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
		Mat V1 = H[i][j] * V2;
		Point left_top;
		left_top.x = V1.at<double>(0, 0) / V1.at<double>(2, 0);
		left_top.y = V1.at<double>(1, 0) / V1.at<double>(2, 0);
		if (left_top.x < 0)left_top.x = 0;

		//͸�ӱ任���½�(0,src.rows,1)
		V2 = (Mat_<double>(3, 1) << 0.0, images[i].rows, 1.0);
		V1 = H[i][j] * V2;
		Point left_bottom;
		left_bottom.x = V1.at<double>(0, 0) / V1.at<double>(2, 0);
		left_bottom.y = V1.at<double>(1, 0) / V1.at<double>(2, 0);
		if (left_bottom.x < 0)left_bottom.x = 0;

		int start_x = min(left_top.x, left_bottom.x);//�غ��������
		int end_x = images[i].cols;//�غ������յ�

		OptimizeSeam(start_x, end_x, WarpImg, DstImg); //ͼ���ں�*/

	}
	
	namedWindow("ͼ��ȫ��ƴ��", WINDOW_FREERATIO);
	resizeWindow("ͼ��ȫ��ƴ��", 640, 480);
	imshow("ͼ��ȫ��ƴ��", Endimg_origin);
	
	OptimizeSeam(dstImage,warpimgs, goodimg.size());
	namedWindow("ͼ���ں�", WINDOW_FREERATIO);
	resizeWindow("ͼ���ں�", 640, 480);
	imshow("ͼ���ں�", warpimgs[goodimg.size()]);
	imwrite("test1.jpg", warpimgs[goodimg.size()]);
	return true;
}


bool OpenCV_Stitching(vector<Mat>images)
{
	//����ƴ��ͼƬ�Ž���������
	//vector<Mat>images;
	//images.push_back(image_left);
	//images.push_back(image_right);

	//����Stitcherģ��
	Ptr<Stitcher>stitcher = Stitcher::create();

	Mat result;
	Stitcher::Status status = stitcher->stitch(images, result);// ʹ��stitch��������ƴ��

	if (status != Stitcher::OK) return false;
	namedWindow("OpenCVͼ��ȫ��ƴ��", WINDOW_NORMAL);  //���䴰�ڴ�С��Ҫ����ʾ������
	imshow("OpenCVͼ��ȫ��ƴ��", result);
	imwrite("test.jpg",result);
	return true;
}

