#include<iostream>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;
int main()
{
	int board_w = 9;//横向的角点数目
	int board_h = 6;//纵向角点的数目
	int n_boards = 15;//需要采集图像的数目
	float delay = 500.f;//相机标定时需要采用的图像帧数  
	float image_sf = 0.5;//图像比例因子
	int board_n = board_w * board_h;//角点总数
	Size board_sz = Size(board_w, board_h);//标定板大小
	VideoCapture capture(0);//打开摄像头
	if (!capture.isOpened())
	{
		cout << "不能打开摄像头\n";
		return -1;
	}
	vector<vector<Point2f> > image_points;//目标点在世界坐标系中的点对应的图像点
	vector<vector<Point3f> > object_points;//目标点在世界坐标系中的点


	//获得角点的视图，直到16张图片循环成功为止
	//获得所有的角点
	double last_captured_timestamp = 0;//上次录制的时间戳
	Size image_size;//指定图像的大小
	while (image_points.size() < (size_t)n_boards)//循环16次
	{
		Mat image0, image;
		capture >> image0;//输入帧数
		image_size = image0.size();
		resize(image0, image, Size(), image_sf, image_sf, INTER_LINEAR);//调整大小


		//发现角点
		vector<Point2f> corners;//角点坐标
		bool found = findChessboardCorners(image, board_sz, corners);//找角点
		drawChessboardCorners(image, board_sz, corners, found);//画出角点


		//添加数据
		double timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;//计算时间戳（秒数）
		if (found && timestamp - last_captured_timestamp > 1) //如果两次图片之间的时间大于1秒
		{
			last_captured_timestamp = timestamp;//把这次的时间记下来，当下一次的上次
			image ^= Scalar::all(255);//搞颜色，不同的颜色，便于区分
			//使缩小的角点坐标变回原来那么大---------corners *= (1.0 / image_sf);不行
			Mat mcorners(corners);
			mcorners *= (1.0 / image_sf);//之前corners缩小了二分之一，现在又扩大了
			//缩放角坐标
			image_points.push_back(corners);
			object_points.push_back(vector<Point3f>());
			vector<Point3f> &opts = object_points.back();

			opts.resize(board_n);
			for (int j = 0; j < board_n; j++) 
			{
				opts[j] = Point3f(static_cast<float>(j / board_w),
					static_cast<float>(j % board_w), 0.0f);
			}
			cout << "已成功 " << static_cast<uint>(image_points.size())
				<< "张图片，一共" << n_boards << " 张图片\n" << endl;
		}
		imshow("校准", image);

		//如果收集到图片就上色
		if ((waitKey(30) & 255) == 27)
			return -1;
	}



	//结束收集WHILE循环.

	destroyWindow("校准");
	cout << "\n\n***  摄像头标定...\n" << endl;

	//摄像头标定！

	Mat intrinsic_matrix/*内参矩阵*/, distortion_coeffs/*畸变系数*/;
	double err = calibrateCamera(
		object_points, image_points, image_size, intrinsic_matrix,
		distortion_coeffs,noArray(), noArray(),
		CALIB_ZERO_TANGENT_DIST/*设定切向畸变参数（p1, p2）为零。*/ | CALIB_FIX_PRINCIPAL_POINT/*在进行优化时会固定光轴点。当CV_CALIB_USE_INTRINSIC_GUESS参数被设置，光轴点将保持在中心或者某个输入的值。*/);

	//保存固有特性和失真
	cout << " *** 完成了!\n\n重投影误差是 " << err
		<< "\n储存 Intrinsics.xml 文件\n\n";
	FileStorage fs("intrinsics.xml", FileStorage::WRITE);
	fs << "image_width" << image_size.width << "image_height" << image_size.height
		<< "camera_matrix" << intrinsic_matrix << "distortion_coefficients"
		<< distortion_coeffs;
	fs.release();//释放

	//加载这些矩阵的例子:
	fs.open("intrinsics.xml", FileStorage::READ);
	cout << "\nimage width: " << static_cast<int>(fs["image_width"]);
	cout << "\nimage height: " << static_cast<int>(fs["image_height"]);
	Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
	fs["camera_matrix"] >> intrinsic_matrix_loaded;
	fs["distortion_coefficients"] >> distortion_coeffs_loaded;
	cout << "\nintrinsic matrix:" << intrinsic_matrix_loaded;
	cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << endl;


	// 构建我们将要使用的无失真图
    // 后续帧。
    //
	Mat map1, map2;//最终的映射类型
	initUndistortRectifyMap(intrinsic_matrix_loaded, distortion_coeffs_loaded,
		Mat(), intrinsic_matrix_loaded, image_size,
		CV_16SC2, map1, map2);

	//相机运行显示未失真的图像。
	for (;;) {
		Mat image, image0;
		capture >> image0;

		if (image0.empty()) {
			break;
		}
		remap(image0, image, map1, map2, INTER_LINEAR,
			BORDER_CONSTANT, Scalar());
		imshow("Undistorted", image);
		if ((waitKey(30) & 255) == 27) {
			break;
		}
	}
	return 0;
}
