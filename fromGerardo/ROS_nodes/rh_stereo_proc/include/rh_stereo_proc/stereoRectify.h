

#ifndef stereoRectify_h
#define stereoRectify_h

class CRectification
{

public:
	
	float scale;
	Mat R1, P1, R2, P2, Q;
	
    //Functions
    void rectifyImgs(Mat img1, Mat img2, Mat K1, Mat K2, Mat D1, Mat D2, Mat R, Mat T, Mat& outImg1, Mat& outImg2, Rect& outroi1, Rect& outroi2)
	{
		ROS_INFO("Rectifying images!");
		Size img_size = img1.size();
        Rect roi1, roi2;
        stereoRectify(K1, D1, K2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
	
		ROS_INFO("Disparity-to-Depth matrix");
		printMatrix(Q, false);
	
		Mat map11, map12, map21, map22;
		initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
		initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		Mat img1r, img2r;
		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);
		
		outImg1 = img1r.clone();
		outImg2 = img2r.clone();

        outroi1 = roi1;
        outroi2 = roi2;

		ROS_INFO("Done!");
	}
	
	void rectifyMatrices(Mat img1, Mat img2, Mat K1, Mat K2, Mat D1, Mat D2, Mat R, Mat T)
	{
		ROS_INFO("Rectifying Matrices!");
		Size img_size = img1.size();
		stereoRectify(K1, D1, K2, D2, img_size, R, T, R1, R2, P1, P2, Q, 0);// CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
	
		ROS_INFO("Disparity-to-Depth matrix:");
		printMatrix(Q, false);
	
		// Publish Images
		ROS_INFO("Done!");

	}
	
	void printMatrix(Mat M, bool printType)
    {
		if(printType)
			ROS_INFO_STREAM("Matrix type:" << M.type());
		// dont print empty matrices
		if (M.empty()){
			ROS_INFO("---");
			return;
		}
		// loop through columns and rows of the matrix
		for(int i=0; i < M.rows; i++){
			for(int j=0; j < M.cols ; j++){
				if(M.type() == 6)
					cout << M.at<double>(i,j) << "\t";
				else
			    	cout << M.at<float>(i,j) << "\t";
			}
			cout<<endl;
		}
		cout<<endl;
	}

private:

    //Functions

};

#endif
