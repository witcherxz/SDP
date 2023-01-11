#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstdlib>

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int test(cv::Mat img);
int test2(cv::Mat img);
cv::Mat camera_matrix, dist_coeffs;

int main()
{
    cv::FileStorage fs("mono.yml", cv::FileStorage::READ);

    fs["K"] >> camera_matrix;
    fs["D"] >> dist_coeffs;

    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1280 ;
    int display_height = 720 ;
    int framerate = 30 ;
    int flip_method = 0 ;

    std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()) {
	std::cout<<"Failed to open camera."<<std::endl;
	return (-1);
    }

    cv::namedWindow("CSI Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat img;

    std::cout << "Hit ESC to exit" << "\n" ;
    while(true)
    {
    	if (!cap.read(img)) {
		std::cout<<"Capture read error"<<std::endl;
		break;
	    }
	
	//test(img);
    test2(img);
	//cv::imshow("CSI Camera",img);
	int keycode = cv::waitKey(10) & 0xff ; 
        if (keycode == 27) break ;
    }

    cap.release();
    cv::destroyAllWindows() ;
    return 0;
}

inline double RAD2DEG(double x) 
{
    return x*180.0/M_PI;
}

void rotationMatrixToEulerAngles(cv::Mat &R, double &roll, double &pitch, double &yaw)
{
    // Checks if a matrix is a valid rotation matrix.
    cv::Mat Rt = R.t();
    double n = cv::norm(cv::Mat::eye(3, 3, R.type()) - Rt*R, cv::NORM_L2);
    CV_Assert(n < 1e-6);

    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6 ? true : false;
    double x, y, z;
    if(~singular)
    {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    roll = RAD2DEG(x);
    pitch = RAD2DEG(y);
    yaw = RAD2DEG(z);

    return;
}

float xytheta[3] = {0,0,0};

int test2(cv::Mat image)
{
	int dictionaryId = 0; 
    float marker_length_m = 0.2;
    int wait_time = 10;

    cv::Mat image_copy;
    std::ostringstream vector_to_marker;    
    /*
    Here are the defined reference frames:

    TAG:
                    A y
                    |
                    |
                    |tag center
                    O---------> x

    CAMERA:


                    X--------> x
                    | frame center
                    |
                    |
                    V y
    */
    //--- 180 deg rotation matrix around the x axis (currently it is 0 deg)
    cv::Mat R_flip  = cv::Mat::zeros(3,3,CV_64F);
    R_flip.at<double>(0,0) = 1.0;
    R_flip.at<double>(1,1) = -1.0;
    R_flip.at<double>(2,2) = -1.0;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

		image.copyTo(image_copy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                    camera_matrix, dist_coeffs, rvecs, tvecs);
                    
            //-- array of rotation and position of each marker in camera frame
            //-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            //-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            //std::cout << "Translation: " << tvecs[0]
            //    << "\tRotation: " << rvecs[0] 
            //    << std::endl;

            //-- Obtain the rotation matrix tag->camera
            cv::Mat R_cam_tag;
            cv::Rodrigues(rvecs[0], R_cam_tag);
            cv::Mat R_tag_cam = R_cam_tag.t();

            //-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            double roll, yaw, pitch;
            cv::Mat R = R_flip*R_tag_cam;
            rotationMatrixToEulerAngles(R, roll, pitch, yaw);           

            //-- Now get Position and attitude of the camera respect to the marker
            cv::Mat tv = cv::Mat(tvecs[0]);
            cv::Mat pos_camera = -1.0*R_tag_cam*tv;
           // std::cout << "camera position (x, y, z): " << pos_camera.at<double>(0,0) << ", " << pos_camera.at<double>(1,0) << ", " << pos_camera.at<double>(2,0) << std::endl;
float alpha = 0.99;
xytheta[0] = xytheta[0]*alpha +pos_camera.at<double>(0,0)*(1-alpha);
xytheta[1] = xytheta[1]*alpha +pos_camera.at<double>(1,0)*(1-alpha);
xytheta[2] = xytheta[2]*alpha +yaw*(1-alpha);
printf("x-%.2f, y-%.2f, theta-%.2f\n",xytheta[0],xytheta[1],xytheta[2]); 

//std::cout << "(x, y, theta): " << xytheta[0] <<", "<< xytheta[1] <<", "<<xytheta[2]<<std::endl;
           // std::cout << "camera rotation (roll, pitch, yaw): " << roll << ", " << pitch << ", " << yaw << std::endl;
            // Draw axis for each marker
            for(int i=0; i < ids.size(); i++)
            {
                cv::drawFrameAxes(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);

                /*
                // This section is going to print the data for all the detected
                // markers. If you have more than a single marker, it is
                // recommended to change the below section so that either you
                // only print the data for a specific marker, or you print the
                // data for each marker separately.
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "x: " << std::setw(8) << tvecs[0](0);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "y: " << std::setw(8) << tvecs[0](1);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "z: " << std::setw(8) << tvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);
                */
            }
        }

        imshow("Pose estimation", image_copy);
}
