#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>

#define PORT "/dev/ttyACM0"
int fd;

int set_interface_attribs (int fd, int speed, int parity) {
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

int main() {
	std::vector<std::vector<cv::Point> > contours;
 	std::vector<cv::Vec4i> hierarchy;

	fd = open(PORT, O_RDWR | O_NOCTTY | O_SYNC);
	set_interface_attribs(fd, B115200, 0);		//If you get "error 9 from tcgetattr", that probably means the serial port wasn't opened
	set_blocking(fd, 0);

	// declare a VideoCapture object to associate webcam, 0 means use 1st (default) webcam
	//may need to use a different index
	cv::VideoCapture capWebcam(1);
	//  To check if object was associated to webcam successfully
	if (capWebcam.isOpened() == false)	 
	{				
		std::cout << "error: Webcam connect unsuccessful\n";
		return(0);
	}

	// Input image
	cv::Mat imgOriginal;		

	// HSV Image
	cv::Mat hsvImg;				

	// Thresh Image
	cv::Mat threshImg;			

	// 3 element vector of floats, this will be the pass by reference output of HoughCircles()
	std::vector<cv::Vec3f> v3fCircles;		

	char charCheckForEscKey = 0;

	// Set Hue
	int lowH = 25;							
	int highH = 30;

	// Set Saturation
	int lowS = 108;							
	int highS = 255;

	// Set Value
	int lowV = 172;							
	int highV = 255;

	// HUE for YELLOW is 21-30.
	// Adjust Saturation and Value depending on the lighting condition of the environment as well as the surface of the object.

	while (charCheckForEscKey != 27 && capWebcam.isOpened()) {				
		// get next frame
		bool frameReadSuccessfully = capWebcam.read(imgOriginal);		

		if (!frameReadSuccessfully || imgOriginal.empty()) {				
			std::cout << "error: frame can't read \n";
			break;
		}

		cv::rotate(imgOriginal, imgOriginal, cv::ROTATE_90_CLOCKWISE);

		// Convert Original Image to HSV Thresh Image
		cv::cvtColor(imgOriginal, hsvImg, cv::COLOR_BGR2HSV);						

		cv::inRange(hsvImg, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), threshImg);

		//Blur Effect
		cv::GaussianBlur(threshImg, threshImg, cv::Size(3, 3), 0);

		// Dilate Filter Effect
		cv::dilate(threshImg, threshImg, 3);

		// Erode Filter Effect
		cv::erode(threshImg, threshImg, 3);

		// fill circles vector with all circles in processed image
		// algorithm for detecting circles
		cv::HoughCircles(threshImg,v3fCircles,cv::HOUGH_GRADIENT,2,threshImg.rows / 4,100,50,10,800);
		contours.clear();
		hierarchy.clear();
		findContours(threshImg,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point(0,0));
		if(contours.size()) {
			cv::drawContours(imgOriginal,contours,-1,cv::Scalar(255,255,0));
			int largest_area = 0;
			int largest_contour_index = 0;
			for(int i = 0; i < contours.size(); i++) // iterate through each contour. 
			{
				double a = contourArea(contours[i], false);  //  Find the area of contour
				if(a > largest_area){
					largest_area = a;
					largest_contour_index = i;                //Store the index of largest contour
				}

			}
			auto bb = cv::boundingRect(contours[largest_contour_index]);
			int x = bb.x + bb.width / 2;
			int y = bb.y + bb.height / 2;
			
			//int x = bb.x;
			//int y = bb.y;


			int mx = threshImg.cols / 2;
			int my = threshImg.rows / 2;

			//x -= mx;
			//y -= my;
		
			cv::rectangle(threshImg, cv::Point(mx - 50, my - 50), cv::Point(mx + 50, my + 50), cv::Scalar(255, 255, 255), 3);	
			cv::rectangle(threshImg, cv::Point(x - 20, y - 20), cv::Point(x + 20, y + 20), cv::Scalar(100, 255, 100), 3);	

			//std::cout << x << " " << y << std::endl;
			if (x > mx + 50) {
				write(fd, "right\n", 6);
				//std::cout << "right ";
			} else if (x < mx - 50) {
				write(fd, "left\n", 5);
				//std::cout << "left ";
			} else {
				write(fd, "s_h\n", 4);
			}
			if (y > my + 50) {
				write(fd, "down\n", 5);
				//std::cout << " down" << std::endl;
			} else if (y < my - 50) {
				write(fd, "up\n", 3);
				//std::cout << " up" << std::endl;
			} else {
				write(fd, "s_v\n", 4);
			}
		}
		else {
			write(fd, "s_v\n", 4);
			write(fd, "s_h\n", 4);
		}

		// declare windows
		cv::namedWindow("imgOriginal", cv::WINDOW_AUTOSIZE);
		cv::namedWindow("threshImg", cv::WINDOW_AUTOSIZE);	

	    /* Create trackbars in "threshImg" window to adjust according to object and environment.*/
		cv::createTrackbar("LowH", "threshImg", &lowH, 179);	//Hue (0 - 179)
		cv::createTrackbar("HighH", "threshImg", &highH, 179);

		cv::createTrackbar("LowS", "threshImg", &lowS, 255);	//Saturation (0 - 255)
		cv::createTrackbar("HighS", "threshImg", &highS, 255);

		cv::createTrackbar("LowV", "threshImg", &lowV, 255);	//Value (0 - 255)
		cv::createTrackbar("HighV", "threshImg", &highV, 255);
		

		cv::imshow("imgOriginal", imgOriginal);					// show windows
		cv::imshow("threshImg", threshImg);
		cv::imshow("hsvImg", hsvImg);

		charCheckForEscKey = cv::waitKey(1);					// delay and get key press
	}
	write(fd, "s_h\n", 4);
	write(fd, "s_v\n", 4);
	
	return 0;											
}
