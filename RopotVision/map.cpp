#include <iostream>
#include <math.h>
#include <cstdint>
#include <opencv2\core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define COLOR_WHITE 7
#define COLOR_RED 4
#define COLOR_BLUE 1
#define COLOR_GREEN 2
class Map {
    public:    
        Map(int hight , int width, int cellSize){
            this->width = width;
            this->hight = hight;
            this->cellSize = cellSize;
            map = cv::Mat(hight / cellSize, width / cellSize, CV_8U);
            frame = cv::Mat(hight * scale, width * scale, CV_8UC3, cv::Scalar(255, 255, 255));
        }
        void updatePostion(double x, double y){
            float factor = 1.0f / ((float) cellSize);
            drawPoint(cx, cy, COLOR_WHITE);
            cx = (int) floor(x * factor);
            cy = (int) floor(y * factor);
            drawPoint(cx, cy, COLOR_RED);            
            cv::imshow("test", this->frame);
        }
        virtual void drawMap(int thickness) = 0;
        // virtual void drawPosition(cv::Mat img) = 0;

    protected:
        cv::Mat map;            // map matrix
        cv::Mat frame;          // map matrix
        int width;              // width of the map in meter
        int hight;              // hight of the map in meter
        int cellSize;           // size of each cell of the matrix in meter
        int cx = 0;             // current x postion
        int cy = 0;             // current y postion
        const int padding = 20; // window padding
        const int scale = 50;   // map scale

        void drawPoint(int x, int y, int flag){
        uint8_t r = ((flag & 4) >> 2) * 255;
        uint8_t g = ((flag & 2) >> 1) * 255;
        uint8_t b = (flag & 1) * 255;
        cv::circle(this->frame, cv::Point2i(x * scale, y * scale), 0, cv::Scalar(b, g, r), 15);
        }
};

class BasicMap: public Map{
    
    public:
        BasicMap(int hight, int width, int cellSize) : Map(hight, width, cellSize){
            // std::cout << "Map is created";
        }
        void drawMap(int thickness){
            // std::cout << "Enter Draw Map \n";
            int h = hight * scale;
            int w = width * scale;
            cv::Mat frame(h, w, CV_8UC3, cv::Scalar(255, 255, 255));
            // Anticlockwise corner starting from top left
            cv::Point2i p0 = cv::Point2i(padding, padding);
            cv::Point2i p1 = cv::Point2i(padding, h - padding);
            cv::Point2i p2 = cv::Point2i( w - padding, h - padding);
            cv::Point2i p3 = cv::Point2i( w - padding, padding);
            
            cv::line(this->frame, p0, p1, cv::Scalar(0, 0,0), thickness);
            cv::line(this->frame, p1, p2, cv::Scalar(0, 0,0), thickness);
            cv::line(this->frame, p2, p3, cv::Scalar(0, 0,0), thickness);
            cv::line(this->frame, p3, p0, cv::Scalar(0, 0,0), thickness);
            // this->this->frame = this->frame;
            cv::namedWindow("test");
            cv::imshow("test", this->frame)
            ;

        }
};