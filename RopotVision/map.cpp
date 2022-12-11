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
        Map(double hight , double width, double cellSize){
            this->width = width;
            this->hight = hight;
            this->cellSize = cellSize;
            map = cv::Mat(int(hight / cellSize), int(width / cellSize), CV_8U);
            frame = cv::Mat(int(hight * scale), int(width * scale), CV_8UC3, cv::Scalar(255, 255, 255));
        }
        void updatePostion(double x, double y){
            double factor = 1.0 / cellSize;
            drawPoint(cx, cy, COLOR_WHITE);
            // cx = (int) floor(x * factor);
            // cy = (int) floor(y * factor);
            cx = x;
            cy = y;
            drawPoint(cx, cy, COLOR_RED);            
            cv::imshow("map", this->frame);
        }
        virtual void drawMap(int thickness) = 0;
        // virtual void drawPosition(cv::Mat img) = 0;

    protected:
        cv::Mat map;            // map matrix
        cv::Mat frame;          // map matrix
        double width;              // width of the map in meter
        double hight;              // hight of the map in meter
        double cellSize;           // size of each cell of the matrix in meter
        int cx = 0;             // current x postion
        int cy = 0;             // current y postion
        const int padding = 20; // window padding
        const int scale = 30;   // map scale

        void drawPoint(double x, double y, int flag){
        uint8_t r = ((flag & 4) >> 2) * 255;
        uint8_t g = ((flag & 2) >> 1) * 255;
        uint8_t b = (flag & 1) * 255;
        // std::cout << "x : " << int (x) << " y = " << int(y) << "\n";
        cv::circle(this->frame, cv::Point2i(int(x * scale), int(y * scale)), 0, cv::Scalar(b, g, r), 15);
        }
};

class BasicMap: public Map{

    void drawMapBorders(int thickness, cv::Point2i &p0, cv::Point2i &p1, cv::Point2i &p2, cv::Point2i &p3) const {
        cv::line(frame, p0, p1, cv::Scalar(0, 0, 0), thickness);
        cv::line(frame, p1, p2, cv::Scalar(0, 0, 0), thickness);
        cv::line(frame, p2, p3, cv::Scalar(0, 0, 0), thickness);
        cv::line(frame, p3, p0, cv::Scalar(0, 0, 0), thickness);
    }

    public:
        BasicMap(double hight, double width, double cellSize) : Map(hight, width, cellSize){
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

        drawMapBorders(thickness, p0, p1, p2, p3);
        // this->this->frame = this->frame;
            cv::namedWindow("map");
            cv::imshow("map", this->frame)
            ;

        }
};