#include <opencv2\core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define WHITE cv::Scalar(255,255,255)
#define RED cv::Scalar(255,0,0)
#define BLACK cv::Scalar(0,0,0)
class Map {
    public:    
        Map(double height , double width, double cellSize){
            this->width = width;
            this->height = height;
            this->cellSize = cellSize;
            map = cv::Mat(int(height / cellSize), int(width / cellSize), CV_8U);
            frame = cv::Mat(int(height * MapScale), int(width * MapScale), CV_8UC3, cv::Scalar(255, 255, 255));
        }

        void changePointPosition(int x, int y){
            drawPoint(cx, cy, WHITE);
            changeCoordinates(x, y);
            drawPoint(cx, cy, RED);
            cv::imshow("map", this->frame);
        }

        virtual void drawMap(int thickness) = 0;

    protected:
        cv::Mat map;
        cv::Mat frame;
        double width;
        double height;
        double cellSize;
        int cx = 0;             // current x position
        int cy = 0;             // current y position
        const int borderPadding = 20;
        const int MapScale = 30;

        void drawPoint(double x, double y, const cv::Scalar& color){
        cv::circle(this->frame, cv::Point2i(int(x * MapScale), int(y * MapScale)), 0, color, 15);
        }

        void changeCoordinates(int x, int y) {
            cx = x;
            cy = y;
        }
};

class BasicMap: public Map{

    void drawMapBorders(int thickness, cv::Point2i &p0, cv::Point2i &p1, cv::Point2i &p2, cv::Point2i &p3) const {
        cv::line(frame, p0, p1, BLACK, thickness);
        cv::line(frame, p1, p2, BLACK, thickness);
        cv::line(frame, p2, p3, BLACK, thickness);
        cv::line(frame, p3, p0, BLACK, thickness);
    }


    void initializeCornerPoints(int h, int w, cv::Point2i &p0, cv::Point2i &p1, cv::Point2i &p2, cv::Point2i &p3) const {
        int p = borderPadding;
        p0= cv::Point2i(p, p);
        p1= cv::Point2i(p, h - p);
        p2= cv::Point2i(w - p, h - p);
        p3= cv::Point2i(w - p, p);
    }

    void createMapWindow() const {
        cv::namedWindow("map");
        cv::imshow("map", frame);
    }

    public:
        BasicMap(double height, double width, double cellSize) : Map(height, width, cellSize){}

        void drawMap(int thickness=2){
            int windowHeight = height * MapScale;
            int windowWidth = width * MapScale;
            cv::Mat frame(windowHeight, windowWidth, CV_8UC3, cv::Scalar(255, 255, 255));
            cv::Point2i p0, p1, p2, p3;
            initializeCornerPoints(windowHeight, windowWidth, p0, p1, p2, p3);
            drawMapBorders(thickness, p0, p1, p2, p3);
            createMapWindow();
        }
};