//
// Created by wzy on 5/2/20.
//
#include "opencv2/opencv.hpp"
#include "opencv2/viz.hpp"
using namespace cv;
using namespace std;
int main(int argc, char *argv[])
{
    viz::Viz3d myWindow("Coordinate Frame");
//    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
    viz::WCube cube_widget1(Point3f(-0.0663, -0.03966, -0.01), Point3f(0.0663, 0, 0.01), false, viz::Color::blue());
    viz::WCube cube_widget2(Point3f(-0.03135, 0, -0.01), Point3f(0.03135, 0.06132, 0.01), false, viz::Color::blue());
    myWindow.showWidget("Cube Widget1", cube_widget1);
    myWindow.showWidget("Cube Widget2", cube_widget2);

    cv::viz::WLine line_widget(Point3d(0, 0, 3), Point3d(0, 0, 0), viz::Color::red());
    myWindow.showWidget("line", line_widget);
    line_widget.setRenderingProperty(viz::LINE_WIDTH, 1);
    while (!myWindow.wasStopped())
    {
        myWindow.spinOnce(1, false);
    }
    return 0;
}

