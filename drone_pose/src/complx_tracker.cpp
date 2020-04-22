//
// Created by ljy on 19-4-22.
//

#include <fstream>
#include "complx_tracker.h"

#define DEBUG_SHOW_IMG
#define OUTPUT_ARTICLE_IMAGE false

using namespace std;
using namespace cv;

pointsDetector::pointsDetector() {
    if (algo == "MOG2")
        pBackSub = createBackgroundSubtractorMOG2(500, 16, false); //500 400 false
    else if (algo == "KNN")
        pBackSub = createBackgroundSubtractorKNN(300, 400, false);
    else if (algo == "MOG")
        pBackSub = bgsegm::createBackgroundSubtractorMOG(500, 10, 0.7, 0);

    imgRec.push_back(Mat::zeros(img.size(),img.type()));
    imgRec.push_back(Mat::zeros(img.size(),img.type()));
    imgRec.push_back(Mat::zeros(img.size(),img.type()));
    imgRec.push_back(Mat::zeros(img.size(),img.type()));
    imgRec.push_back(Mat::zeros(img.size(),img.type()));

}


bool pointsDetector::updateFrame(const cv::Mat &frame, bool using_points_position_guess) {
    if (frame.empty()) {
        ROS_INFO("pointsDetector: Can't get frame.");
        return false;
    }
    img = frame;

    if(OUTPUT_ARTICLE_IMAGE)
    {
        imgRec[0] = img;
        imgRec[1] = valueImg;
    }

    if(!using_points_position_guess)
    {
        //! [apply]
        //update the background model
        pBackSub->apply(img, fgMask);
        //! [apply]
        frame_masked = Mat::zeros(frame.size(), frame.type());
        img.copyTo(frame_masked, fgMask);
        if(OUTPUT_ARTICLE_IMAGE) {
            imgRec[2] = frame_masked;
        }

    } else{


        frame_masked = Mat::zeros(frame.size(), frame.type());
        img.copyTo(frame_masked);
        if(OUTPUT_ARTICLE_IMAGE) {
            //!for imgRec
            //! [apply]
            //update the background model
            pBackSub->apply(img, fgMask);
            //! [apply]
            imgRec[2] = Mat::zeros(valueImg.size(), valueImg.type());
            valueImg.copyTo(imgRec[2], fgMask);
        }
    }
}


int pointsDetector::getPoints(const Mat &frame, cv::Point2f &BlueoutputPoint,cv::Point2f &GreenoutputPoint,
        cv::Point2f &RedoutputPoint,cv::Point2f &WhiteoutputPoint, bool using_points_position_guess) {
    //! the first three images used to initiate backgroundSubtraction
    if(count < 3)
    {
        if(!using_points_position_guess)
        {
            updateFrame(frame,false);
            count++;
        }
        return 1;
    }
    else
    {
        updateFrame(frame,false);
        ROS_INFO("find contours");
        ////opencv read image in B G R channel
        vector<Mat> channels;
        split(frame_masked,channels);
        BlueImg = channels.at(0);
        GreenImg = channels.at(1);
        RedImg = channels.at(2);

        threshold(BlueImg, BbinImg, 200, 255, THRESH_BINARY);
        threshold(GreenImg, GbinImg, 200, 255, THRESH_BINARY);
        threshold(RedImg, RbinImg, 200, 255, THRESH_BINARY);


        int structElementSize = 1;
        //!erode
        Mat erodestructElement = getStructuringElement(MORPH_RECT,
                                                       Size(3 * structElementSize + 1, 3 * structElementSize + 1));
        erode(BbinImg, BerodedImg, erodestructElement);
        erode(GbinImg, GerodedImg, erodestructElement);
        erode(RbinImg, RerodedImg, erodestructElement);
        ////dilate
        Mat dilatestructElement = getStructuringElement(MORPH_RECT,
                                                        Size(2 * structElementSize + 1, 2 * structElementSize + 1));
        dilate(BerodedImg, BdilatedImg, dilatestructElement);
        dilate(GerodedImg, GdilatedImg, dilatestructElement);
        dilate(RerodedImg, RdilatedImg, dilatestructElement);

        ////find contours
        findContours(BdilatedImg, contours_org_BW, RETR_CCOMP, CHAIN_APPROX_NONE);
        findContours(GdilatedImg, contours_org_GW, RETR_CCOMP, CHAIN_APPROX_NONE);
        findContours(RdilatedImg, contours_org_RW, RETR_CCOMP, CHAIN_APPROX_NONE);
        cout << "contours_org_BW.size()" << contours_org_BW.size() << endl;
        cout << "contours_org_GW.size()" << contours_org_GW.size() << endl;
        cout << "contours_org_RW.size()" << contours_org_RW.size() << endl;
        ////calculate center of blue, green, red and white

        ROS_INFO("calculate center of blue, green, red and white");

        BMomentCenters.clear();GMomentCenters.clear();RMomentCenters.clear();
        if(contours_org_BW.size() == 2 && contours_org_GW.size() == 2 && contours_org_RW.size() == 2)
        {   ROS_INFO("each contours has two element");
            for (auto itContours = contours_org_BW.begin(); itContours != contours_org_BW.end() ; itContours++) {
                Moments m;
                m = moments(*itContours,true);
                Point2f center(m.m10/m.m00,m.m01/m.m00);
                BMomentCenters.push_back(center);
            }
            for (auto itContours = contours_org_GW.begin(); itContours != contours_org_GW.end() ; itContours++) {
                Moments m;
                m = moments(*itContours,true);
                Point2f center(m.m10/m.m00,m.m01/m.m00);
                GMomentCenters.push_back(center);
            }
            for (auto itContours = contours_org_RW.begin(); itContours != contours_org_RW.end() ; itContours++) {
                Moments m;
                m = moments(*itContours,true);
                Point2f center(m.m10/m.m00,m.m01/m.m00);
                RMomentCenters.push_back(center);
            }
            ////select white marker by comparision
            for (int i=0; i < BMomentCenters.size() ; i++) {
                for (int j=0; j < GMomentCenters.size(); j++) {
                    float error = sqrt(pow((BMomentCenters[i].x-GMomentCenters[j].x),2)+pow((BMomentCenters[i].y-GMomentCenters[j].y),2));
                    float error_min = 10;//default error between white and white block
                    if(error < error_min)
                    {    ////figure out which is white marker and blue marker
                        if(i==0)
                        {
                            drone_0.white_p = BMomentCenters[0];
                            drone_0.blue_p = BMomentCenters[1];
                        }
                        if (i==1)
                        {
                            drone_0.white_p = BMomentCenters[1];
                            drone_0.blue_p = BMomentCenters[0];
                        }
                        //figure out green marker
                        if(j==0)
                            drone_0.green_p = GMomentCenters[1];
                        if(j==1)
                            drone_0.green_p = GMomentCenters[0];
                    }


                }
                for (int k = 0; k < RMomentCenters.size(); k++)
                {
                    float error = sqrt(pow((BMomentCenters[i].x-RMomentCenters[k].x),2)+pow((BMomentCenters[i].y-RMomentCenters[k].y),2));
                    float error_min = 5;//default error between white and white block
                    if(error < error_min)
                    {
                        if(k==0)
                            drone_0.red_p = RMomentCenters[1];
                        if(k==1)
                            drone_0.red_p = RMomentCenters[0];
                    }
                }
            }
            BlueoutputPoint=drone_0.blue_p;
            GreenoutputPoint=drone_0.green_p;
            RedoutputPoint=drone_0.red_p;
            WhiteoutputPoint=drone_0.white_p;


//            cv::destroyWindow("marker");
        }
        else
            //! detecting fail, re-initiate updateframe
        {
            count = 0;
            return 1;
        }

        return 2;
    }

}

bool pointsDetector::prefollowPoints(const Mat & frame)
{
    updateFrame(frame,true);
    ROS_INFO("pre follow Points stage");
    ////opencv read image in B G R channel
    vector<Mat> channels;
    split(frame_masked,channels);
    BlueImg = channels.at(0);
    GreenImg = channels.at(1);
    RedImg = channels.at(2);

}


bool pointsFollower::followPoints(const cv::Mat &frame, std::vector<cv::Point2f> &outputPointsVector) {
    binImg = frame;
    if (binImg.empty()) {
        ROS_INFO("pointsFollower: Can't get frame.");
        return false;
    }

    prevImgPts = outputPointsVector;
    nextImgPts.resize(prevImgPts.size());

    vector<uchar> status;
    vector<float> err;
//    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,50,0.001);
    TermCriteria termcrit(TermCriteria::EPS, 50, 0.001);
    Size winSize(15, 15);


    if (outputPointsVector.size() != 2) {
        swap(binImg, prevbinImg);
        ROS_INFO("pointsFollower: Number of Points to be followed is wrong.");
        return false;
    }

    if (prevbinImg.empty())
    {
        binImg.copyTo(prevbinImg);
        ROS_INFO("prevbinImg is empty ,so binImg copy to prebinImg");

    }


//    calcOpticalFlowPyrLK(prevGray, gray, prevImgPts, nextImgPts, status, err);
    calcOpticalFlowPyrLK(prevbinImg, binImg, prevImgPts, nextImgPts, status, err, winSize,
                         0, termcrit, 0, 0.001);
    ROS_INFO("complete opticalFlow");
    cout << prevImgPts[0] << "follow to " << nextImgPts[0] << endl;
    size_t i, k;
    for (i = k = 0; i < nextImgPts.size(); i++) {
        if (!status[i])
            continue;
        nextImgPts[k++] = nextImgPts[i];// according to status, only assign detected element into nextImgPts
//        circle( depth_img, nextImgPts[i], 2, Scalar(0,255,0), -1, 8);
    }
    if (prevImgPts.size() == nextImgPts.size()) {
        outputPointsVector = nextImgPts;
        swap(binImg, prevbinImg);
        swap(prevImgPts, nextImgPts);

        imgDronemarker = Mat::zeros(binImg.size(), CV_8UC3);
        circle(imgDronemarker,nextImgPts[0],10,Scalar(0,0,255),5);
        circle(imgDronemarker,nextImgPts[1],10,Scalar(255,255,255),5);

        cv::namedWindow("imgDronemarker",WINDOW_NORMAL);
        cv::resizeWindow("imgDronemarker",960,540);
        cv::imshow("imgDronemarker",imgDronemarker);
        cv::waitKey(50);
        ROS_INFO("show Pts marker");
        return true;
    } else {
        swap(binImg, prevbinImg);
        ROS_INFO("pointsFollower: failed.");
        return false;
    }
}

complx_tracker::complx_tracker() {
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    distCoeffs = std::vector<double>{0.0, -0.0, -0.0, 0.0, 0};
    cameraMatrix.at<double>(0, 0) = 1206.89;
    cameraMatrix.at<double>(0, 2) = 960;
    cameraMatrix.at<double>(1, 1) = 678.88;
    cameraMatrix.at<double>(1, 2) = 540;
    cameraMatrix.at<double>(2, 2) = 1;

//    start_time = ros::Time::now();

    if (STRUCTURE_INDEX == 0) {
        objectPts.emplace_back(0, 0.165, -0.05);
        objectPts.emplace_back(-0.165, 0, -0.05);
        objectPts.emplace_back(0.165, 0, -0.05);
        objectPts.emplace_back(0, -0.165, -0.05);
    } else if (STRUCTURE_INDEX == 1) {
        objectPts.emplace_back(0.01, 0.0925, 0);
        objectPts.emplace_back(0.01, -0.0925, 0);
        objectPts.emplace_back(0.01, -0.078, 0.088);
        objectPts.emplace_back(0.01, 0.078, 0.088);
    } else if (STRUCTURE_INDEX == 2) {
        objectPts.emplace_back(0, 0.098, 0 - 0.01);
        objectPts.emplace_back(0, -0.098, 0 - 0.01);
        objectPts.emplace_back(0, -0.076, 0.086 - 0.01);
        objectPts.emplace_back(0, 0.076, 0.086 - 0.01);
    }

    BlueimgPts = std::vector<cv::Point2f>(2);
    GreenimgPts = std::vector<cv::Point2f>(2);
    RedimgPts = std::vector<cv::Point2f>(2);

    std::string pkg_path = ros::package::getPath("drone_pose");


    Matx44d trans_cam;
    ifstream mtx;
    mtx.open(pkg_path + "/data/transformation_matrix.txt");
    if (mtx.is_open()) {
        mtx >> trans_cam.val[0] >> trans_cam.val[1] >> trans_cam.val[2] >> trans_cam.val[3]
            >> trans_cam.val[4] >> trans_cam.val[5] >> trans_cam.val[6] >> trans_cam.val[7]
            >> trans_cam.val[8] >> trans_cam.val[9] >> trans_cam.val[10] >> trans_cam.val[11]
            >> trans_cam.val[12] >> trans_cam.val[13] >> trans_cam.val[14] >> trans_cam.val[15];
        mtx.close();
    } else
        ROS_ERROR("Can't read matrix file.");
    camPose = Affine3d(trans_cam);
//
//    following_flag = false;
//    myWindow = viz::Viz3d("Coordinate Frame");
//    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
//    cube_widget1 = viz::WCube(Point3f(-0.0663, -0.03966, -0.01), Point3f(0.0663, 0, 0.01), false, viz::Color::blue());
//    cube_widget2 = viz::WCube(Point3f(-0.03135, 0, -0.01), Point3f(0.03135, 0.06132, 0.01), false, viz::Color::blue());
//    myWindow.showWidget("Cube Widget1", cube_widget1);
//    myWindow.showWidget("Cube Widget2", cube_widget2);
//
//    cv::viz::WLine line_widget(Point3d(0, 0, 3), Point3d(0, 0, 0), viz::Color::red());
//    myWindow.showWidget("line", line_widget);
//    line_widget.setRenderingProperty(viz::LINE_WIDTH, 1);

}

bool complx_tracker::findImgPts() {
//    if (!following_flag) {
        firstStage = pts_detector.getPoints(img, BimgPts,GimgPts,RimgPts,WimgPts);
        if (firstStage == 1){
            ROS_INFO("pts_detect 1st stage");
            return true;
        }
        else if (firstStage == 2)
        {   imgPts.clear();
            imgPts.push_back(BimgPts);
            imgPts.push_back(GimgPts);
            imgPts.push_back(RimgPts);
            imgPts.push_back(WimgPts);
            //!check imgPts have four useful values
            int i = 0;
            for (int j = 0; j < 4; ++j) {
                if(imgPts[j].x > 0 )
                    i++;
            }
            if(i==4)
                enablePnP = true;
            else
                enablePnP = false;
        }
//            following_flag = true;
//            BlueimgPts.clear(),GreenimgPts.clear(),RedimgPts.clear();
//            BlueimgPts.push_back(BimgPts),BlueimgPts.push_back(WimgPts);
//            GreenimgPts.push_back(GimgPts),GreenimgPts.push_back(WimgPts);
//            RedimgPts.push_back(RimgPts),RedimgPts.push_back(WimgPts);
//            cout << BlueimgPts[0] << BlueimgPts[1] << endl;
//            ROS_INFO("pts_detect 2nd stage");
//            return true;
//        }
//        else
//            return false;
//    }
//    else
//    {   //!update img to dilated img ,then take these images to follow
//        pts_detector.prefollowPoints(img);
//        //! show img to compare optical following
//        cv::namedWindow("img",WINDOW_NORMAL);
//        cv::resizeWindow("img",960,540);
//        cv::imshow("img",img);
//
//        following_flag = Blue_pts_follower.followPoints(pts_detector.BlueImg,BlueimgPts) &&
//                Green_pts_follower.followPoints(pts_detector.GreenImg,GreenimgPts) &&
//                Red_pts_follower.followPoints(pts_detector.RedImg,RedimgPts);
//        ROS_INFO("follow stage");
//        cout << BlueimgPts.size() << GreenimgPts[0] << RedimgPts[0] << BlueimgPts[1] << endl;
//        pts_detector.updateFrame(img,true);

//        for (int i = 0; i < 4; i++) {
//            int t = pts_detector.valueImg.at<uchar>(imgPts[i]);
//            if (t < 200) {
//                following_flag = pts_detector.getPoints(img, imgPts, true);

//                if(OUTPUT_ARTICLE_IMAGE) {
//                    std::string pkg_path = ros::package::getPath("drone_pnp");
//                    int rand_int = random() % 100;
//                    for (int saveImgIndex = 0; saveImgIndex < pts_detector.imgRec.size(); saveImgIndex++) {
//                        imwrite(pkg_path + "/image/" + to_string(rand_int) + to_string(saveImgIndex) + ".jpg",
//                                pts_detector.imgRec[saveImgIndex]);
//                    }
//                }
//
//                break;
//            }
//        }
//        if(!following_flag)
//            return findImgPts();
    return true;

//    }
}

bool complx_tracker::apply(const cv::Mat &frame, cv::Mat &img_to_show, cv::Affine3d &pose_world, cv::Affine3d &pose_raw) {
    undistort(frame, img, cameraMatrix, distCoeffs);
    ROS_INFO("apply!");
    followImgPtsSuccess = findImgPts();
    if(followImgPtsSuccess)
    {
        ROS_INFO("detect or follow Pts success");
    }
    else
        ROS_INFO("follow Pts fail");
//    waitKey(0);
    if(enablePnP){
    solvePnP(objectPts, imgPts, cameraMatrix, distCoeffs, outputRvecRaw, outputTvecRaw);
    pose = Affine3d(outputRvecRaw, outputTvecRaw);
    worldPose = pose.concatenate(camPose);
    pose_world = worldPose;
    pose_raw = pose;
    }
//
//    virtualize();
//    img_to_show = showImg;
    return true;
}


//void complx_tracker::virtualize() {
////    myWindow.setWidgetPose("Cube Widget1", worldPose);
////    myWindow.setWidgetPose("Cube Widget2", worldPose);
////    myWindow.spinOnce(1, true);
//    img.copyTo(showImg);
//    if (!pts_follower.nextImgPts.empty()) {
////        circle( showImg, pts_follower.nextImgPts[0], 1, Scalar(0,255,0), -1, 8);
////        circle( showImg, pts_follower.nextImgPts[1], 1, Scalar(255,255,0), -1, 8);
////        circle( showImg, pts_follower.nextImgPts[2], 1, Scalar(0,255,255), -1, 8);
////        circle( showImg, pts_follower.nextImgPts[3], 1, Scalar(255,0,255), -1, 8);
//
//    }
////    imshow("contours",pts_detector.contoursImg);
//    if (!pts_detector.sortedImgPts.empty()) {
////        circle( showImg, pts_detector.sortedImgPts[0], 3, Scalar(255,0,0), -1, 8);
////        circle( showImg, pts_detector.sortedImgPts[1], 3, Scalar(255,0,0), -1, 8);
////        circle( showImg, pts_detector.sortedImgPts[2], 3, Scalar(255,0,0), -1, 8);
////        circle( showImg, pts_detector.sortedImgPts[3], 3, Scalar(255,0,0), -1, 8);
//
//    }
//    if (!imgPts.empty()) {
////        circle( showImg, imgPts[0], 3, Scalar(0,255,0), -1, 8);
////        circle( showImg, imgPts[1], 3, Scalar(255,255,0), -1, 8);
////        circle( showImg, imgPts[2], 3, Scalar(0,255,255), -1, 8);
////        circle( showImg, imgPts[3], 3, Scalar(255,0,255), -1, 8);
//
//        putText(showImg, "0", imgPts[0], FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255));
//        putText(showImg, "1", imgPts[1], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
//        putText(showImg, "2", imgPts[2], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
//        putText(showImg, "3", imgPts[3], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
//    }
//
//    imshow("tracking", showImg);
//    waitKey(1);
//}





















