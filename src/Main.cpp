#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace cv;
using namespace std;

vector<Point2f> points;
void CallBackFunc(int event, int x, int y, int flags, void *userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        points.push_back(Point2f(x, y));
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        if (points.size() >= 4)
        {
            cout << "You have enought point, press any key to continue" << endl;
        }
        if (points.size() > 4)
        {
            points.erase(points.begin());
        }
    }
}

int main(int argc, char **argv)
{
    // DEBUG OPENCV VERSION
    std::cout << "OpenCV Version: " << CV_VERSION << std::endl;

    ////////////////
    //// PART 1 ////
    ////////////////

    // Get I0 from Images/I0.jpg
    Mat I0 = imread("./Images/I0.jpg", IMREAD_COLOR);
    Mat I0Original = I0.clone();
    if (I0.empty())
    {
        printf("Error opening image\n");
        return -1;
    }
    // display I0 in window
    namedWindow("I0", WINDOW_NORMAL);
    imshow("I0", I0);

    // I0 real size (I0 is a card)
    float cardWidth = 150;
    float cardHeight = 150;
    cout << "I0 size: " << I0.size() << endl;
    cout << "Card size: " << cardWidth << "x" << cardHeight << endl;

    ////////////////
    //// PART 2 ////
    ////////////////

    cout << "====================================" << endl;
    cout << "Click on the 4 corners of the card" << endl;

    // Start mouse callback to get corner points
    setMouseCallback("I0", CallBackFunc, NULL);

    // wait for 4 points
    while (points.size() < 4)
    {
        waitKey(0);
    }

    // get the last 4 points
    vector<Point2f> cardPoints;
    for (int i = 0; i < 4; i++)
    {
        cardPoints.push_back(points[points.size() - 4 + i]);
    }

    // DEBUG
    cout << "Card points: " << endl;
    for (int i = 0; i < 4; i++)
    {
        cout << cardPoints[i] << endl;
    }
    points.clear();

    // draw the points and lines
    for (int i = 0; i < 4; i++)
    {
        line(I0, cardPoints[i], cardPoints[(i + 1) % 4], Scalar(0, 0, 255), 5);
    }

    ////////////////
    //// PART 3 ////
    ////////////////

    // THIS IS THE CALIBRATION DATA GET WITH Calibration.cpp

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1801.007946952068, 0, 940.4847995845461,
                            0, 1818.038712323559, 546.9773425745414,
                            0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.09990003035182503, 1.434678429670576, -0.002847463971810034, -0.007174763721013124, -6.507474025550844);

    // END OF CALIBRATION DATA

    ////////////////
    //// PART 5 ////
    ////////////////

    // Create 3D points of the card
    // We assume that the card is on the ground plane (Z = 0), scene planaire
    // WARNING : Card size is in mm
    vector<Point3f> objectPoints;
    objectPoints.push_back(Point3f(0, 0, 0));                  // Top-left corner
    objectPoints.push_back(Point3f(cardWidth, 0, 0));          // Top-right corner
    objectPoints.push_back(Point3f(cardWidth, cardHeight, 0)); // Bottom-right corner
    objectPoints.push_back(Point3f(0, cardHeight, 0));         // Bottom-left corner

    // Camera pose estimation
    Mat rvec, tvec;
    solvePnP(objectPoints, cardPoints, cameraMatrix, distCoeffs, rvec, tvec); // solvePnP compute tout seul les points homogenes

    // Display the results
    cout << "Rotation vector (rvec):" << endl
         << rvec << endl;
    cout << "Translation vector (tvec):" << endl
         << tvec << endl;

    vector<Point3f> worldRepere;
    worldRepere.push_back(Point3f(cardWidth / 2, cardHeight / 2, 0));
    worldRepere.push_back(Point3f(cardWidth / 2, cardHeight / 2, -50));
    worldRepere.push_back(Point3f(cardWidth / 2 + 50, cardHeight / 2, 0));
    worldRepere.push_back(Point3f(cardWidth / 2, cardHeight / 2 + 50, 0));

    vector<Point2f> worldRepereProjectedPoints;
    projectPoints(worldRepere, rvec, tvec, cameraMatrix, distCoeffs, worldRepereProjectedPoints);

    arrowedLine(I0, worldRepereProjectedPoints[0], worldRepereProjectedPoints[1], Scalar(0, 255, 0), 5);
    arrowedLine(I0, worldRepereProjectedPoints[0], worldRepereProjectedPoints[2], Scalar(255, 0, 0), 5);
    arrowedLine(I0, worldRepereProjectedPoints[0], worldRepereProjectedPoints[3], Scalar(0, 0, 255), 5);

    imshow("I0", I0);

    cout << "====================================" << endl;
    cout << "Image with borders and world repere" << endl;
    cout << "Press any key to continue" << endl;

    waitKey(0);

    /* DEBUG PRINT CUBE ON CARD

    // Create 3D points of the cube
    vector<Point3f> cubePoints;
    cubePoints.push_back(Point3f(0, 0, 0));                   // Top-left-back corner
    cubePoints.push_back(Point3f(cardWidth, 0, 0));           // Top-right-back corner
    cubePoints.push_back(Point3f(cardWidth, cardHeight, 0));  // Bottom-right-back corner
    cubePoints.push_back(Point3f(0, cardHeight, 0));          // Bottom-left-back corner
    cubePoints.push_back(Point3f(0, 0, 10));                  // Top-left-front corner
    cubePoints.push_back(Point3f(cardWidth, 0, 10));          // Top-right-front corner
    cubePoints.push_back(Point3f(cardWidth, cardHeight, 10)); // Bottom-right-front corner
    cubePoints.push_back(Point3f(0, cardHeight, 10));         // Bottom-left-front corner

    // Project 3D cube points to 2D image coordinates
    vector<Point2f> cubeProjectedPoints;
    projectPoints(cubePoints, rvec, tvec, cameraMatrix, distCoeffs, cubeProjectedPoints);

    // Draw lines of the cube
    for (int i = 0; i < 4; i++)
    {
        line(I0, cubeProjectedPoints[i], cubeProjectedPoints[(i + 1) % 4], Scalar(0, 255, 0), 10);
        line(I0, cubeProjectedPoints[i + 4], cubeProjectedPoints[((i + 1) % 4) + 4], Scalar(0, 255, 0), 10);
        line(I0, cubeProjectedPoints[i], cubeProjectedPoints[i + 4], Scalar(0, 255, 0), 10);
    }

    // Display the image with the cube
    namedWindow("Cube Visualization", WINDOW_NORMAL);
    imshow("Cube Visualization", I0);
    waitKey(0);
    */

    ////////////////
    //// PART 6 ////
    ////////////////

    cout << "====================================" << endl;
    cout << "Computing SIFT keypoints..." << endl;

    // Convert the image to grayscale
    Mat I0Gray;
    cvtColor(I0Original, I0Gray, COLOR_BGR2GRAY);

    // Detect SIFT keypoints and descriptors
    Ptr<SIFT> sift = SIFT::create();
    vector<KeyPoint> keypoints_original;
    Mat descriptorOriginal;
    sift->detectAndCompute(I0Gray, noArray(), keypoints_original, descriptorOriginal);

    // Remove keypoints and descriptors outside the card
    vector<KeyPoint> keypointsOriginalInsidePolygon;
    Mat descriptorOriginalInsidePolygon;
    for (int i = 0; i < keypoints_original.size(); i++)
    {
        if (pointPolygonTest(cardPoints, keypoints_original[i].pt, false) >= 0)
        {
            keypointsOriginalInsidePolygon.push_back(keypoints_original[i]);
            descriptorOriginalInsidePolygon.push_back(descriptorOriginal.row(i));
        }
    }

    cout << "Number of keypoints: " << keypoints_original.size() << endl;
    Mat I0WithKeypoints;

    // Draw the keypoints on the image
    drawKeypoints(I0Original, keypointsOriginalInsidePolygon, I0WithKeypoints, Scalar(0, 0, 255));

    // Display the image with SIFT keypoints
    namedWindow("SIFT Keypoints", WINDOW_NORMAL);
    imshow("SIFT Keypoints", I0WithKeypoints);

    cout << "====================================" << endl;
    cout << "Image with SIFT keypoints inside the card" << endl;
    cout << "Press any key to continue" << endl;
    waitKey(0);

    ////////////////
    //// PART 8 ////
    ////////////////

    cout << "====================================" << endl;
    cout << "Starting video processing..." << endl;

    // get all image in Images/sequence folder
    vector<String> filenames;
    glob("./Images/sequence/*.jpg", filenames);

    // for each image, create Mat
    vector<Mat> images;
    for (size_t i = 0; i < filenames.size(); ++i)
    {
        images.push_back(imread(filenames[i]));
    }

    ////////////////
    //// PART 9 ////
    ////////////////

    vector<Point2f> oldSceneCorners;
    vector<float> SceneDistances;
    float seuil = 300;
    bool correction = true;

    // for each image, detect keypoints and descriptors
    for (Mat &IScene : images)
    {
        // Convert the image to grayscale
        Mat ISceneGray;
        cvtColor(IScene, ISceneGray, COLOR_BGR2GRAY);

        cout << "Processing image... [" << (&IScene - &images[0]) * 100 / images.size() << "%]"
             << "[ " << (&IScene - &images[0]) << "/" << images.size() << " ]" << endl;

        vector<KeyPoint> keypoints_scene;
        Mat descriptorScene;
        sift->detectAndCompute(ISceneGray, noArray(), keypoints_scene, descriptorScene);

        // Match 2nn descriptors
        FlannBasedMatcher matcher;
        vector<vector<DMatch>> matches;
        matcher.knnMatch(descriptorOriginalInsidePolygon, descriptorScene, matches, 2);

        // fill the vector of points with matches
        vector<Point2f> pointsOriginal;
        vector<Point2f> pointsScene;
        for (int i = 0; i < matches.size(); i++)
        {
            if (matches[i][0].distance < 0.75 * matches[i][1].distance)
            {
                pointsOriginal.push_back(keypointsOriginalInsidePolygon[matches[i][0].queryIdx].pt);
                pointsScene.push_back(keypoints_scene[matches[i][0].trainIdx].pt);
            }
        }

        // compute homography
        Mat H = findHomography(pointsOriginal, pointsScene, RANSAC);

        /*
        cout << "Homography matrix: " << endl
             << H << endl;
        */

        // calculate the corners of the object in the scene
        vector<Point2f> sceneCorners;

        perspectiveTransform(cardPoints, sceneCorners, H);

        // check distance between corners
        if ((&IScene - &images[0]) > 0 && correction)
        {
            float distance = 0;
            for (int i = 0; i < 4; i++)
            {
                distance += norm(sceneCorners[i] - oldSceneCorners[i]);
            }
            // cout << "Distance between corners: " << distance << endl;
            if (distance > seuil)
            {
                cout << "Error detected, trying to correct" << endl;
                sceneCorners = oldSceneCorners;
            }
            else
            {
                // save scene corners for next iteration
                oldSceneCorners = sceneCorners;
            }
            // save distance in vector
            SceneDistances.push_back(distance);
        }
        else
        {
            // save scene corners for next iteration
            oldSceneCorners = sceneCorners;
        }

        // draw a contour around the object in the scene
        line(IScene, sceneCorners[0], sceneCorners[1], Scalar(255, 0, 255), 5);
        line(IScene, sceneCorners[1], sceneCorners[2], Scalar(255, 0, 255), 5);
        line(IScene, sceneCorners[2], sceneCorners[3], Scalar(255, 0, 255), 5);
        line(IScene, sceneCorners[3], sceneCorners[0], Scalar(255, 0, 255), 5);

        /// Extract 3D-2D point correspondences for solvePnP
        vector<Point3f> objectPoints3D;
        objectPoints3D.push_back(Point3f(0, 0, 0));                  // Top-left corner
        objectPoints3D.push_back(Point3f(cardWidth, 0, 0));          // Top-right corner
        objectPoints3D.push_back(Point3f(cardWidth, cardHeight, 0)); // Bottom-right corner
        objectPoints3D.push_back(Point3f(0, cardHeight, 0));         // Bottom-left corner

        ////////////////
        //// PART 10 ///
        ////////////////

        vector<Point2f> sceneCorners2D;

        for (int i = 0; i < 4; ++i)
        {
            sceneCorners2D.push_back(sceneCorners[i]);
        }

        Mat rvec;
        Mat tvec;

        // Camera pose estimation using solvePnP
        solvePnP(objectPoints3D, sceneCorners2D, cameraMatrix, distCoeffs, rvec, tvec);

        /* Display the results
        cout << "Rotation vector (rvec):" << endl
             << rvec << endl;
        cout << "Translation vector (tvec):" << endl
             << tvec << endl;
        */

        ////////////////
        //// PART 11 ///
        ////////////////

        vector<Point3f> worldRepere;
        worldRepere.push_back(Point3f(cardWidth / 2, cardHeight / 2, 0));
        worldRepere.push_back(Point3f(cardWidth / 2, cardHeight / 2, -50));
        worldRepere.push_back(Point3f(cardWidth / 2 + 50, cardHeight / 2, 0));
        worldRepere.push_back(Point3f(cardWidth / 2, cardHeight / 2 + 50, 0));

        // compute world repere projected points
        vector<Point2f> worldRepereProjectedPoints;
        projectPoints(worldRepere, rvec, tvec, cameraMatrix, distCoeffs, worldRepereProjectedPoints);

        // Draw arrows for the coordinate axis
        arrowedLine(IScene, worldRepereProjectedPoints[0], worldRepereProjectedPoints[2], Scalar(255, 0, 0), 5);
        arrowedLine(IScene, worldRepereProjectedPoints[0], worldRepereProjectedPoints[3], Scalar(0, 0, 255), 5);
        arrowedLine(IScene, worldRepereProjectedPoints[0], worldRepereProjectedPoints[1], Scalar(0, 255, 0), 5);

        // Save the scene image with bounding box
        stringstream filenameStream;
        filenameStream << "./Images/res/frames/" << setw(4) << setfill('0') << (&IScene - &images[0]) << ".jpg";
        imwrite(filenameStream.str(), IScene);
    }

    cout << "====================================" << endl;
    cout << "Video processing done" << endl;
    cout << "Frames saved in ./Images/res/frames" << endl;
    cout << "Saving debug data..." << endl;

    // write distances.txt
    ofstream distancesFile;
    distancesFile.open("./Images/res/distances.txt");
    for (int i = 0; i < SceneDistances.size(); i++)
    {
        distancesFile << SceneDistances[i] << endl;
    }

    ////////////////
    //// PART 13 ///
    ////////////////

    cout << "====================================" << endl;
    cout << "To create the video, run the following command:" << endl;
    cout << "ffmpeg -framerate 25 -pattern_type glob -i './Images/res/frames/*.jpg' -c:v libx264 -r 25 ./Images/res/out.mp4" << endl;

    // ffmpeg -framerate 25 -pattern_type glob -i '*.jpg' -c:v libx264 -r 25 out.mp4

    return 0;
}
