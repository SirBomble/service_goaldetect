#include <OpenNI.h>
//#include <pthread.h>
#include <signal.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "smootour.h"

using namespace openni;
int print_ping = 0;

void
do_ping(int signal)
{
    print_ping = 1;
}

void
keepalive_message(void)
{
    static int ping_num = 0;
    printf("{type:\"ping\",value:%d}\n", ping_num);
    fflush(stdout);
    ++ping_num;
}

void
json_log(const char* str)
{
    printf("{type:\"log\",string:\"%s\"}\n", str);
    fflush(stdout);
}

int
main()
{
    signal(SIGALRM, do_ping);
    alarm(5);

    OpenNI::initialize();
    json_log("Initializing orbbec");
    Device device;
    if (device.open(openni::ANY_DEVICE) != 0)
    {
        json_log("Camera not detected!");
        return -1;
    }
    json_log("Successfully opened Orbbec");
    VideoStream depth;
    depth.create(device, SENSOR_DEPTH);
    depth.start();
    json_log("Successfully started depth sensor");
    VideoMode paramvideo;
    paramvideo.setResolution(640, 480);
    paramvideo.setFps(30);
    paramvideo.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
    depth.setVideoMode(paramvideo);
    device.setDepthColorSyncEnabled(false);

    // device.setDepthColorSyncEnabled(true);
    // device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

    json_log("Initialization complete!");

    VideoStream** stream = new VideoStream*[1];
    stream[0] = &depth;

    if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
        VideoFrameRef depthFrame;
        cv::Mat depthmat;
        cv::Mat template_mat;
        IplImage* depthcv = cvCreateImageHeader(cvSize(640, 480), IPL_DEPTH_16U, 1);
        cvNamedWindow("Depth");
        cvNamedWindow("Template");
        cvNamedWindow("Detected");
        int canny_thresh = 150;
        int erosion_dia_size = 4;
        cv::Mat con_drawing = cv::Mat::zeros(depthmat.size(), CV_8UC3);
        cv::Mat template_con_drawing;
        cv::Scalar color;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<cv::Point> > template_contours;
        std::vector<cv::Vec4i> template_hierarchy;
        std::vector<double> potential_matches;
        // int potential_match_index;
        // Smootour smootour(640, 480);

        // Create a contour of the template image
        template_mat = cv::imread("GoalContourTemplate.png", CV_LOAD_IMAGE_GRAYSCALE);
        cv::Canny(template_mat, template_mat, canny_thresh, canny_thresh*2, 3);
        cv::findContours(template_mat, template_contours, template_hierarchy,
            CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1, cv::Point(0, 0));
        template_con_drawing = cv::Mat::zeros(template_mat.size(), CV_8UC3);
        for(int i = 0; i < template_contours.size(); i++)
        {
            color = cv::Scalar(255, 255, 0);
            cv::drawContours(template_con_drawing, template_contours, i, color, 2, 8, template_hierarchy, 0, cv::Point());
        }
        cv::imshow("Template", template_con_drawing);

        int changedIndex;
        while(device.isValid())
        {
            if (print_ping)
            {
                keepalive_message();
                print_ping = 0;
                alarm(5);
            }

            OpenNI::waitForAnyStream(stream, 1, &changedIndex);
            switch (changedIndex)
            {
                case 0:
                {
                    depth.readFrame(&depthFrame);

                    if (depthFrame.isValid())
                    {
                        depthcv->imageData = (char*) depthFrame.getData();
                        depthmat = cv::Mat(depthcv);
                        depthmat.convertTo(depthmat, CV_8U, 0.00390625, -20);
                        cv::flip(depthmat, depthmat, 1);
                        cv::Mat element = getStructuringElement(0,
                            cv::Size(2*erosion_dia_size + 1, 2*erosion_dia_size+1),
                            cv::Point(erosion_dia_size, erosion_dia_size));
                        cv::erode(depthmat, depthmat, element);
                        cv::dilate(depthmat, depthmat, element);
                        cv::blur(depthmat, depthmat, cv::Size(4, 4));
                        cv::Canny(depthmat, depthmat, canny_thresh, canny_thresh*2, 3);
                        cv::findContours(depthmat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1, cv::Point(0, 0));
                        // smootour.update(depthmat);
                        // contours = smootour.get_contours();

                        std::vector<cv::Point2f> contour_center(contours.size());
                        std::vector<float> contour_radius(contours.size());

                        for (int i = 0; i < contours.size(); i++)
                        {
                            cv::approxPolyDP(cv::Mat(contours[i]), contours[i], 2, true);
                            cv::minEnclosingCircle((cv::Mat)contours[i], contour_center[i], contour_radius[i]);
                        }

                        // Compare the contours to the template contour
                        potential_matches.clear();
                        for (int i = 0; i < contours.size(); i++)
                        {
                            potential_matches.push_back(cv::matchShapes(contours.at(i), template_contours.at(0), CV_CONTOURS_MATCH_I1, 0.0));
                            printf("Got with acc = %f\n", potential_matches.at(i));
                        }

                        con_drawing = cv::Mat::zeros(depthmat.size(), CV_8UC3);
                        for(int i = 0; i < contours.size(); i++)
                        {
                            color = cv::Scalar(255, 255, 0);
                            cv::drawContours(con_drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                            // cv::circle(con_drawing, contour_center[i], (int)contour_radius[i], color, 2, 8, 0);
                        }

                        cv::imshow("Depth", depthmat);
                        cv::imshow("Detected", con_drawing);
                    }
                    break;
                }

                default:
                    json_log("Error retrieving stream");
                    break;
            }
            cvWaitKey(1);
        }
        cvReleaseImageHeader(&depthcv);
        cvDestroyWindow("Depth");
        cvDestroyWindow("Detected");
    }
    depth.stop();
    depth.destroy();
    device.close();
    OpenNI::shutdown();
}
