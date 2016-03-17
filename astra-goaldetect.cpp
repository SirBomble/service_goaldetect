#include <OpenNI.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


using namespace openni;

main()
{
    OpenNI::initialize();
    puts("Initializing orbbec");
    Device device;
    if (device.open(openni::ANY_DEVICE) != 0)
    {
        puts("Camera not detected!");
        return -1;
    }
    puts("Successfully opened Orbbec");
    VideoStream depth;
    depth.create(device, SENSOR_DEPTH);
    depth.start();
    puts("Successfully started depth sensor");
    VideoMode paramvideo;
    paramvideo.setResolution(640, 480);
    paramvideo.setFps(30);
    paramvideo.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
    depth.setVideoMode(paramvideo);
    device.setDepthColorSyncEnabled(false);

    // device.setDepthColorSyncEnabled(true);
    // device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

    puts("Initialization complete!");

    VideoStream** stream = new VideoStream*[1];
    stream[0] = &depth;

    if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
        VideoFrameRef depthFrame;
        cv::Mat depthmat;
        IplImage* depthcv = cvCreateImageHeader(cvSize(640, 480), IPL_DEPTH_16U, 1);
        cvNamedWindow("Depth");
        cvNamedWindow("Detected");
        int canny_thresh = 25;
        int erosion_size = 12;
        cv::Mat con_drawing = cv::Mat::zeros(depthmat.size(), CV_8UC3);
        cv::Scalar color;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        int changedIndex;
        while(device.isValid())
        {
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
                            cv::Size(2*erosion_size + 1, 2*erosion_size+1),
                            cv::Point(erosion_size, erosion_size));
                        cv::erode(depthmat, depthmat, element);
                        cv::blur(depthmat, depthmat, cv::Size(4, 4));
                        cv::Canny(depthmat, depthmat, canny_thresh, canny_thresh*2, 3);
                        cv::findContours(depthmat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

                        con_drawing = cv::Mat::zeros(depthmat.size(), CV_8UC3);
                        for(int i = 0; i < contours.size(); i++)
                        {
                            color = cv::Scalar(255, 255, 0);
                            cv::drawContours(con_drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                        }

                        cv::imshow("Depth", depthmat);
                        cv::imshow("Detected", con_drawing);
                    }
                    break;
                }

                default:
                    puts("Error retrieving stream");
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
