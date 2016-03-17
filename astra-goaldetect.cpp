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
        int canny_thresh = 100;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        int changedIndex;
        while(device.isValid())
        {
            OpenNI::waitForAnyStream(stream, 1, &changedIndex);
            switch (changedIndex)
            {
                case 0:
                    depth.readFrame(&depthFrame);

                    if (depthFrame.isValid())
                    {
                        depthcv->imageData = (char*) depthFrame.getData();
                        depthmat = cv::Mat(depthcv);
                        cv::blur(depthmat, depthmat, cv::Size(3, 3));
                        //cv::Canny(depthmat, depthmat, canny_thresh, canny_thresh*2, 3);
                        //cv::findContours(depthmat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
                        cv::flip(depthmat, depthmat, 1);
                        cv::imshow("Depth", depthmat);
                    }
                    break;

                default:
                    puts("Error retrieving stream");
                    break;
            }
            cvWaitKey(1);
        }
        cvReleaseImageHeader(&depthcv);
        cvDestroyWindow("Depth");
    }
    depth.stop();
    depth.destroy();
    device.close();
    OpenNI::shutdown();
}
