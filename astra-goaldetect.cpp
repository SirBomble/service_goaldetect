#include <OpenNI.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


using namespace openni;

main()
{
    OpenNI::initialize();
    puts( "Kinect initialization..." );
    Device device;
    if ( device.open( openni::ANY_DEVICE ) != 0 )
    {
        puts( "Kinect not found!" );
        return -1;
    }
    puts( "Kinect opened" );
    VideoStream depth, color;
    color.create( device, SENSOR_COLOR );
    color.start();
    puts( "Camera ok" );
    depth.create( device, SENSOR_DEPTH );
    depth.start();
    puts( "Depth sensor ok" );
    VideoMode paramvideo;
    paramvideo.setResolution( 640, 480 );
    paramvideo.setFps( 30 );
    paramvideo.setPixelFormat( PIXEL_FORMAT_DEPTH_100_UM );
    depth.setVideoMode( paramvideo );
    paramvideo.setPixelFormat( PIXEL_FORMAT_RGB888 );
    color.setVideoMode( paramvideo );
    puts( "Reglages des flux videos ok" );

    // If the depth/color synchronisation is not necessary, start is faster :
    device.setDepthColorSyncEnabled( false );

    // Otherwise, the streams can be synchronized with a reception in the order of our choice :
    //device.setDepthColorSyncEnabled( true );
    //device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );

    VideoStream** stream = new VideoStream*[2];
    stream[0] = &depth;
    stream[1] = &color;
    puts( "Kinect initialization completed" );


    if ( device.getSensorInfo( SENSOR_DEPTH ) != NULL )
    {
        VideoFrameRef depthFrame, colorFrame;
        IplImage* colorcv = cvCreateImageHeader( cvSize( 640, 480 ), IPL_DEPTH_8U, 3 );
        IplImage* depthcv = cvCreateImageHeader( cvSize( 640, 480 ), IPL_DEPTH_16U, 1 );
        cvNamedWindow("RGB");
        cvNamedWindow("Depth");

        int changedIndex;
        while( device.isValid() )
        {
            OpenNI::waitForAnyStream( stream, 2, &changedIndex );
            switch ( changedIndex )
            {
                case 0:
                    depth.readFrame( &depthFrame );

                    if ( depthFrame.isValid() )
                    {
                        depthcv->imageData = (char*) depthFrame.getData();
                        cvShowImage( "Depth", depthcv );
                    }
                    break;

                case 1:
                    color.readFrame( &colorFrame );

                    if ( colorFrame.isValid() )
                    {
                        colorcv->imageData = (char*) colorFrame.getData();
                        cvCvtColor( colorcv, colorcv, CV_BGR2RGB );
                        cvShowImage( "RGB", colorcv );
                    }
                    break;

                default:
                    puts( "Error retrieving a stream" );
            }
            cvWaitKey( 1 );
        }

        cvReleaseImageHeader( &colorcv );
        cvReleaseImageHeader( &depthcv );
        cvDestroyWindow( "RGB" );
        cvDestroyWindow( "Depth" );
    }
    depth.stop();
    depth.destroy();
    color.stop();
    color.destroy();
    device.close();
    OpenNI::shutdown();
}
