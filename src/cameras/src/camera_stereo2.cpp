
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <cameras/CameraStereoConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_listener.h>
#include <boost/thread/mutex.hpp>
#include <string>


class CameraDriver
{
public:

    typedef cameras::CameraStereoConfig Config;
    typedef camera_info_manager::CameraInfoManager CameraInfoManager;

    enum
    {
        LEFT        = 0,
        RIGHT       = 1,
        NUM_CAMERAS
    };

    static const int DEFAULT_CAMERA_INDEX;  // 设备索引只有一个
    static const double DEFAULT_FPS;
    static const char* DEFAULT_CAMERA_NAME;

    static const std::string CameraString[NUM_CAMERAS];

	CameraDriver()
	:
		nh( "~" ),
        camera_nh( "stereo" ),
        it( nh ),
        server( nh ),
		reconfiguring( false )
	{
        camera.release();
        nh.param<int>( "camera_index", camera_index, DEFAULT_CAMERA_INDEX );

        if( camera.open( camera_index ) )
        {
            ROS_INFO_STREAM( "camera (index = " << camera_index << ") successfully opened!" );
        }
        else
        {
            ROS_ERROR_STREAM( "Failed to open " << " camera (index = " << camera_index << ")!" );
            ros::shutdown();
        }

		nh.param<std::string>( "camera_name", camera_name, DEFAULT_CAMERA_NAME );
		nh.param<double>( "fps", fps, DEFAULT_FPS );
        
        ROS_INFO_STREAM( "set name and fps" );
        
		for( size_t i = 0; i < NUM_CAMERAS; ++i )
        {
            single_camera_nh[i] = ros::NodeHandle( camera_nh, CameraString[i] );

            camera_info_manager[i] = boost::make_shared< CameraInfoManager >( single_camera_nh[i] );
            setCameraName( *camera_info_manager[i], camera_name + "_" + CameraString[i] );

		    frame[i] = boost::make_shared< cv_bridge::CvImage >();
		    frame[i]->encoding  = sensor_msgs::image_encodings::BGR8;

            camera_pub[i] = it.advertiseCamera( CameraString[i] + "/image_raw", 1 );
        }
        
        ROS_INFO_STREAM( "init single cam nodehandle" );

		camera_info = boost::make_shared< sensor_msgs::CameraInfo >();
        ROS_INFO_STREAM( "make_shared CameraInfo" );
        // boost::shared_ptr<CameraDriver> self(this);
        server_callback = boost::bind( &CameraDriver::reconfig, this, _1, _2 );
        ROS_INFO_STREAM( "bind function" );
		server.setCallback( server_callback );
        ROS_INFO_STREAM( "setCallback" );

		timer = nh.createTimer( ros::Duration( 1. / fps ), &CameraDriver::capture, this );
        ROS_INFO_STREAM( "init succeed" );
	}

	~CameraDriver()
	{
        camera.release();
	}

	void reconfig( Config& newconfig, uint32_t level )
	{
		reconfiguring = true;
		boost::mutex::scoped_lock lock( mutex );

        // Resolve frame ID using tf_prefix parameter:
        if( newconfig.frame_id == "" )
        {
            newconfig.frame_id = "camera";
        }
        std::string tf_prefix = tf::getPrefixParam( nh );
        ROS_DEBUG_STREAM( "tf_prefix = " << tf_prefix );
        newconfig.frame_id = tf::resolve( tf_prefix, newconfig.frame_id );

        setCameraInfo( *camera_info_manager[LEFT] , config.camera_info_url_left , newconfig.camera_info_url_left  );
        setCameraInfo( *camera_info_manager[RIGHT], config.camera_info_url_right, newconfig.camera_info_url_right );

        newconfig.frame_width  = setProperty( camera, CV_CAP_PROP_FRAME_WIDTH , newconfig.frame_width  );
        newconfig.frame_height = setProperty( camera, CV_CAP_PROP_FRAME_HEIGHT, newconfig.frame_height );
        //newconfig.fps          = setProperty( camera, CV_CAP_PROP_FPS         , newconfig.fps          );
        newconfig.brightness   = setProperty( camera, CV_CAP_PROP_BRIGHTNESS  , newconfig.brightness   );
        newconfig.contrast     = setProperty( camera, CV_CAP_PROP_CONTRAST    , newconfig.contrast     );
        newconfig.saturation   = setProperty( camera, CV_CAP_PROP_SATURATION  , newconfig.saturation   );
        newconfig.hue          = setProperty( camera, CV_CAP_PROP_HUE         , newconfig.hue          );
        newconfig.gain         = setProperty( camera, CV_CAP_PROP_GAIN        , newconfig.gain         );
        newconfig.exposure     = setProperty( camera, CV_CAP_PROP_EXPOSURE    , newconfig.exposure     );

            //setFOURCC( camera[i], newconfig.fourcc );

        for( size_t i = 0; i < NUM_CAMERAS; ++i )
        {
            frame[i]->header.frame_id = newconfig.frame_id;
        }

        if( fps != newconfig.fps )
        {
            fps = newconfig.fps;
            timer.setPeriod( ros::Duration( 1. / fps ) );
        }

		config = newconfig;
		reconfiguring = false;
	}

	void capture( const ros::TimerEvent& te )  // 需要将图像分割给left和right
	{
		if( not reconfiguring )
		{
			boost::mutex::scoped_lock lock( mutex );

            camera >> stereo_frame_mat;  // 读取双目图像
            frame[0]->image = stereo_frame_mat(cv::Rect(0, 0, config.frame_width / 2, config.frame_height));
            frame[1]->image = stereo_frame_mat(cv::Rect(config.frame_width / 2, 0, config.frame_width / 2, config.frame_height));

            for( size_t i = 0; i < NUM_CAMERAS; ++i )
            {
                if( frame[i]->image.empty() ) return;
            }

            ros::Time now = ros::Time::now();

            for( size_t i = 0; i < NUM_CAMERAS; ++i )
            {
				frame[i]->header.stamp = now;

			    *camera_info = camera_info_manager[i]->getCameraInfo();
				camera_info->header = frame[i]->header;

				camera_pub[i].publish( frame[i]->toImageMsg(), camera_info );
            }
		}
	}

private:

    void setCameraName( CameraInfoManager& camera_info_manager, const std::string& camera_name )
    {
        if( not camera_info_manager.setCameraName( camera_name ) )
        {
            ROS_ERROR_STREAM( "Invalid camera name '" << camera_name << "'" );
            ros::shutdown();
        }
        else
        {
            ROS_INFO_STREAM( "Camera name set: '" << camera_name << "'" );
        }
    }

    void setCameraInfo( CameraInfoManager& camera_info_manager, const std::string& camera_info_url, std::string& camera_info_url_new )
    {
	    if( camera_info_url != camera_info_url_new )
    	{
	        if( camera_info_manager.validateURL( camera_info_url_new ) )
		    {
    			camera_info_manager.loadCameraInfo( camera_info_url_new );
	    	}
		    else
    		{
	    		camera_info_url_new = camera_info_url;
		    }
    	}
    }
    
    double setProperty( cv::VideoCapture& camera, int property, double value )
    {
        if( camera.set( property, value ) )
        {
            double current_value = camera.get( property );
            ROS_WARN_STREAM(
                    "Failed to set property #" << property << " to " << value <<
                    " (current value = " << current_value << ")"
            );
            return current_value;
        }

        return value;
    }

    std::string setFOURCC( cv::VideoCapture& camera, std::string& value )
    {
        ROS_ASSERT_MSG( value.size() == 4, "Invalid FOURCC codec" );

        int property = CV_CAP_PROP_FOURCC;
        int fourcc = CV_FOURCC( value[0], value[1], value[2], value[3] );
        if( camera.set( property, fourcc ) )
        {
            fourcc = camera.get( property );
            std::string current_value = fourccToString( fourcc );
            ROS_WARN_STREAM(
                "Failed to set FOURCC codec to '" << value <<
                "' (current value = '" << current_value << "' = " <<  fourcc << ")"
            );
            return current_value;
        }

        return value;
    }

    std::string fourccToString( int fourcc )
    {
    
        std::string str( 4, ' ' );
        
        for( size_t i = 0; i < 4; ++i )
        {
            str[i] = fourcc & 255;
            fourcc >>= 8;
        }
        
        return str;
    }

private:
	ros::NodeHandle nh, camera_nh, single_camera_nh[NUM_CAMERAS];
	image_transport::ImageTransport it;
	image_transport::CameraPublisher camera_pub[NUM_CAMERAS];
	sensor_msgs::CameraInfoPtr camera_info;
    boost::shared_ptr< CameraInfoManager > camera_info_manager[NUM_CAMERAS];
	std::string camera_name;

	Config config;
	dynamic_reconfigure::Server< Config > server;
    dynamic_reconfigure::Server< Config >::CallbackType server_callback;
	bool reconfiguring;
	boost::mutex mutex;

    cv::VideoCapture camera;
	cv_bridge::CvImagePtr frame[NUM_CAMERAS];
    cv::Mat stereo_frame_mat;

	ros::Timer timer;

	int camera_index;
	double fps;
};

const int CameraDriver::DEFAULT_CAMERA_INDEX = 0;
const double CameraDriver::DEFAULT_FPS = 30.;
const char* CameraDriver::DEFAULT_CAMERA_NAME = "usb_cam";

const std::string CameraDriver::CameraString[NUM_CAMERAS] = {"left", "right"};


int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "camera_stereo" );

	CameraDriver camera_driver;

	while( ros::ok() )
	{
		ros::spin();
	}

	return EXIT_SUCCESS;
}

