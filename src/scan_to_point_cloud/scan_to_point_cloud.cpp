#include <laser_geometry/laser_geometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

namespace scan_to_point_cloud
{
    /**
     * @brief A nodelet converting laser scans to point clouds.
     *
     * This nodelet converts scans (i.e., sensor_msgs::LaserScan messages)
     * to point clouds (i.e., to sensor_msgs::PointCloud2 messages).
     */
    class ScanToPointCloud: public nodelet::Nodelet
    {
    public:
        ScanToPointCloud() {}
        virtual ~ScanToPointCloud() {}
        void onInit()
        {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            // Process parameters.
            pnh.param("target_frame", target_frame_, target_frame_);
            NODELET_INFO("Using target frame: %s.", target_frame_.c_str());

            pnh.param("fixed_frame", fixed_frame_, fixed_frame_);
            NODELET_INFO("Using fixed frame: %s.", fixed_frame_.c_str());

            pnh.param("channel_options", channel_options_, channel_options_);
            NODELET_INFO("Channel options: %#x.", channel_options_);

            pnh.param("scan_queue_size", scan_queue_size_, scan_queue_size_);
            NODELET_INFO("Scan queue size: %i.", scan_queue_size_);

            pnh.param("cloud_queue_size", cloud_queue_size_, cloud_queue_size_);
            NODELET_INFO("Point cloud queue size: %i.", cloud_queue_size_);

            auto tf_cache = tf_cache_.toSec();
            pnh.param("tf_cache", tf_cache, tf_cache);
            tf_cache_ = ros::Duration(tf_cache);
            NODELET_INFO("Transform cache: %.3g s.", tf_cache_.toSec());

            auto tf_timeout = tf_timeout_.toSec();
            pnh.param("tf_timeout", tf_timeout, tf_timeout);
            tf_timeout_ = ros::Duration(tf_timeout);
            NODELET_INFO("Transform timeout: %.3g s.", tf_timeout_.toSec());

            tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(tf_cache_));
            tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

            // Advertise scan point cloud.
            cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", uint32_t(cloud_queue_size_), false);
            NODELET_INFO("Cloud advertised: %s.", cloud_pub_.getTopic().c_str());

            // Subscribe scan topic.
            scan_sub_ = nh.subscribe("scan", uint32_t(scan_queue_size_), &ScanToPointCloud::convertScan, this);
            NODELET_INFO("Scan subscribed: %s.", scan_sub_.getTopic().c_str());
        }

        void convertScan(const sensor_msgs::LaserScan& scan)
        {
            ros::Duration scan_duration(scan.ranges.size() * scan.time_increment);
            auto scan_finished = scan.header.stamp + scan_duration;
            auto target_frame = target_frame_.empty() ? scan.header.frame_id : target_frame_;
            auto fixed_frame = fixed_frame_.empty() ? scan.header.frame_id : fixed_frame_;
            if (!tf_->canTransform(target_frame, scan_finished,
                                   scan.header.frame_id, scan_finished,
                                   fixed_frame, tf_timeout_))
            {
                NODELET_WARN_THROTTLE(1.0,
                        "Could not transform from %s to %s at %.3f.",
                        scan.header.frame_id.c_str(),
                        fixed_frame_.c_str(),
                        scan.header.stamp.toSec());
                return;
            }

            auto cloud = boost::make_shared<sensor_msgs::PointCloud2>();
            try
            {
                projector_.transformLaserScanToPointCloud(target_frame, scan, *cloud, fixed_frame,
                                                          *tf_, -1.0, channel_options_);
                cloud_pub_.publish(cloud);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_ERROR_THROTTLE(1.0, "Transform exception: %s.", ex.what());
                return;
            }
        }

    protected:
        std::string target_frame_ = "";
        std::string fixed_frame_ = "";
        /**
         * @brief channelOptions Channels to extract.
         *
         * Channels to extract, see laser_geometry.h for details.
         * 0x00 - no channels enabled,
         * 0x01 - enable intensity (default),
         * 0x02 - enable index (default),
         * 0x04 - enable distance,
         * 0x08 - enable stamps,
         * 0x10 - enable viewpoint.
         */
        int channel_options_{laser_geometry::channel_option::Default};
        int scan_queue_size_{2};
        int cloud_queue_size_{2};
        ros::Duration tf_cache_{10.0};
        ros::Duration tf_timeout_{1.0};

        std::shared_ptr<tf2_ros::Buffer> tf_;
        laser_geometry::LaserProjection projector_;

        std::shared_ptr<tf2_ros::TransformListener> tf_sub_;
        ros::Subscriber scan_sub_;
        ros::Publisher cloud_pub_;
    };
}

PLUGINLIB_EXPORT_CLASS(scan_to_point_cloud::ScanToPointCloud, nodelet::Nodelet)
