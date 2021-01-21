#ifndef _ALARM_MANAGER_H_
#define _ALARM_MANAGER_H_

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

namespace skywell
{
    class AlarmManager
    {
    public:
        AlarmManager(ros::NodeHandle &nh, skywell::Param *param);
        ~ALarmManager();

    private:
        ros::Subscriber sub_front_rslidar_;
        ros::Subscriber sub_back_rslidar_;
        skywell::Param param_;

    private:
        AlarmManagerSubInit(ros::NodeHandle &hn, skywell::Param *param);
        void doRslidarWork(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr, const std::string topic_name);
        void removeNan(CloudT::Ptr in, CloudT::Ptr out);
    };
} // namespace skywell

#endif