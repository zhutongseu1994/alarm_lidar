#include "alarm_manager.h"

namespace skywell
{
    AlarmManager::AlarmManager(ros::NodeHandle &nh, skywell::Param *param)
    {
        param_ = param;
        AlarmManagerSubInit(nh, param);
    }

    AlarmManager::~AlarmManager(){};

    AlarmManager::AlarmManagerSubInit(ros::NodeHandle &nh, skywell::Param *param)
    {
        if (param->right_rslidar_topic != "")
        {
            sub_front_rslidar_ =
                nh.subscribe<sensor_msgs::PointCloud2>(param->right_rslidar_topic,
                                                       10,
                                                       boost::bind(&AlarmManager::doRslidarWork,
                                                                   this, _1, param->right_rslidar_topic));
        }

        if (param->left_rslidar_topic != "")
        {
            sub_back_rslidar_ =
                nh.subscribe<sensor_msgs::PointCloud2>(param->left_rslidar_topic,
                                                       10,
                                                       boost::bind(&AlarmManager::doRslidarWork,
                                                                   this, _1, param->left_rslidar_topic));
        }
    }

    void AlarmManager::doRslidarWork(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr, const std::string topic_name)
    {
        param_->loadcfg();

        if (topic_name == param_->right_rslidar_topic != "")
        {
            CloudT::Ptr _current_point_cloud_ptr = boost::make_shared<CloudT>();
            pcl::fromROSMsg(*in_cloud_ptr, *_current_point_cloud_ptr);

            CloudT::Ptr _remove_nan_ptr = boost::make_shared<CloudT>();
            removeNan(_current_point_cloud_ptr, _remove_nan_ptr);
        }
    }

    void AlarmManager::removeNan(CloudT::Ptr in, CloudT::Ptr out)
    {
        std::vector<int> _indices;
        pcl::removeNaNFromPointCloud(*in, *out, _indices);
    }

    void AlarmManager::countPointsNumber(float ray_angle, CloudT::Ptr in_cloud_ptr)
    {
        std::vector<pcl::PointIndices> _radial_division_indices;
        std::vector<PointCloudXYZIRT> _radial_ordered_clouds;

        int _radial_division_num = ceil(360 / ray_angle);
        _radial_division_indices.resize(_radial_division_num);
        _radial_ordered_clouds.resize(_radial_division_num);

        XYZI_to_RTZ(ray_angle, in_cloud_ptr, _radial_division_indices, _radial_ordered_clouds);
    }

    void AlarmManager::XYZI_to_RTZ(float ray_angle, const CloudT::Ptr in_cloud,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZIRT> &out_radial_ordered_clouds)
    {
        //out_radial_divided_indices.clear();
        //#pragma omp parallel for
        //printf("in_cloud->points.size() = %ld\n",in_cloud->points.size());
        for (size_t i = 0; i < in_cloud->points.size(); i++)
        {
            PointXYZIRT new_point;
            auto radius = (float)sqrt(
                in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
            auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / 3.1415926;
            if (theta < 0)
            {
                theta += 360;
            }
            //角度的微分
            auto radial_div = (size_t)floor(theta / ray_angle);
            //printf("x:%f,y:%f,z:%f\n",in_cloud->points[i].x,in_cloud->points[i].y,in_cloud->points[i].z);
            //auto radial_div = 0;
            new_point.point = in_cloud->points[i];
            new_point.radius = radius;
            new_point.theta = theta;
            new_point.radial_div = radial_div;
            new_point.ground_flag = -1;
            new_point.original_index = i;
            //#pragma omp critical
            {
                out_radial_divided_indices[radial_div].indices.push_back(i);
                out_radial_ordered_clouds[radial_div].push_back(new_point);
            }
        } //end for

        //将同一根射线上的点按照半径（距离）排序
        //#pragma omp for

        for (size_t i = 0; i < out_radial_ordered_clouds.size(); i++)
        {
            std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                      [](const PointXYZIRT &a, const PointXYZIRT &b) { return a.radius < b.radius; });
        }
    }
} // namespace skywell