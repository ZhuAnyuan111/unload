#define PCL_NO_PRECOMPILE

#include <algorithm>
#include <thread>
#include <ros/ros.h>
#include <chrono>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <std_srvs/Empty.h>

using namespace std;

class UnloadPointSelection
{
public:
    UnloadPointSelection() : nh("~")
    {

        if (!read_topic_names())
        {
            ROS_ERROR("%s: Error with loading necessary topic names", nh.getNamespace().c_str());
            exit(1);
        }
        // Setup subscriber
        input_truck_lidar_points_subscriber = nh.subscribe(input_truck_lidar_topic_, 1, &UnloadPointSelection::input_truck_lidar_points_call_back, this);
        input_hopper_box_subscriber = nh.subscribe(input_hopper_topic_, 1, &UnloadPointSelection::input_hopper_call_back, this);
        input_unload_times_subscriber = nh.subscribe(input_unload_times_topic_, 1, &UnloadPointSelection::input_unload_times_call_back, this);

        // Setup publisher
        output_traj_publisher = nh.advertise<std_msgs::Float64MultiArray>(output_traj_topic_, 1);
        output_truck_full_publisher = nh.advertise<std_msgs::Bool>(output_truck_full_topic_, 1);

        // Setup service
        calc_unload_point_service = nh.advertiseService("SelectUnloadPointSrv", &UnloadPointSelection::calc_unload_point_call_back, this);

        // Load debug switch option
        read_single_param(CONSOLE_DEBUG_, nh.getNamespace() + "/CONSOLE_DEBUG", false);
        read_single_param(TIMING_DEBUG_, nh.getNamespace() + "/TIMING_DEBUG", false);
        read_single_param(VIS_DEBUG_, nh.getNamespace() + "/VIS_DEBUG", false);

        // Load parameters
        read_single_param(unload_point_selection_param.full_height_bias, nh.getNamespace() + "/full_height_bias", 0.0);
        read_single_param(unload_point_selection_param.num_candidate_points, nh.getNamespace() + "/num_candidate_points", 0);
        read_single_param(unload_point_selection_param.endpoint_to_truckhead, nh.getNamespace() + "/endpoint_to_truckhead", 0.0);
        read_single_param(unload_point_selection_param.bucket_angle, nh.getNamespace() + "/bucket_angle", 0.0);
        read_single_param(unload_point_selection_param.unload_point_height_bias, nh.getNamespace() + "/unload_point_height_bias", 0.0);
        read_single_param(unload_point_selection_param.first_point_num, nh.getNamespace() + "/first_point_num", 0);
        read_single_param(unload_point_selection_param.select_soil_radius, nh.getNamespace() + "/select_soil_radius", 0.0);
        read_single_param(unload_point_selection_param.num_soil_point, nh.getNamespace() + "/num_soil_point", 0);
        read_single_param(unload_point_selection_param.bias_x, nh.getNamespace() + "/bias_x", 0.0);
        read_single_param(unload_point_selection_param.bias_z, nh.getNamespace() + "/bias_z", 0.0);
        read_single_param(unload_point_selection_param.length_boom, nh.getNamespace() + "/length_boom", 0.0);
        read_single_param(unload_point_selection_param.length_arm, nh.getNamespace() + "/length_arm", 0.0);
        read_single_param(unload_point_selection_param.length_bucket, nh.getNamespace() + "/length_bucket", 0.0);
        read_single_param(unload_point_selection_param.P3_boomBias, nh.getNamespace() + "/P3_boomBias", 0.0);
        read_single_param(unload_point_selection_param.bucket_attitude_angle, nh.getNamespace() + "/bucket_attitude_angle", 0.0);
        read_eigen_vector2d_param(unload_point_selection_param.boom_limit, nh.getNamespace() + "/boom_limit");
        read_eigen_vector2d_param(unload_point_selection_param.arm_limit, nh.getNamespace() + "/arm_limit");
        read_eigen_vector2d_param(unload_point_selection_param.bucket_limit, nh.getNamespace() + "/bucket_limit");

        read_eigen_vector3f_param(PointC, nh.getNamespace() + "/PointC");
        read_eigen_vector3f_param(PointF, nh.getNamespace() + "/PointF");
        read_eigen_vector3f_param(PointQ_F, nh.getNamespace() + "/PointQ_F");
        read_eigen_vector3f_param(PointV_Q, nh.getNamespace() + "/PointV_Q");
        read_eigen_vector3f_param(Carriage_to_Boom_Offset, nh.getNamespace() + "/Carriage_to_Boom_Offset");
        read_single_param(Bucket_Width, nh.getNamespace() + "/Bucket_Width", 0.0);
        read_single_param(Bucket_Length, nh.getNamespace() + "/Bucket_Length", 0.0);

#if (CONSOLE_DEBUG)
        if (CONSOLE_DEBUG_)
        {
            unload_point_selection_param.print();
        }
#endif

#if (VIS_DEBUG)
        all_candidate_points_pub = nh.advertise<visualization_msgs::Marker>("/all_candidate_points", 1, true);
        feasible_points_pub = nh.advertise<visualization_msgs::Marker>("/feasible_points", 1, true);
        infeasible_points_pub = nh.advertise<visualization_msgs::Marker>("/infeasible_points", 1, true);
        selected_points_pub = nh.advertise<visualization_msgs::Marker>("/selected_points", 1, true);
        full_points_pub = nh.advertise<visualization_msgs::Marker>("/full_points", 1, true);
        unfull_points_pub = nh.advertise<visualization_msgs::Marker>("/unfull_points", 1, true);
        final_unload_point_pub = nh.advertise<visualization_msgs::Marker>("/final_unload_point", 1, true);
        unload_trajectory_pub = nh.advertise<visualization_msgs::Marker>("/unload_trajectory", 1, true);
        unload_upper_plane_pub = nh.advertise<visualization_msgs::Marker>("/unload_upper_plane", 1, true);
#endif

#if (TIMING_DEBUG)
        frames_ = 0;
        avg_time_ = 0;
        max_time_ = 0;
        M2_ = 0;
        std_time_ = 0;
#endif
    }

    ~UnloadPointSelection()
    {
        nh.deleteParam(nh.getNamespace());
    }

    ros::NodeHandle GetNodeHandle()
    {
        return nh;
    }

private:
    typedef struct
    {
        double time;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;
    } lidar_data_t;

    typedef struct
    {
        std::string box_name;
        Eigen::Vector3d box_center;
        Eigen::Vector3d box_size;
        double box_yaw;
#if (VIS_DEBUG)
        ros::Publisher box_pub;
#endif

        void print()
        {
            ROS_INFO("Box %s: [%2.4f, %2.4f, %2.4f], size: [%2.4f, %2.4f, %2.4f], yaw: %2.4f",
                     box_name.c_str(),
                     box_center.x(), box_center.y(), box_center.z(),
                     box_size.x(), box_size.y(), box_size.z(), box_yaw);
        }

#if (VIS_DEBUG)
        void visualize_box()
        {
            Eigen::Quaternionf q;

            Eigen::Affine3f yaw_rotation = Eigen::Affine3f::Identity();
            yaw_rotation.rotate(Eigen::AngleAxisf(box_yaw, Eigen::Vector3f::UnitZ()) *
                                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));
            q = yaw_rotation.rotation();
            // ROS_INFO("%0.2f, %0.2f, %0.2f, %0.2f", q.x(), q.y(), q.z(), q.w());

            visualization_msgs::Marker marker;
            marker.header.frame_id = "base";
            marker.header.stamp = ros::Time();
            marker.ns = box_name;
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = box_center.x();
            marker.pose.position.y = box_center.y();
            marker.pose.position.z = box_center.z();
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();
            marker.scale.x = box_size.x();
            marker.scale.y = box_size.y();
            marker.scale.z = box_size.z();
            marker.color.a = 0.5; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            box_pub.publish(marker);
        }
#endif

    } box_info;

    typedef struct
    {
        double full_height_bias = 0;
        int num_candidate_points = 0;
        double endpoint_to_truckhead = 0;
        double bucket_angle = 0;
        double unload_point_height_bias = 0;
        int first_point_num = 0;
        double select_soil_radius = 0;
        int num_soil_point = 0;
        double bias_x = 0;
        double bias_z = 0;
        double length_boom = 0.0;
        double length_arm = 0.0;
        double length_bucket = 0.0;
        double P3_boomBias = 0.0;
        double bucket_attitude_angle = 0.0;
        Eigen::Vector2d boom_limit{-45.44, 55.28};
        Eigen::Vector2d arm_limit{-138.69, -37.35};
        Eigen::Vector2d bucket_limit{-133.48, 30.53};

        void print()
        {
            ROS_INFO("full_height_bias: %2.4f", full_height_bias);
            ROS_INFO("endpoint_to_truckhead: %2.4f", endpoint_to_truckhead);
            ROS_INFO("bucket_angle: %2.4f", bucket_angle);
            ROS_INFO("select_soil_radius: %2.4f", select_soil_radius);
            ROS_INFO("num_soil_point: %d", num_soil_point);
            ROS_INFO("unload_point_height_bias: %2.4f", unload_point_height_bias);
            ROS_INFO("first_point_num: %d", first_point_num);
            ROS_INFO("P3_boomBias: %2.4f", P3_boomBias);

            ROS_INFO("num_candidate_points: %d", num_candidate_points);
            ROS_INFO("bias_x: %2.4f", bias_x);
            ROS_INFO("bias_z: %2.4f", bias_z);
            ROS_INFO("length_boom: %2.4f", length_boom);
            ROS_INFO("length_arm: %2.4f", length_arm);
            ROS_INFO("length_bucket: %2.4f", length_bucket);
            ROS_INFO("bucket_attitude_angle: %2.4f", bucket_attitude_angle);

            ROS_INFO("boom_limit: [%2.4f, %2.4f]", boom_limit.x(), boom_limit.y());
            ROS_INFO("arm_limit: [%2.4f, %2.4f]", arm_limit.x(), arm_limit.y());
            ROS_INFO("bucket_limit: [%2.4f, %2.4f]", bucket_limit.x(), bucket_limit.y());
        }
    } param;

    enum SOILFULLSTATUS
    {
        UNKNOWN,
        FULL,
        NOTFULL,
        BLIND
    };

    enum FEASIBILITY
    {
        UNCLEAR = 0,
        FEASIBLE = 1,
        INFEASIBLE = 2,
    };

    // node handle
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber input_truck_lidar_points_subscriber;
    ros::Subscriber input_hopper_box_subscriber;
    ros::Subscriber input_unload_times_subscriber;

    // publisher
    ros::Publisher output_traj_publisher;
    ros::Publisher output_truck_full_publisher;

    // service
    ros::ServiceServer calc_unload_point_service;

    // LiDAR data
    lidar_data_t input_truck_lidar_data;

    // hopper box
    std::vector<box_info> hopper_boxes;

    // geometry parameters
    // Carriage based transform
    Eigen::Affine3f transform_carriage_base = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_platform_boom = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_boom_arm = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_arm_bkt = Eigen::Affine3f::Identity();

    // Geometry information
    Eigen::Vector3f PointC;
    Eigen::Vector3f PointF;
    Eigen::Vector3f PointQ_F;
    Eigen::Vector3f PointV_Q;
    Eigen::Vector3f Carriage_to_Boom_Offset;
    double Bucket_Width;
    double Bucket_Length;

    // unload parameters & outputs
    param unload_point_selection_param;
    int64_t unload_times = 0;
    double P3_BoomRef = 0.0;
    bool truck_full_flag = false;
    Eigen::Index unload_point_index = 0;
    Eigen::Vector3d unload_point;
    Eigen::Vector4d unload_joint;

    // topic name
    std::string input_truck_lidar_topic_;
    std::string input_hopper_topic_;
    std::string input_unload_times_topic_;
    std::string output_traj_topic_;
    std::string output_truck_full_topic_;

#if (VIS_DEBUG)
    // publisher to rviz for visualization
    ros::Publisher all_candidate_points_pub;
    ros::Publisher feasible_points_pub;
    ros::Publisher infeasible_points_pub;
    ros::Publisher selected_points_pub;
    ros::Publisher full_points_pub;
    ros::Publisher unfull_points_pub;
    ros::Publisher final_unload_point_pub;
    ros::Publisher unload_trajectory_pub;
    ros::Publisher unload_upper_plane_pub;
#endif

    // Timing information
#if (TIMING_DEBUG)
    long frames_;
    double avg_time_;
    double max_time_;
    double M2_;
    double std_time_;
#endif
    // Debug switch
    bool CONSOLE_DEBUG_ = false;
    bool TIMING_DEBUG_ = false;
    bool VIS_DEBUG_ = false;

    bool read_topic_names()
    {
        if (!nh.getParam("input_truck_lidar_topic", input_truck_lidar_topic_))
        {
            ROS_ERROR("Could not read parameter `input_truck_lidar_topic`.");
            return false;
        }
        if (!nh.getParam("input_hopper_topic", input_hopper_topic_))
        {
            ROS_ERROR("Could not read parameter `input_hopper_topic`.");
            return false;
        }
        if (!nh.getParam("input_unload_times_topic", input_unload_times_topic_))
        {
            ROS_ERROR("Could not read parameter `input_unload_times_topic`.");
            return false;
        }
        nh.param("output_traj_topic", output_traj_topic_, std::string("/traj"));

        ROS_INFO("Successfully Load %s: %s", "input_truck_lidar_topic", input_truck_lidar_topic_.c_str());
        ROS_INFO("Successfully Load %s: %s", "input_hopper_topic", input_hopper_topic_.c_str());
        ROS_INFO("Successfully Load %s: %s", "input_unload_times_topic", input_unload_times_topic_.c_str());
        ROS_INFO("Successfully Load %s: %s", "output_traj_topic", output_traj_topic_.c_str());
        return true;
    }

    template <typename T>
    void read_single_param(T &v, const string &param_name, T default_v)
    {
        if (!nh.hasParam(param_name))
        {
            ROS_WARN("Did not find param %s, use default value of %s", param_name.c_str(), std::to_string(default_v).c_str());
            v = default_v;
            return;
        }
        if (nh.getParam(param_name, v))
        {
            ROS_INFO("Successfully Load %s", param_name.c_str());
        }
        else
        {
            ROS_ERROR("Failed to get param %s", param_name.c_str());
        }
    }

    void read_eigen_vector2d_param(Eigen::Vector2d &v, const string &param_name)
    {
        // 从launch文件读取Eigen Vector
        std::vector<double> v_list;
        if (nh.getParam(param_name, v_list))
        {
            ROS_INFO("Successfully Load %s", param_name.c_str());
        }
        else
        {
            ROS_ERROR("Failed to get param %s", param_name.c_str());
        }
        if (v_list.size() == 2)
        {
            for (int i = 0; i < 2; i++)
            {
                v[i] = v_list[i];
            }
            return;
        }
        if (v_list.size() < 2)
        {
            ROS_WARN("Size of %s has length of %ld instead of 2!", param_name.c_str(), v_list.size());
            for (size_t i = 0; i < v_list.size(); i++)
            {
                v[i] = v_list[i];
            }
            return;
        }
        if (v_list.size() > 2)
        {
            ROS_WARN("Size of %s has length of %ld instead of 2!", param_name.c_str(), v_list.size());
            for (int i = 0; i < 3; i++)
            {
                v[i] = v_list[i];
            }
            return;
        }
    }

    void read_eigen_vector3f_param(Eigen::Vector3f &v, const string &param_name)
    {
        // 从launch文件读取Eigen Vector
        std::vector<double> v_list;
        if (nh.getParam(param_name, v_list))
        {
            ROS_INFO("Successfully Load %s", param_name.c_str());
        }
        else
        {
            ROS_ERROR("Failed to get param %s", param_name.c_str());
        }
        if (v_list.size() == 3)
        {
            for (int i = 0; i < 3; i++)
            {
                v[i] = v_list[i];
            }
            return;
        }
        if (v_list.size() < 3)
        {
            ROS_WARN("Size of %s has length of %ld instead of 3!", param_name.c_str(), v_list.size());
            for (size_t i = 0; i < v_list.size(); i++)
            {
                v[i] = v_list[i];
            }
            return;
        }
        if (v_list.size() > 3)
        {
            ROS_WARN("Size of %s has length of %ld instead of 3!", param_name.c_str(), v_list.size());
            for (int i = 0; i < 3; i++)
            {
                v[i] = v_list[i];
            }
            return;
        }
    }

    void input_truck_lidar_points_call_back(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pointcloud_msg, *pointcloud);
        double t = round(pointcloud_msg->header.stamp.toSec() * 1000) / 1000;
        input_truck_lidar_data.pointcloud = pointcloud;
        input_truck_lidar_data.time = t;
    }

    void input_hopper_call_back(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        hopper_boxes.clear();
        hopper_boxes.resize(msg->layout.dim[0].size);
        ROS_WARN_COND(hopper_boxes.size() > 1, "%s: More than one hopper box (%ld)", nh.getNamespace().c_str(), hopper_boxes.size());

        for (size_t i = 0; i < msg->layout.dim[0].size; i++)
        {
            hopper_boxes[i].box_name = input_hopper_topic_ + "_" + std::to_string(i);
            hopper_boxes[i].box_center.x() = msg->data[i * msg->layout.dim[1].size + 0];
            hopper_boxes[i].box_center.y() = msg->data[i * msg->layout.dim[1].size + 1];
            hopper_boxes[i].box_center.z() = msg->data[i * msg->layout.dim[1].size + 2];
            hopper_boxes[i].box_size.x() = msg->data[i * msg->layout.dim[1].size + 3];
            hopper_boxes[i].box_size.y() = msg->data[i * msg->layout.dim[1].size + 4];
            hopper_boxes[i].box_size.z() = msg->data[i * msg->layout.dim[1].size + 5];
            hopper_boxes[i].box_yaw = msg->data[i * msg->layout.dim[1].size + 6];
#if (VIS_DEBUG)
            if (VIS_DEBUG_)
            {
                hopper_boxes[i].box_pub = nh.advertise<visualization_msgs::Marker>(hopper_boxes[i].box_name + "_box", 1);
                hopper_boxes[i].visualize_box();
            }
#endif
        }
    }

    void input_unload_times_call_back(const std_msgs::Int64::ConstPtr msg)
    {
        unload_times = msg->data;
#if (CONSOLE_DEBUG)
        if (CONSOLE_DEBUG_)
        {
            ROS_INFO("input unload_times: %ld", unload_times);
        }
#endif
    }

    bool calc_unload_point_call_back(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
        (void)request;
        (void)response;
        ROS_INFO("Request unload point...");
        // calc_unload_point();
        if (hopper_boxes.empty())
        {
            ROS_ERROR("No Hopper found");
        }
        else if (hopper_boxes.size() > 1)
        {
            ROS_WARN("More than 1 hopper found!");
        }
        else
        {
            P3_BoomRef = boomRef_P3(hopper_boxes[0], unload_point_selection_param);
            candidate_points_generation(unload_point_selection_param, hopper_boxes[0]);
            generate_traj();
#if (CONSOLE_DEBUG)
            if (CONSOLE_DEBUG_)
            {
                ROS_INFO("%s Parameters", nh.getNamespace().c_str());
                unload_point_selection_param.print();
                ROS_INFO("%s Hopper info", nh.getNamespace().c_str());
                hopper_boxes[0].print();
                ROS_INFO("%s Results", nh.getNamespace().c_str());
                ROS_INFO("%s P3_BoomRef: %2.4f", nh.getNamespace().c_str(), P3_BoomRef);
                ROS_INFO("%s Select idx: %ld", nh.getNamespace().c_str(), unload_point_index);
                ROS_INFO("%s unload_point: [%2.4f, %2.4f, %2.4f]", nh.getNamespace().c_str(), unload_point.x(), unload_point.y(), unload_point.z());
                ROS_INFO("%s unload_joint: [%2.4f, %2.4f, %2.4f, %2.4f]", nh.getNamespace().c_str(), unload_joint.x(), unload_joint.y(), unload_joint.z(), unload_joint.w());
                ROS_INFO("%s truck_full_flag: %d", nh.getNamespace().c_str(), truck_full_flag);
            }
#endif
        }

        return true;
    }

    void visualize_points(ros::Publisher &pub,
                          Eigen::Matrix<double, Eigen::Dynamic, 3> points,
                          string ns,
                          double point_size,
                          Eigen::Vector4d rgba_color)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base";
        marker.header.stamp = ros::Time();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = point_size;
        marker.scale.y = point_size;
        marker.scale.z = point_size;
        marker.color.a = rgba_color.w(); // Don't forget to set the alpha!
        marker.color.r = rgba_color.x();
        marker.color.g = rgba_color.y();
        marker.color.b = rgba_color.z();
        marker.points.clear();

        // for each dimension
        geometry_msgs::Point point;
        for (int r = 0; r < points.rows(); r++)
        {
            point.x = points(r, 0);
            point.y = points(r, 1);
            point.z = points(r, 2);
            marker.points.push_back(point);
        }
        if (!marker.points.empty())
        {
            pub.publish(marker);
        }
    }

    void visualize_traj_line(ros::Publisher &pub,
                             Eigen::Matrix<double, Eigen::Dynamic, 3> points,
                             string ns,
                             double point_size,
                             Eigen::Vector4d rgba_color)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base";
        marker.header.stamp = ros::Time();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = point_size;
        marker.scale.y = point_size;
        marker.scale.z = point_size;
        marker.color.a = rgba_color.w(); // Don't forget to set the alpha!
        marker.color.r = rgba_color.x();
        marker.color.g = rgba_color.y();
        marker.color.b = rgba_color.z();
        marker.points.clear();

        // for each dimension
        geometry_msgs::Point point;
        for (int r = 0; r < points.rows(); r++)
        {
            point.x = points(r, 0);
            point.y = points(r, 1);
            point.z = points(r, 2);
            marker.points.push_back(point);
        }
        if (!marker.points.empty())
        {
            pub.publish(marker);
        }
    }

    void candidate_points_generation(const param &p, const box_info &b)
    {
        // 假设在可行点中选出3个候选点
        int Num_SelectedPoints = 3;

        Eigen::Vector3d truck_center = b.box_center;
        Eigen::Vector3d truck_size = b.box_size;

        std::vector<SOILFULLSTATUS> soil_full_in_point(Num_SelectedPoints);
        fill(soil_full_in_point.begin(), soil_full_in_point.end(), SOILFULLSTATUS::UNKNOWN);

        double length_bias = truck_size.x() / 2 + p.endpoint_to_truckhead;
        Eigen::VectorXd PresetTruckUnloadPoint = Eigen::VectorXd::LinSpaced(p.num_candidate_points, -length_bias, length_bias);
        Eigen::Matrix<double, Eigen::Dynamic, 3> CandidateUnloadPoints(p.num_candidate_points, 3);
        CandidateUnloadPoints.setConstant(0);
        calculate_candidate_points(p, hopper_boxes[0], PresetTruckUnloadPoint, CandidateUnloadPoints);
#if (VIS_DEBUG)
        if (VIS_DEBUG_)
        {
            Eigen::Vector4d rgba_color = {1.0, 1.0, 0.0, 1.0};
            visualize_points(all_candidate_points_pub, CandidateUnloadPoints, "all_candidate_points", 0.05, rgba_color);
        }
#endif

        std::vector<FEASIBILITY> feasibility(p.num_candidate_points);
        fill(feasibility.begin(), feasibility.end(), FEASIBILITY::UNCLEAR);

        std::vector<double> unload_capacity(Num_SelectedPoints);
        fill(unload_capacity.begin(), unload_capacity.end(), 0.0);

        Eigen::Matrix<double, Eigen::Dynamic, 4> joint_angles(feasibility.size(), 4);
        Eigen::Matrix<double, Eigen::Dynamic, 3> selected_unload_points(feasibility.size(), 3);
        Eigen::Matrix<double, Eigen::Dynamic, 4> selected_unload_joints(feasibility.size(), 4);
        selected_unload_points.setConstant(0);
        selected_unload_joints.setConstant(0);
        for (int i = 0; i < joint_angles.rows(); i++)
        {
            joint_angles(i, 0) = 180;
            joint_angles(i, 1) = 20;
            joint_angles(i, 2) = -70;
            joint_angles(i, 3) = 20;
        }
        for (int i = 0; i < p.num_candidate_points; i++)
        {
            inverse_kinematics_solution(CandidateUnloadPoints.row(i), p, feasibility, joint_angles, i);
            if (feasibility[i] == FEASIBILITY::INFEASIBLE)
            {
                // 若定铲斗姿态角求逆解时铲斗关节角超出限制，则改为固定铲斗关节角30°计算
                inverse_kinematics_solution_jointbucket(CandidateUnloadPoints.row(i), p, feasibility, joint_angles, i);
            }
        }
        std::vector<size_t> case_feasible;
        for (size_t i = 0; i < feasibility.size(); i++)
        {
            if (feasibility[i] == FEASIBILITY::FEASIBLE)
            {
                case_feasible.push_back(i);
            }
        }
#if (VIS_DEBUG)
        if (VIS_DEBUG_)
        {
            Eigen::Vector4d rgba_color_feasible = {0.0, 1.0, 1.0, 1.0};
            Eigen::Vector4d rgba_color_infeasible = {1.0, 0.0, 0.0, 1.0};
            Eigen::Matrix<double, Eigen::Dynamic, 3> feasible_points;
            Eigen::Matrix<double, Eigen::Dynamic, 3> infeasible_points;
            for (size_t i = 0; i < feasibility.size(); i++)
            {
                if (feasibility[i] == FEASIBILITY::FEASIBLE)
                {
                    feasible_points.conservativeResize(feasible_points.rows() + 1, feasible_points.cols());
                    feasible_points.row(feasible_points.rows() - 1) = CandidateUnloadPoints.row(i);
                }
                else
                {
                    infeasible_points.conservativeResize(infeasible_points.rows() + 1, infeasible_points.cols());
                    infeasible_points.row(infeasible_points.rows() - 1) = CandidateUnloadPoints.row(i);
                }
            }

            visualize_points(feasible_points_pub, feasible_points, "feasible_points", 0.08, rgba_color_feasible);
            visualize_points(infeasible_points_pub, infeasible_points, "infeasible_points", 0.08, rgba_color_infeasible);
        }
#endif

        if (case_feasible.empty())
        {
            // TODO
            ROS_ERROR_COND(case_feasible.empty(), "%s: no case is feasible", nh.getNamespace().c_str());
            return;
        }
        int SelectedPointStep = std::round((case_feasible.size() - 1) / (Num_SelectedPoints - 1));

        for (int i = 0; i < Num_SelectedPoints; i++)
        {
            if (i == 0)
            {
                selected_unload_joints.row(0) = joint_angles.row(case_feasible[0]);
                selected_unload_points.row(0) = CandidateUnloadPoints.row(case_feasible[0]);
            }
            else
            {
                int index = i * SelectedPointStep; // 1 10  20 30 40
                index = std::min(index, static_cast<int>(case_feasible.size()));
                // candidate的index分别为1, 10, 20, 30, 40
                selected_unload_joints.row(i) = joint_angles.row(case_feasible[index - 1]);
                selected_unload_points.row(i) = CandidateUnloadPoints.row(case_feasible[index - 1]);
            }
        }
#if (VIS_DEBUG)
        if (VIS_DEBUG_)
        {
            Eigen::Vector4d rgba_color = {1.0, 0.0, 1.0, 1.0};
            visualize_points(selected_points_pub, selected_unload_points, "selected_points", 0.12, rgba_color);
        }
#endif
        double full_MeanHeight = truck_center.z() + truck_size.z() / 2 + p.full_height_bias;
        for (int i = 0; i < Num_SelectedPoints; i++)
        {
            judge_soil_full_in_point(selected_unload_points.row(i), p, full_MeanHeight, unload_capacity, soil_full_in_point, i);
        }
        // Another function
        unload_point_selection(p, soil_full_in_point, selected_unload_points, selected_unload_joints);
#if (VIS_DEBUG)
        if (VIS_DEBUG_)
        {
            Eigen::Vector4d rgba_color_full = {1.0, 0.0, 0.0, 1.0};
            Eigen::Vector4d rgba_color_not_full = {0.0, 0.0, 1.0, 1.0};
            Eigen::Vector4d rgba_color_final = {0.0, 1.0, 0.0, 1.0};
            Eigen::Matrix<double, Eigen::Dynamic, 3> full_points;
            Eigen::Matrix<double, Eigen::Dynamic, 3> unfull_points;
            Eigen::Matrix<double, 1, 3> final_unload_point;
            for (int i = 0; i < selected_unload_points.rows(); i++)
            {
                if (soil_full_in_point[i] == SOILFULLSTATUS::FULL)
                {
                    full_points.conservativeResize(full_points.rows() + 1, full_points.cols());
                    full_points.row(full_points.rows() - 1) = selected_unload_points.row(i);
                }
                else
                {
                    unfull_points.conservativeResize(unfull_points.rows() + 1, unfull_points.cols());
                    unfull_points.row(unfull_points.rows() - 1) = selected_unload_points.row(i);
                }
            }
            final_unload_point.row(0) = unload_point;

            visualize_points(full_points_pub, full_points, "full_points", 0.15, rgba_color_full);
            visualize_points(unfull_points_pub, unfull_points, "unfull_points", 0.15, rgba_color_not_full);
            visualize_points(final_unload_point_pub, final_unload_point, "final_unload_point", 0.3, rgba_color_final);
        }
#endif
    }

    void calculate_candidate_points(const param &p, const box_info &b, const Eigen::VectorXd &PresetTruckUnloadPoint, Eigen::Matrix<double, Eigen::Dynamic, 3> &CandidateUnloadPoints)
    {
        // 通过中心点坐标确定另外两个候选卸载点的位置
        CandidateUnloadPoints = b.box_center.replicate(1, PresetTruckUnloadPoint.size()).transpose();
        CandidateUnloadPoints.col(0) += std::cos(b.box_yaw) * PresetTruckUnloadPoint;
        CandidateUnloadPoints.col(1) += std::sin(b.box_yaw) * PresetTruckUnloadPoint;
        CandidateUnloadPoints.col(2).array() = b.box_center.z() + b.box_size.z() / 2 + p.unload_point_height_bias;
    }

    void inverse_kinematics_solution(const Eigen::Vector3d point, const param &p, std::vector<FEASIBILITY> &feasibility, Eigen::Matrix<double, Eigen::Dynamic, 4> &joint_angle, size_t i)
    {
        // 依据铲斗齿间位置和铲斗姿态角，计算各关节角度值（SY870)
        // function [feasibility, joint_angle] = Inverse_kinematics_solution(point, bucket_attitude_angle)
        double bias_z = p.bias_z;
        double bias_x = p.bias_x;
        double length_boom = p.length_boom;
        double length_arm = p.length_arm;
        double length_bucket = p.length_bucket;
        double bucket_attitude_angle = p.bucket_attitude_angle;
        length_bucket /= 2.0;
        double x = point.x();
        double y = point.y();
        double z = point.z();
        // % bucket_attitude_angle = arrive_point(4);
        Eigen::Vector2d boom_limit = p.boom_limit;
        Eigen::Vector2d arm_limit = p.arm_limit;
        Eigen::Vector2d bucket_limit = p.bucket_limit;
        feasibility[i] = FEASIBILITY::FEASIBLE;

        joint_angle.row(i) = Eigen::Vector4d{180.0, boom_limit.y(), arm_limit.y(), bucket_limit.y()};
        double swing_angle = std::atan2(y, x) * 180 / M_PI;

        double x_1 = 0;
        if (std::fabs(swing_angle - 90.0) < 1e-15)
        {
            x_1 = y - bias_x * std::sin(swing_angle / 180 * M_PI);
        }
        else if (std::fabs(swing_angle + 90.0) < 1e-15)
        {
            x_1 = -(y - bias_x * std::sin(swing_angle / 180 * M_PI));
        }
        else
        {
            x_1 = (x - bias_x * std::cos(swing_angle / 180 * M_PI)) / (std::cos(swing_angle / 180 * M_PI));
        }
        double z_1 = z - bias_z;

        double x3 = x_1 - length_bucket * std::cos(bucket_attitude_angle / 180 * M_PI);
        double z3 = z_1 - length_bucket * std::sin(bucket_attitude_angle / 180 * M_PI);

        double l5 = std::sqrt(x_1 * x_1 + z_1 * z_1);
        double l6 = std::sqrt(x3 * x3 + z3 * z3);

        double cos_beta = (l5 * l5 + l6 * l6 - length_bucket * length_bucket) / (2 * l5 * l6);
        double cos_alpha = (length_boom * length_boom + l6 * l6 - length_arm * length_arm) / (2 * length_boom * l6);

        double r = 0;
        double b = 0;
        double a = 0;
        if ((-1 <= cos_beta && cos_beta <= 1) && (-1 <= cos_alpha && cos_alpha <= 1))
        {
            r = std::atan2(z_1, x_1) * 180 / M_PI;
            b = std::acos(cos_beta) * 180 / M_PI;
            a = std::acos(cos_alpha) * 180 / M_PI;
        }
        else
        {
            feasibility[i] = FEASIBILITY::INFEASIBLE;
            return;
        }

        double boom_angle = r + b + a;
        double arm_angle = std::acos((length_boom * length_boom + length_arm * length_arm - l6 * l6) / (2 * length_boom * length_arm)) * 180 / M_PI - 180;
        double bucket_angle = bucket_attitude_angle - boom_angle - arm_angle;

        // 将回转角度转换为与回转传感器一致
        swing_angle = process_swing_angle(swing_angle);

        // 关节限位
        joint_angle.row(i) = Eigen::Vector4d{swing_angle, boom_angle, arm_angle, bucket_angle};

        if ((boom_angle < boom_limit.x() || boom_angle > boom_limit.y() || arm_angle < arm_limit.x() || arm_angle > arm_limit.y() || bucket_angle < bucket_limit.x() || bucket_angle > bucket_limit.y()))
        {
            // 如果无解，轨迹生成失败，可行性设为0
            feasibility[i] = FEASIBILITY::INFEASIBLE;
            // 结束循环
            return;
        }
    }

    void inverse_kinematics_solution_jointbucket(Eigen::Vector3d point, const param &p, std::vector<FEASIBILITY> &feasibility, Eigen::Matrix<double, Eigen::Dynamic, 4> &joint_angle, size_t i)
    {
        // 依据铲斗齿间位置和铲斗姿态角，计算各关节角度值（SY870)
        // function [feasibility, joint_angle] = Inverse_kinematics_solution(point, bucket_attitude_angle)

        double bias_z = p.bias_z;
        double bias_x = p.bias_x;
        double length_boom = p.length_boom;
        double length_arm = p.length_arm;
        double length_bucket = p.length_bucket;
        double bucket_angle = p.bucket_angle;
        length_bucket /= 2.0;
        double x = point.x();
        double y = point.y();
        double z = point.z();

        // % bucket_attitude_angle = arrive_point(4);
        Eigen::Vector2d boom_limit = p.boom_limit;
        Eigen::Vector2d arm_limit = p.arm_limit;
        Eigen::Vector2d bucket_limit = p.bucket_limit;
        feasibility[i] = FEASIBILITY::FEASIBLE;

        joint_angle.row(i) = Eigen::Vector4d{180.0, boom_limit.y(), arm_limit.y(), bucket_limit.y()};
        double swing_angle = std::atan2(y, x) * 180 / M_PI;

        double edge_bucket_x = x - bias_x * cos(swing_angle / 180 * M_PI);
        double edge_bucket_y = y - bias_x * sin(swing_angle / 180 * M_PI);
        double edge_bucket_z = z - bias_z;

        double theta_FQV = 0;
        double theta_FCV = 0;
        double theta_CFV = 0;
        if (bucket_angle < 0)
        {
            theta_FQV = M_PI + bucket_angle / 180 * M_PI;
        }
        else
        {
            theta_FQV = M_PI - bucket_angle / 180 * M_PI;
        }

        double FV = std::sqrt(length_arm * length_arm + length_bucket * length_bucket - 2 * cos(theta_FQV) * length_arm * length_bucket);
        double CV = std::sqrt(edge_bucket_x * edge_bucket_x + edge_bucket_y * edge_bucket_y + edge_bucket_z * edge_bucket_z);
        double cos_FCV = (length_boom * length_boom + (edge_bucket_x * edge_bucket_x + edge_bucket_y * edge_bucket_y + edge_bucket_z * edge_bucket_z) - FV * FV) / (2 * length_boom * std::sqrt(edge_bucket_x * edge_bucket_x + edge_bucket_y * edge_bucket_y + edge_bucket_z * edge_bucket_z));
        double cos_CFV = (length_boom * length_boom + FV * FV - CV * CV) / (2 * length_boom * FV);
        // % mustBeInRange(cos_FCV, -1, 1)
        if (-1 <= cos_FCV && cos_FCV <= 1 && -1 <= cos_CFV && cos_CFV <= 1)
        {
            theta_FCV = acos(cos_FCV);
            theta_CFV = acos(cos_CFV);
        }
        else
        {
            feasibility[i] = FEASIBILITY::INFEASIBLE;
            return;
        }
        double boom_angle = theta_FCV + atan2(edge_bucket_z, std::sqrt(edge_bucket_x * edge_bucket_x + edge_bucket_y * edge_bucket_y));
        double arm_angle = 0.0;
        double theta_VFQ = acos((FV * FV + length_arm * length_arm - length_bucket * length_bucket) / (2 * FV * length_arm));
        if (bucket_angle < 0)
        {
            arm_angle = theta_CFV + theta_VFQ - M_PI;
        }
        else
        {
            arm_angle = theta_CFV - theta_VFQ - M_PI;
        }

        // 将回转角度转换为与回转传感器一致
        swing_angle = process_swing_angle(swing_angle);

        if ((boom_angle < boom_limit.x() / 180 * M_PI || boom_angle > boom_limit.y() / 180 * M_PI || arm_angle < arm_limit.x() / 180 * M_PI || arm_angle > arm_limit.y() / 180 * M_PI || bucket_angle < bucket_limit.x() || bucket_angle > bucket_limit.y()))
        {
            // 如果无解，轨迹生成失败，可行性设为0
            feasibility[i] = FEASIBILITY::INFEASIBLE;
            // 结束循环
            return;
        }

        boom_angle = boom_angle * 180 / M_PI;
        arm_angle = arm_angle * 180 / M_PI;

        joint_angle.row(i) = Eigen::Vector4d{swing_angle, boom_angle, arm_angle, bucket_angle};
    }

    double boomRef_P3(const box_info &b, const param &p)
    {
        // 假设斗杆与铲斗姿态角均为-90°，计算在车斗上侧面中心点高度对应的动臂高度为P3动臂参考
        double P3_z_Limit = b.box_center.z() + b.box_size.z() / 2 - p.bias_z + p.P3_boomBias;
        double P3_BoomRef = std::asin((p.length_arm + p.length_bucket + P3_z_Limit) / p.length_boom) * 180 / M_PI;

        if (!(P3_BoomRef >= p.boom_limit.x() && P3_BoomRef <= p.boom_limit.y()))
        {
            P3_BoomRef = p.boom_limit.y();
        }
        return P3_BoomRef;
    }

    double process_swing_angle(double swing_angle)
    {
        if (swing_angle < 0)
        {
            swing_angle += 360;
        }
        swing_angle += 180;
        if (swing_angle > 360)
        {
            swing_angle -= 360;
        }
        return swing_angle;
    }

    void judge_soil_full_in_point(Eigen::Vector3d selected_unload_points_i, const param &p, const double full_average_height, std::vector<double> &unload_capacity, std::vector<SOILFULLSTATUS> &soil_full_in_point, size_t i)
    {
        // 判断当前卸载点是否装满
        Eigen::MatrixXd point_cloud_soil_truck(input_truck_lidar_data.pointcloud->points.size(), 3);
        for (Eigen::Index i = 0; i < point_cloud_soil_truck.rows(); i++)
        {
            point_cloud_soil_truck(i, 0) = input_truck_lidar_data.pointcloud->points[i].x;
            point_cloud_soil_truck(i, 1) = input_truck_lidar_data.pointcloud->points[i].y;
            point_cloud_soil_truck(i, 2) = input_truck_lidar_data.pointcloud->points[i].z;
        }

        int soilPointNum = 0;
        double total_height = 0;
        for (int r = 0; r < point_cloud_soil_truck.rows(); r++)
        {
            if ((point_cloud_soil_truck.row(r).x() - selected_unload_points_i.x()) * (point_cloud_soil_truck.row(r).x() - selected_unload_points_i.x()) +
                    (point_cloud_soil_truck.row(r).y() - selected_unload_points_i.y()) * (point_cloud_soil_truck.row(r).y() - selected_unload_points_i.y()) <
                p.select_soil_radius * p.select_soil_radius)
            {
                soilPointNum++;
                total_height += point_cloud_soil_truck.row(r).z();
            }
        }
        unload_capacity[i] = 0.0;

        if (soilPointNum > p.num_soil_point)
        {
            unload_capacity[i] = total_height / soilPointNum;
            if (unload_capacity[i] < full_average_height)
            {
                soil_full_in_point[i] = SOILFULLSTATUS::NOTFULL;
            }
            else
            {
                soil_full_in_point[i] = SOILFULLSTATUS::FULL;
            }
        }
        else
        {
            soil_full_in_point[i] = SOILFULLSTATUS::NOTFULL;
        }
    }

    void unload_point_selection(const param &p, std::vector<SOILFULLSTATUS> &soil_full_in_point, const Eigen::MatrixXd selected_unload_points, const Eigen::MatrixXd selected_unload_joints)
    {
        truck_full_flag = false;
        unload_joint = selected_unload_joints.row(1);
        unload_point = selected_unload_points.row(1);
        unload_point_index = 0;

        if (std::none_of(soil_full_in_point.begin(), soil_full_in_point.end(), [](SOILFULLSTATUS s)
                         { return s == SOILFULLSTATUS::NOTFULL; }))
        {
            ROS_INFO("Truck Full");
            truck_full_flag = true;
            unload_joint = selected_unload_joints.row(std::round(selected_unload_joints.rows() / 2));
            unload_point = selected_unload_points.row(std::round(selected_unload_points.rows() / 2));
            return;
        }
        Eigen::Index max_idx;
        selected_unload_points.col(0).maxCoeff(&max_idx);
        if (unload_times <= p.first_point_num)
        {
            unload_joint = selected_unload_joints.row(max_idx);
            unload_point = selected_unload_points.row(max_idx);
            unload_point_index = selected_unload_joints.rows() - 1;
        }
        else if (unload_times <= p.first_point_num + 2 && soil_full_in_point[max_idx] == SOILFULLSTATUS::NOTFULL)
        {
            unload_joint = selected_unload_joints.row(max_idx);
            unload_point = selected_unload_points.row(max_idx);
            unload_point_index = selected_unload_joints.rows() - 1;
        }
        else if (unload_times > p.first_point_num + 2 || soil_full_in_point[max_idx] == SOILFULLSTATUS::FULL)
        {
            soil_full_in_point[max_idx] = SOILFULLSTATUS::FULL;
            for (size_t i = selected_unload_joints.rows(); i > 0; i--)
            {
                if (soil_full_in_point[i] == SOILFULLSTATUS::NOTFULL)
                {
                    unload_joint = selected_unload_joints.row(i);
                    unload_point = selected_unload_points.row(i);
                    unload_point_index = i;
                    return;
                }
            }
        }
    }

    void generate_traj()
    {
        trajectory_msgs::JointTrajectory unload_joint_traj;
        double total_time = 4.0;
        double time_step = 1.0 / 10.0;
        size_t traj_length = static_cast<size_t>(total_time / time_step) + 1;
        unload_joint_traj.points.resize(4);
        unload_joint_traj.joint_names.resize(4);
        unload_joint_traj.header.frame_id = "base";
        unload_joint_traj.header.stamp = ros::Time::now();
        // unload_joint_traj.points.resize(traj_length);

        unload_joint_traj.joint_names[0] = "swing_traj";
        unload_joint_traj.joint_names[1] = "boom_traj";
        unload_joint_traj.joint_names[2] = "arm_traj";
        unload_joint_traj.joint_names[3] = "bkt_traj";

        std::vector<double> swing_wp = {180, 150, 120, unload_joint.x()};
        std::vector<double> boom_wp = {20, 35, 45, unload_joint.y()};
        std::vector<double> arm_wp = {-150, -135, -120, unload_joint.z()};
        std::vector<double> bkt_wp = {-100, -80, -60, unload_joint.w()};
        std::vector<double> time_interval = {0.0, 1.5, 2.5, total_time};

        for (size_t i = 0; i < 4; i++)
        {
            unload_joint_traj.points[i].positions.resize(traj_length);
        }

        for (size_t i = 0; i < traj_length; i++)
        {
            // TODO
            unload_joint_traj.points[0].positions[i] = linear(time_interval, swing_wp, static_cast<double>(i) / (traj_length - 1) * total_time);
            unload_joint_traj.points[1].positions[i] = linear(time_interval, boom_wp, static_cast<double>(i) / (traj_length - 1) * total_time);
            unload_joint_traj.points[2].positions[i] = linear(time_interval, arm_wp, static_cast<double>(i) / (traj_length - 1) * total_time);
            unload_joint_traj.points[3].positions[i] = linear(time_interval, bkt_wp, static_cast<double>(i) / (traj_length - 1) * total_time);
        }

        Eigen::Matrix<double, Eigen::Dynamic, 4> joints;
        joints.resize(traj_length, 4);

        for (size_t i = 0; i < 4; i++)
        {
            for (size_t j = 0; j < traj_length; j++)
            {
                joints(j, i) = unload_joint_traj.points[i].positions[j];
            }
        }

#if (VIS_DEBUG)
        if (VIS_DEBUG_)
        {
            Eigen::Matrix<double, Eigen::Dynamic, 3> points;
            std::cout << joints.row(joints.rows() - 1) << std::endl;
            joints_to_points(joints, points);
            std::cout << points.row(joints.rows() - 1) << std::endl;
            Eigen::Vector4d traj_color{1.0, 1.0, 0.0, 1.0};
            visualize_traj_line(unload_trajectory_pub, points, "unload_traj", 0.1, traj_color);
        }
#endif
    }

    void joints_to_points(const Eigen::Matrix<double, Eigen::Dynamic, 4> joints, Eigen::Matrix<double, Eigen::Dynamic, 3> &points)
    {
        // Convert joints(N * 4) to points (N * 3)
        // Eigen::Matrix<double, Eigen::Dynamic, 3> points(joints.rows(), 3)
        points.resize(joints.rows(), 3);
        for (int i = 0; i < joints.rows(); i++)
        {
            double swing_angle = joints(i, 0) - 180.0;
            double boom_angle = joints(i, 1);
            double arm_angle = joints(i, 2);
            double bkt_angle = joints(i, 3);

            transform_carriage_base = Eigen::Affine3f::Identity();
            transform_platform_boom = Eigen::Affine3f::Identity();
            transform_boom_arm = Eigen::Affine3f::Identity();
            transform_arm_bkt = Eigen::Affine3f::Identity();

            // Carriage
            transform_carriage_base.rotate(Eigen::AngleAxisf(swing_angle / 180 * M_PI, Eigen::Vector3f::UnitZ()) *
                                           Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                                           Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));

            // Boom
            float rotate_platform_boom_y = (-std::atan((PointF[2] / PointF[0])) * 180 / M_PI + boom_angle) / 180 * M_PI;
            transform_platform_boom.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
                                           Eigen::AngleAxisf(rotate_platform_boom_y, Eigen::Vector3f::UnitY()) *
                                           Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));
            Eigen::Vector3f PointF2 = transform_platform_boom.inverse() * PointF;

            // Arm
            Eigen::Vector3f CF = PointC - PointF2;
            Eigen::Vector3f QF = PointQ_F;
            float CFQ = std::acos(CF.dot(QF) / (CF.norm() * QF.norm())) * 180 / M_PI - 180;
            transform_boom_arm.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
                                      Eigen::AngleAxisf((-CFQ + arm_angle) / 180 * M_PI, Eigen::Vector3f::UnitY()) *
                                      Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));
            transform_boom_arm.translate(-PointF2);
            transform_boom_arm.translate(PointF2);
            Eigen::Vector3f PointQ_F2 = transform_boom_arm.inverse() * PointQ_F;

            // Bucket
            Eigen::Vector3f VQ = PointV_Q;
            Eigen::Vector3f FQ = -PointQ_F2;
            float VQF = std::acos(VQ.dot(FQ) / (VQ.norm() * FQ.norm())) * 180 / M_PI - 180;

            transform_arm_bkt.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
                                     Eigen::AngleAxisf((-VQF + bkt_angle) / 180 * M_PI, Eigen::Vector3f::UnitY()) *
                                     Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));
            Eigen::Vector3f PointV_Q2 = transform_arm_bkt.inverse() * PointV_Q;

            // Translate everying to carriage coordinate
            Eigen::Affine3f transform_carriage_translate = Eigen::Affine3f::Identity();
            transform_carriage_translate.translate(Carriage_to_Boom_Offset);

            // Calculate each point
            Eigen::Vector3f PointC_2 = PointC;
            Eigen::Vector3f PointF_2 = PointC_2 + PointF2;
            Eigen::Vector3f PointQ_2 = PointF_2 + PointQ_F2;
            Eigen::Vector3f PointV_2 = PointQ_2 + PointV_Q2;
            Eigen::Vector3f PointV_2L = PointV_2 - 0.5 * Bucket_Width * Eigen::Vector3f::UnitY();
            Eigen::Vector3f PointV_2R = PointV_2 + 0.5 * Bucket_Width * Eigen::Vector3f::UnitY();
            Eigen::Vector3f PointV_2LF = PointV_2L + 0.5 * Bucket_Length * Eigen::Vector3f::UnitX();
            Eigen::Vector3f PointV_2RF = PointV_2R + 0.5 * Bucket_Length * Eigen::Vector3f::UnitX();
            Eigen::Vector3f PointV_2RR = PointV_2R - 0.5 * Bucket_Length * Eigen::Vector3f::UnitX();
            Eigen::Vector3f PointV_2LR = PointV_2L - 0.5 * Bucket_Length * Eigen::Vector3f::UnitX();

            PointC_2 = transform_carriage_translate * PointC_2;
            PointF_2 = transform_carriage_translate * PointF_2;
            PointQ_2 = transform_carriage_translate * PointQ_2;
            PointV_2 = transform_carriage_translate * PointV_2;
            PointV_2L = transform_carriage_translate * PointV_2L;
            PointV_2R = transform_carriage_translate * PointV_2R;
            PointV_2LF = transform_carriage_translate * PointV_2LF;
            PointV_2RF = transform_carriage_translate * PointV_2RF;
            PointV_2RR = transform_carriage_translate * PointV_2RR;
            PointV_2LR = transform_carriage_translate * PointV_2LR;

            PointC_2 = transform_carriage_base * PointC_2;
            PointF_2 = transform_carriage_base * PointF_2;
            PointQ_2 = transform_carriage_base * PointQ_2;
            PointV_2 = transform_carriage_base * PointV_2;
            PointV_2L = transform_carriage_base * PointV_2L;
            PointV_2R = transform_carriage_base * PointV_2R;
            PointV_2LF = transform_carriage_base * PointV_2LF;
            PointV_2RF = transform_carriage_base * PointV_2RF;
            PointV_2RR = transform_carriage_base * PointV_2RR;
            PointV_2LR = transform_carriage_base * PointV_2LR;

            points.row(i) = Eigen::Vector3d{PointV_2.x(), PointV_2.y(), PointV_2.z()};
        }
    }

    double linear(const std::vector<double> x, const std::vector<double> y, const double input)
    {
        size_t n = x.size();
        if (x[0] >= input)
        {
            return y[0];
        }
        if (x[n - 1] <= input)
        {
            return y[n - 1];
        }
        for (size_t i = 0; i < n - 1; i++)
        {
            if (x[i] <= input && input <= x[i + 1])
            {
                double t = (input - x[i]) / (x[i + 1] - x[i]);
                return naive_lerp(y[i], y[i + 1], t);
            }
        }
        return 0.0;
    }

    double naive_lerp(double a, double b, double t)
    {
        return a + t * (b - a);
    }
};

int main(int argc, char **argv)
{
    std::string node_name = "unload_point_selection_node";
    ros::init(argc, argv, node_name);
    ROS_INFO("\033[1;32m----> %s node_name Started.\033[0m", node_name.c_str());

    UnloadPointSelection UnloadPointSelection_;

    ros::spin();
    return 0;
}