#include "ros/ros.h"
#include <esr_msgs/Track.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <cstdlib>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

#define BETA 0
#define SENSOR_NUMBER 3
#define DES_X 0.88

std::vector<esr_msgs::Track> sensor_data[SENSOR_NUMBER];
std::vector<geometry_msgs::Vector3> tf_vector(SENSOR_NUMBER);
std::vector<geometry_msgs::Vector3> tf_rpy(SENSOR_NUMBER);
std::vector<int> data_size(3, 0);
std::vector<bool> complet(3, 0);
float last_v;
Eigen::MatrixXd last_ego_mat(3, 1);
int msg_count;
bool beginning;
static Eigen::Vector3d current_XYZ(0, 0, 0);
static Eigen::Vector3d current_RPY(0, 0, 0);

// Declare publisher
ros::Publisher moving_obj;
ros::Publisher static_obj;
ros::Publisher moving_obj_rate;
ros::Publisher static_obj_pc;
ros::Publisher ego_motion;
ros::Publisher esr_left_obstacles;
ros::Publisher esr_right_obstacles;

// Declare IMU parameter
static ros::Publisher marker_pub;
static ros::Time current_time;
static ros::Time previous_time(0, 0);
static ros::Time imu_previous_time;
static Eigen::Vector3d acc_global;
static Eigen::Vector3d velocity(0, 0, 0);

Eigen::MatrixXd deleteCols(Eigen::MatrixXd mat, std::vector<int> cols) {
    Eigen::MatrixXd rm_mat(cols.size(), mat.cols());
    int current_row = 0, current_vec = 0;
    for (int i = 0; i < rm_mat.rows(); i++)
        rm_mat.row(i) = mat.row(cols[i]);
    return rm_mat;
}

std::vector<int> calcRansacMat(Eigen::MatrixXd transfer_mat, Eigen::MatrixXd velocity_mat, int size, double threshold) {
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = size;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = transfer_mat(i, 1);
        cloud->points[i].y = transfer_mat(i, 2);
        cloud->points[i].z = velocity_mat(i, 0);
    }
    std::vector<int> inliers;
    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

    // Do RANSAC
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setMaxIterations(1000);
    ransac.setDistanceThreshold(threshold);//0.05 in low mode 0.005 in highway mode
    ransac.computeModel();
    ransac.getInliers(inliers);
    return inliers;
}

std::vector<int> findStaticObject(Eigen::MatrixXd transfer_mat, Eigen::MatrixXd velocity_mat, Eigen::MatrixXd plant_mat, int size, double threshold) {
    Eigen::MatrixXd v_mat(size, 1);
    std::vector<int> inliers;
    std::vector<int> selection;
    v_mat = plant_mat * last_ego_mat;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    last_cloud->width = size;
    last_cloud->height = 1;
    last_cloud->is_dense = false;
    last_cloud->points.resize(last_cloud->width * last_cloud->height);

    for (size_t i = 0; i < 3; ++i) {
        last_cloud->points[i].x = plant_mat(i, 1);
        last_cloud->points[i].y = plant_mat(i, 2);
        last_cloud->points[i].z = v_mat(i, 0);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = size;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = transfer_mat(i, 1);
        cloud->points[i].y = transfer_mat(i, 2);
        cloud->points[i].z = velocity_mat(i, 0);
    }

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr last_model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(last_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> last_sac(last_model);

    selection.push_back(0);
    selection.push_back(1);
    selection.push_back(2);

    Eigen::VectorXf model_coefficients;
    last_sac.sac_model_->computeModelCoefficients (selection, model_coefficients);

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> sac(model);
    sac.sac_model_->countWithinDistance (model_coefficients, threshold);
    sac.sac_model_->selectWithinDistance (model_coefficients, threshold, inliers);

    return inliers;
}

void calcMotion(int can_num) {
    std::vector<int> RANSAC_v;
    //std::vector<esr_msgs::Track> tmp_v;
    ros::NodeHandle n;
    Eigen::MatrixXd ego_mat(3, 1);
    Eigen::MatrixXd azimuth_mat;
    Eigen::Matrix2d beta_mat;

    //Include data to matrix
    for (int j = 0; j < data_size[can_num]; j++) {
        azimuth_mat.resize(data_size[can_num], 2);
        azimuth_mat(j, 0) = cos(sensor_data[can_num][j].azimuth / 180 * M_PI);
        azimuth_mat(j, 1) = sin(sensor_data[can_num][j].azimuth / 180 * M_PI);
    }

    //Beta matrix
    double beta = tf_rpy[can_num].z - 2.15;
    beta_mat.resize(2, 2);
    beta_mat(0, 0) = cos(beta);
    beta_mat(0, 1) = sin(beta);
    beta_mat(1, 0) = -sin(beta);
    beta_mat(1, 1) = cos(beta);

    //Global matrix
    Eigen::MatrixXd global_mat;
	if (data_size[can_num] == 0) return;
    else {
        global_mat.resize(data_size[can_num], 2);
        global_mat = azimuth_mat * beta_mat;
    }
    //Sensor matrix
   Eigen::MatrixXd sensor_mat;
    sensor_mat.resize(2, 3);
    sensor_mat(0, 0) = (-1) * tf_vector[can_num].y; //y
    sensor_mat(0, 1) = 1;
    sensor_mat(0, 2) = 0;
    sensor_mat(1, 0) = 1 * (tf_vector[can_num].x + DES_X); //x
    sensor_mat(1, 1) = 0;
    sensor_mat(1, 2) = 1;
    //Transfer matrix
    Eigen::MatrixXd transfer_m;
	if (data_size[can_num] != 0) {
        transfer_m.resize(data_size[can_num], 3);
        transfer_m = global_mat * sensor_mat;
    } else
    return;
    //Combine to matrix
    Eigen::MatrixXd transfer_mat(data_size[can_num], 3);
    Eigen::MatrixXd velocity_mat(data_size[can_num], 1);

    Eigen::MatrixXd plant_mat(3, 3);

    Eigen::MatrixXd plant_azimuth_mat(3, 2);
    plant_azimuth_mat(0, 0) = cos(0 / 180 * M_PI);
    plant_azimuth_mat(0, 1) = sin(0 / 180 * M_PI);
    plant_azimuth_mat(1, 0) = cos(45.f / 180 * M_PI);
    plant_azimuth_mat(1, 1) = sin(45.f / 180 * M_PI);
    plant_azimuth_mat(2, 0) = cos(-45.f / 180 * M_PI);
    plant_azimuth_mat(2, 1) = sin(-45.f / 180 * M_PI);
    plant_mat = plant_azimuth_mat * beta_mat * sensor_mat;

    for (int i = 0; i < data_size[can_num]; i++) {
        velocity_mat(i, 0) = sensor_data[can_num][i].range_rate * (-1);
    }

    //cout << "velocity_mat: " << velocity_mat << endl << endl;
    transfer_mat = transfer_m;

    Eigen::MatrixXd RANSAC_transfer_mat;
    Eigen::MatrixXd RANSAC_velocity_mat;
    //Do RANSAC

    if (data_size[can_num] > 2) {
		if (beginning) {
            RANSAC_v = calcRansacMat(transfer_mat, velocity_mat, data_size[can_num], 0.1);
	    beginning = false;
	   } else
        RANSAC_v = findStaticObject(transfer_mat, velocity_mat, plant_mat, data_size[can_num], 0.1); //0.1

	if (RANSAC_v.size() < 1) {
            //std::cout << "RANSAC size: " << RANSAC_v.size() << std::endl;
        }
	else {
            //Matrix after RANSAC
            RANSAC_transfer_mat.resize(data_size[can_num] - RANSAC_v.size(), 3);
            RANSAC_velocity_mat.resize(data_size[can_num] - RANSAC_v.size(), 1);

            //Remove static data
            RANSAC_transfer_mat = deleteCols(transfer_mat, RANSAC_v);
            RANSAC_velocity_mat = deleteCols(velocity_mat, RANSAC_v);

            //Compute Ego-Motion
            ego_mat = RANSAC_transfer_mat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(RANSAC_velocity_mat);
            //std::cout << "ev:" << ego_mat(1, 0) << std::endl;
	}
    } else {
        RANSAC_transfer_mat.resize(1, 3);
        RANSAC_velocity_mat.resize(1, 1);
        RANSAC_transfer_mat = transfer_mat;
        RANSAC_velocity_mat = velocity_mat;
    }

    if(data_size[can_num] == 0)
        return;
    if (RANSAC_v.size() > 4) {
	last_ego_mat(0, 0) = ego_mat(0, 0);
	last_ego_mat(1, 0) = ego_mat(1, 0);
	last_ego_mat(2, 0) = ego_mat(2, 0);
    	last_v = -1 * ego_mat(1, 0);
    }

    //Publish Ego-Motion
    geometry_msgs::Vector3 ego_m;
    ego_m.x = ego_mat(0, 0);
    ego_m.y = ego_mat(1, 0);
    ego_m.z = ego_mat(2, 0);
    ego_motion.publish(ego_m);


    //Initialize to publish pointcloud2
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    cloud2->width = 0;//RANSAC_v.size();
    cloud2->height = 1;
    cloud2->is_dense = false;
    cloud2->points.resize(cloud2->width * cloud2->height);
    cloud2->header.stamp = pcl_conversions::toPCL(sensor_data[can_num][0].header.stamp);
    int p_num = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //visualization the track objects
    float est_V[data_size[can_num]];
    Eigen::MatrixXd tmp_mat(2, 1);
    tmp_mat.resize(data_size[can_num], 1);
    tmp_mat = transfer_m * last_ego_mat;
    for (int j = 0; j < data_size[can_num]; j++) {
        int flag = 0;
        double r = sensor_data[can_num][j].azimuth * M_PI / 180;
        est_V[j] = (-1) * tmp_mat(j,0);

        //Determine static object by RANSAC
        int num = j;
        for (int a = 0; a < can_num; a++)
            num += data_size[a];

        for (int k = 0; k < RANSAC_v.size(); k++)
            if (num == RANSAC_v[k]) {
                flag = 1;
                break;
            }

        //Cylinder marker for object
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/radar_map";
        marker.header.stamp = sensor_data[can_num][j].header.stamp;
        marker.ns = "track";
        marker.id = sensor_data[can_num][j].index;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.1); // 0.5 seconds
        marker.pose.position.x = sensor_data[can_num][j].range * std::cos(r + tf_rpy[can_num].z);
        marker.pose.position.y = sensor_data[can_num][j].range * std::sin(r + tf_rpy[can_num].z);
        marker.pose.position.z = tf_vector[can_num].z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 2.0;
        marker.color.a = 1.f;
        if(!flag){
        //Moving object
            marker.color.b = 0;
            marker.color.g = 1.f;
            marker.color.r = 0;
            moving_obj.publish(marker);
        } else {
        //Static object
            marker.color.b = 0;
            marker.color.g = 1.f;
            marker.color.r = 1.f;
            static_obj.publish(marker);
        //publish pointcloud2
        	pcl::PointXYZI point2;
        	point2.x = marker.pose.position.x;
        	point2.y = marker.pose.position.y;
        	point2.z = marker.pose.position.z;
			point2.intensity = sensor_data[can_num][j].amplitude;
            cloud2->width++;
        	cloud2->push_back(point2);
        }

        //publish obstacle pointcloud
        pcl::PointXYZ point;
        point.x = marker.pose.position.x;
        point.y = marker.pose.position.y;
        point.z = marker.pose.position.z;
        float est_point_x, est_point_y;
        est_point_x = point.x * std::cos(M_PI*0.2) + point.y * std::sin(M_PI*0.2);
        est_point_y = point.y * std::cos(M_PI*0.2) - point.x * std::sin(M_PI*0.2);
        if (fabs(est_point_x) > 5 || fabs(est_point_y) > 40);
        else {
            if (est_point_x > 0)
                right_cloud->push_back(point);
            else
                left_cloud->push_back(point);
        }

        //Calculate orientation
        geometry_msgs::Quaternion q_rot;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(tf_rpy[can_num].x, tf_rpy[can_num].y, r + tf_rpy[can_num].z),q_rot);
        //Arrow marker for velocity
        visualization_msgs::Marker pose;
        pose.header.frame_id = "/radar_map";
        pose.header.stamp = sensor_data[can_num][j].header.stamp;
        pose.ns = "track";
        pose.id = sensor_data[can_num][j].index;
        pose.lifetime = ros::Duration(0.1); // 0.5 seconds
        pose.type = visualization_msgs::Marker::ARROW;
        pose.action = visualization_msgs::Marker::ADD;
    	pose.scale.x = 2 * (sensor_data[can_num][j].range_rate - est_V[j]);
      	pose.scale.y = 0.5;
        pose.scale.z = 1.0;
        pose.color.a = 1.f;
        pose.color.b = 0;
        pose.color.g = 1.f;
        pose.color.r = 0;
        pose.pose.position.x = sensor_data[can_num][j].range * std::cos(r + tf_rpy[can_num].z);
        pose.pose.position.y = sensor_data[can_num][j].range * std::sin(r + tf_rpy[can_num].z);
        pose.pose.position.z = 0.0;
        pose.pose.orientation = q_rot;
        moving_obj_rate.publish(pose);
    }

    // Implicit convert by pcl_ros/point_cloud.h
    cloud2->header.frame_id = "/radar_map";
    left_cloud->header.frame_id = "/radar_map";
    right_cloud->header.frame_id = "/radar_map";

    // Publish the data
    static_obj_pc.publish(cloud2);
    esr_left_obstacles.publish(left_cloud);
    esr_right_obstacles.publish(right_cloud);
}

class can_message {
    private:
        int can_num;

    public:
        void callback(const esr_msgs::Track&);
        can_message(int num) {can_num = num;}
};

void can_message::callback(const esr_msgs::Track& msg)
{
    int can = can_num;
    int size = sensor_data[can].size() - 1;
    sensor_data[can].push_back(msg);

    //Have a completely matrix
    if (sensor_data[can][size].header.stamp != msg.header.stamp && size > 0) {
        data_size[can] = sensor_data[can].size() - 1;
	    calcMotion(can);
        // clean the sensor data
        sensor_data[can].erase(sensor_data[can].begin(),sensor_data[can].begin() + data_size[can]);
        data_size[can] = 0;
    }
};

void ImuCallback(const sensor_msgs::Imu::ConstPtr &input) {
    current_time = input->header.stamp;

    if (imu_previous_time.toSec() == 0)
        imu_previous_time = current_time;

    static Eigen::Vector3d gravity (-0.19641456008 ,0.10873106122 ,9.83648490906);
    //(input->linear_acceleration.x, input->linear_acceleration.y, input->linear_acceleration.z);

    double dt = (current_time - imu_previous_time).toSec();
    velocity += (acc_global - gravity) * dt;

    last_ego_mat(0, 0) += input->angular_velocity.z * dt;
    last_ego_mat(1, 0) += input->linear_acceleration.x * dt;
    last_ego_mat(2, 0) += input->linear_acceleration.y * dt;

    imu_previous_time = current_time;
    // cout << "Ego-Motion" << endl;
    // cout << "w: " << last_ego_mat(0, 0) << "    vx: " << last_ego_mat(1, 0) << "     vy: " << last_ego_mat(2, 0) << endl;
    // cout << endl;
}

void tfCallback(const  tf2_msgs::TFMessage& m)
{
    for (int i = 0; i < m.transforms.size(); i++){
        geometry_msgs::TransformStamped msg = m.transforms[i];
        int can_num;
        if (msg.child_frame_id == "/esr_can0_frame")
            can_num = 0;
        else if (msg.child_frame_id == "/esr_can1_frame")
            can_num = 1;
        else if (msg.child_frame_id == "/esr_can2_frame")
            can_num = 2;
        else
            continue;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.transform.rotation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        tf_rpy[can_num].x = roll;
        tf_rpy[can_num].y = pitch;
        tf_rpy[can_num].z = yaw;
        tf_vector[can_num].x = msg.transform.translation.x;
        tf_vector[can_num].y = msg.transform.translation.y;
        tf_vector[can_num].z = msg.transform.translation.z;
    }
};

int main(int argc, char **argv)
{
    last_v = 0;
    msg_count = 0;
    beginning = true;
    ros::init(argc, argv, "radar_ego_motion");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    moving_obj = n.advertise<visualization_msgs::Marker> ("esr_moving_obj", 100);
    static_obj = n.advertise<visualization_msgs::Marker> ("esr_static_obj", 100);
    moving_obj_rate = n.advertise<visualization_msgs::Marker> ("esr_moving_obj_rate", 100);
    static_obj_pc = n.advertise<sensor_msgs::PointCloud2> ("esr_static_obj_pc", 100);
    ego_motion = n.advertise<geometry_msgs::Vector3> ("esr_ego_motion", 100);
    esr_left_obstacles = n.advertise<sensor_msgs::PointCloud2> ("esr_left_obstacles", 100);
    esr_right_obstacles = n.advertise<sensor_msgs::PointCloud2> ("esr_right_obstacles", 100);
    ros::Subscriber sub_tf = n.subscribe("tf_static", 10, tfCallback);

    can_message can0_message(0), can1_message(1), can2_message(2);
    ros::Subscriber sub_can0 = n.subscribe("esr_can0_tracks", 100, &can_message::callback, &can0_message);
    ros::Subscriber sub_can1 = n.subscribe("esr_can1_tracks", 100, &can_message::callback, &can1_message);
    ros::Subscriber sub_can2 = n.subscribe("esr_can2_tracks", 100, &can_message::callback, &can2_message);
    ros::Subscriber sub = n.subscribe("/imu/data", 1, ImuCallback);
    ros::spin();

  return 0;
}
