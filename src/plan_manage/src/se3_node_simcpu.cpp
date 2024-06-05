#include <plan_manage/se3_planner.h>

ros::Subscriber waypoints_sub, _odom_sub;
Eigen::Vector3d start_pos, start_vel, start_acc;
Eigen::Vector3d endpos, endvel;
Eigen::Vector3d odom_position;
Eigen::Vector3d odom_velocity;
Eigen::Vector3d Zero3d(0, 0, 0);
Eigen::MatrixXd initState, finState;
std::vector<Eigen::Vector3d> gate_list;
Eigen::Vector3d target_pt;
std::mutex startpos_mutex;
void rcvWaypointsCallback(const nav_msgs::Path& wp);
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
MavGlobalPlanner* glbPlanner;
int main(int argc, char** argv) {
    ros::init(argc, argv, "map_se3_sim_node");
    start_pos << 0, 0, 1.0;
    start_vel << 0, 0, 0;
    start_acc << 0, 0, 0;
    endvel << 0, 0, 0;
    initState.resize(3, 3);
    finState.resize(3, 3);
    Config config;
    ros::NodeHandle nh_priv("~");
    waypoints_sub = nh_priv.subscribe("waypoints", 1, rcvWaypointsCallback);
    _odom_sub = nh_priv.subscribe("odom", 1, odom_callback);
    config.loadParameters(ros::NodeHandle("~"));
    glbPlanner = new MavGlobalPlanner(config, nh_priv);
    /*
    1. 读取中间路径点位置
    2. waypoints_sub 设置终点 并且用于启动全局规划器
    3. 全局规划期接下来会计算路径

    */

    /*obtain the gate center pose*/
    //获得每个gate 的位置
    std::vector<string> gates_list;

    // 人为设置途径gate 中间路径点
    Eigen::Vector3d gate_wp;
    // nh_priv.getParam("gate_wp_x", gate_wp(0));
    // nh_priv.getParam("gate_wp_y", gate_wp(1));
    // nh_priv.getParam("gate_wp_z", gate_wp(2));
    gate_wp << 10, 0 , 1.0;
    gate_list.push_back(gate_wp);
    sleep(2);
    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
/*
    1. 设置终点
    2. 启动全局规划器
*/
void rcvWaypointsCallback(const nav_msgs::Path& wp) {
    if (wp.poses[0].pose.position.z < 0.0) return;
    target_pt << wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, 1.0;
    // 使用线程琐保护 start_pos资源， 防止odom_callback修改start_pos
    // {
    //     std::lock_guard<std::mutex> lock(startpos_mutex);
    start_pos = odom_position;
    ROS_INFO_STREAM("START=" << start_pos);
    ROS_INFO_STREAM("TARGET=" << target_pt);
    ROS_INFO("[node] receive the planning target");
    initState << start_pos, Zero3d, Zero3d;
    finState << target_pt, Zero3d, Zero3d;
    if (!gate_list.empty()) {
        for (auto& gate : gate_list) {
            ROS_INFO_STREAM("gate=" << gate);
        }
    } else {
        ROS_INFO("no gate");
    }
    // go out of definition
    // the lock_guard will be destroyed and the mutex will be unlocked
    // destory the lock_guard
    // }
    glbPlanner->plan(initState, finState, &gate_list);
}

/*
    1. 读取当前位置
    2. 读取当前速度
    3. 读取当前姿态
    4. 下一次启动se3的时候，将当前的位置作为起始位置
*/
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    const Eigen::Vector3d position(odom->pose.pose.position.x,
                                   odom->pose.pose.position.y,
                                   odom->pose.pose.position.z);
    const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                   odom->twist.twist.linear.y,
                                   odom->twist.twist.linear.z);
    odom_position = position;
    odom_velocity = velocity;
    Eigen::Quaterniond q;
    q.w() = odom->pose.pose.orientation.w;
    q.x() = odom->pose.pose.orientation.x;
    q.y() = odom->pose.pose.orientation.y;
    q.z() = odom->pose.pose.orientation.z;
    q = q.normalized();
    //下一次启动se3的时候，将当前的位置作为起始位置
    // {
    // std::lock_guard<std::mutex> lock(startpos_mutex);
    // start_pos = position;
    // }
}
