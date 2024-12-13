#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "trac_ik/trac_ik.hpp"
#include "tf2_kdl/tf2_kdl.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"


#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <memory>

std::shared_ptr<rclcpp::Node> node;

void get_ik(const std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> request,
          std::shared_ptr<moveit_msgs::srv::GetPositionIK::Response>      response)
{
//    node->declare_parameter("robot_description", "test");
    std::string urdf_xml;

    std::string end_effector_link = request->ik_request.ik_link_name;
    std::string base_link = request->ik_request.pose_stamped.header.frame_id;
    double timeout = 0.005;
    double eps = 1e-2;

    node->get_parameter("robot_description", urdf_xml);

//    RCLCPP_INFO(node->get_logger(), "Using %s as URDF", urdf_xml.c_str());

    // This constructor parses the URDF loaded in rosparm urdf_xml into the
    // needed KDL structures.  We then pull these out to compare against the KDL
    // IK solver.
    TRAC_IK::TRAC_IK tracik_solver(base_link, end_effector_link, urdf_xml, timeout, eps);

    KDL::Chain chain;
    KDL::JntArray ll, ul;  // lower joint limits, upper joint limits

    bool valid = tracik_solver.getKDLChain(chain);

    if (!valid) {
        RCLCPP_ERROR(node->get_logger(), "There was no valid KDL chain found");
        return;
    }

    valid = tracik_solver.getKDLLimits(ll, ul);

    if (!valid) {
        RCLCPP_ERROR(node->get_logger(), "There were no valid KDL joint limits found");
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Chain is valid");
    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

//     Create Nominal chain configuration midway between all joint limits
    KDL::JntArray nominal(chain.getNrOfJoints());

    for (uint j = 0; j < request->ik_request.robot_state.joint_state.position.size(); j++) {
        nominal(j) = request->ik_request.robot_state.joint_state.position[j];
        RCLCPP_INFO(node->get_logger(), "Joint %d: %f", j, nominal(j));
    }

    KDL::Frame end_effector_pose;
    KDL::JntArray result;
    RCLCPP_INFO(node->get_logger(), "Transforming pose to KDL frame");

    tf2::fromMsg(request->ik_request.pose_stamped.pose, end_effector_pose);

    RCLCPP_INFO(node->get_logger(), "Calculating IK solution");

    // Calculating the actual ik solution
    int rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);

    RCLCPP_INFO(node->get_logger(), "Found %d solutions", rc);
    if(rc > 0){
        response->error_code.val = response->error_code.SUCCESS;
        RCLCPP_INFO(node->get_logger(), "Writing solution to response");

         for(uint i = 0; i < result.rows(); i++){
         response->solution.joint_state.position.push_back(result(i));
            response->solution.joint_state.name.push_back(request->ik_request.robot_state.joint_state.name[i]);
//            response->solution.joint_state.position[i] = result(i);
//            response->solution.joint_state.name = request->ik_request.robot_state.joint_state.name;
        }
    }
    else{
    response->error_code.val = response->error_code.NO_IK_SOLUTION;
        RCLCPP_INFO(node->get_logger(), "No solution found");
    }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("trac_ik_server");
  node->declare_parameter("robot_description", std::string());

  rclcpp::Service<moveit_msgs::srv::GetPositionIK>::SharedPtr service =
    node->create_service<moveit_msgs::srv::GetPositionIK>("get_ik", &get_ik);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready calculate ik.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(0);
}