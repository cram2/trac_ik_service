from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from rclpy.parameter import Parameter
from launch_ros.substitutions import FindPackageShare
import os
import rclpy

class TestIK(Node):
    def __init__(self):
        super().__init__('test_ik',
                         allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        pkg_share = FindPackageShare('trac_ik_examples').find('trac_ik_examples')
        urdf_file = os.path.join(pkg_share, 'launch', 'pr2.urdf')
        with open(urdf_file, 'r') as infp:
            robot_desc = infp.read()

        # self.declare_parameter('robot_description'," Parameter.Type.STRING")
        self.declare_parameter(
            "robot_description",
            robot_desc)
        # Then set the parameter "my_parameter" back to string value "world"
        my_new_param = rclpy.parameter.Parameter(
            "robot_description",
            rclpy.Parameter.Type.STRING,
            robot_desc
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

        self.req = GetPositionIK.Request()
        self.req.ik_request.pose_stamped.header.frame_id = "torso_lift_link"
        self.req.ik_request.ik_link_name = "r_wrist_roll_link"

        self.cli = self.create_client(GetPositionIK, 'get_ik')
        self.test_solutions()

    def test_solutions(self):

        for x in range(5, 10, 1):
            for y in range(5, 10, 1):
                for z in range(5, 10, 1):
                    self.req.ik_request.pose_stamped.pose.position.x = x/10
                    self.req.ik_request.pose_stamped.pose.position.y = y/10
                    self.req.ik_request.pose_stamped.pose.position.z = z/10
                    self.req.ik_request.pose_stamped.pose.orientation.w = 1.0
                    self.req.ik_request.pose_stamped.pose.orientation.x = 0.0
                    self.req.ik_request.pose_stamped.pose.orientation.y = 0.0
                    self.req.ik_request.pose_stamped.pose.orientation.z = 0.0

                    response = self.cli.call(self.req)
                    print(response)

def main(args=None):
    rclpy.init(args=args)

    minimal_service = TestIK()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
