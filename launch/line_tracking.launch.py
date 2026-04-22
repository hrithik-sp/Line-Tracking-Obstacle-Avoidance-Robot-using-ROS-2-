import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg       = get_package_share_directory('line_tracking_avoidance')
    world     = os.path.join(pkg, 'worlds', 'line_track.sdf')
    tb3_gz    = get_package_share_directory('turtlebot3_gazebo')
    tb3_desc  = get_package_share_directory('turtlebot3_description')
    robot_sdf = os.path.join(tb3_gz, 'models', 'turtlebot3_waffle', 'model.sdf')
    urdf_file = os.path.join(tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    res_path = os.path.join(tb3_gz, 'models') + ':/opt/ros/jazzy/share'

    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    gz_env  = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', res_path)

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        additional_env={
            'GZ_SIM_RESOURCE_PATH': res_path,
            'RMW_IMPLEMENTATION':   'rmw_cyclonedds_cpp'
        },
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # ── BRIDGE ───────────────────────────────────────────────────────────────
    # The TurtleBot3 Waffle camera sensor publishes on Gazebo as:
    #   /camera/image_raw
    # We bridge it to ROS 2 and remap it to /camera/rgb/image_raw
    # so line_detector.py receives it on the correct topic.
    #
    # If after launch `ros2 topic list` does NOT show /camera/rgb/image_raw,
    # run: gz topic -l | grep camera   (in a new terminal while Gazebo is running)
    # and adjust the gz topic name in the argument below accordingly.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/camera/image_raw', '/camera/rgb/image_raw'),
        ],
        output='screen'
    )

    # ── SPAWN ROBOT ──────────────────────────────────────────────────────────
    # World name MUST match <world name="line_track_world"> in line_track.sdf ✅
    spawn = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/line_track_world/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000',
             '--req',
             f'sdf_filename: "{robot_sdf}", name: "turtlebot3_waffle", '
             f'pose: {{position: {{x: -1.5, y: 0.0, z: 0.05}}, '
             f'orientation: {{x: 0, y: 0, z: 0, w: 1}}}}'],
        output='screen'
    )

    # ── NODES — all get use_sim_time so timing is consistent with Gazebo ─────
    line_det = Node(
        package='line_tracking_avoidance',
        executable='line_detector',
        output='screen',
        parameters=[{
            'roi_top_ratio': 0.55,
            'min_area': 180,
            'use_sim_time': True
        }]
    )
    obs_det = Node(
        package='line_tracking_avoidance',
        executable='obstacle_detector',
        output='screen',
        parameters=[{
            'safe_distance': 0.5,
            'front_fov_deg': 60.0,
            'side_fov_deg': 70.0,
            'use_sim_time': True
        }]
    )
    ctrl = Node(
        package='line_tracking_avoidance',
        executable='controller',
        output='screen',
        parameters=[{
            'Kp':                    1.0,
            'Kd':                    0.3,
            'base_speed':            0.18,
            'max_angular':           0.5,
            'avoid_speed':           0.18,
            'turn_speed':            0.5,
            'turn_duration':         3.0,
            'strafe_time':           2.5,
            'pass_time':             5.5,
            'search_turn_speed':     0.35,
            'max_no_line_sec':       0.8,
            'obstacle_trigger_count': 3,
            'return_linear':         0.10,
            'return_angular':        0.35,
            'use_sim_time':          True,
        }]
    )

    return LaunchDescription([
        set_rmw, gz_env,
        gazebo,
        rsp,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawn]),
        TimerAction(period=8.0, actions=[line_det, obs_det, ctrl]),
    ])