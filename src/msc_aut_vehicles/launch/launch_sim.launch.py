import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='msc_aut_vehicles' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    script_directory = '/home/mpinho/proj_ws/src/msc_aut_vehicles/nodes'

    obstacle_detector = Node(
        package=None,  # No package specified
        executable=os.path.join(script_directory, 'obstacle_detection_node.py'),  # Full path to the Python script
        name='obstacle_detector',
        output='screen'
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[
            ('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    #gazebo_params_file = os.path.join('/proj_ws/src/msc_aut_vehicles/config', 'gazebo_params.yaml')


    # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #                 launch_arguments={'world': '/usr/share/gazebo-11/worlds/cafe.world', #"/usr/share/gazebo-11/worlds/polyline.world",
    #                                   'extra_gazebo_args': '--ros-args --params-file ' +  gazebo_params_file}.items()
    #          )
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': os.path.join(get_package_share_directory(package_name),'worlds','obstacles3.world'), #"/usr/share/gazebo-11/worlds/polyline.world",
                                      'extra_gazebo_args': '--ros-args --params-file ' +  gazebo_params_file}.items()
             )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')



    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[joint_broad_spawner],
        )
    )

    slam_params = os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')

    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','online_async_launch.py'
                )]),launch_arguments={'use_sim_time': 'true','params_file': slam_params}.items()
    )

    nav2_nav_params = os.path.join(get_package_share_directory(package_name),'config','nav2_params1.yaml')

    nav2_nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )]),launch_arguments={'use_sim_time': 'true','map_subscribe_transient_local':'true','params_file': nav2_nav_params}.items()
    )

    localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','localization_launch.py'
                )]),launch_arguments={'map': os.path.join('/home/mpinho/proj_ws/map4.yaml'),'use_sim_time': 'true','params_file': nav2_nav_params}.items()
    )

    nav2_bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),'launch','bringup_launch.py'
                )]),launch_arguments={'map': os.path.join('/home/mpinho/proj_ws/map4.yaml'),'use_sim_time': 'true','params_file': nav2_nav_params}.items()
    )

    nav2_col_params = os.path.join(get_package_share_directory(package_name),'config','collision_monitor_params.yaml')

    nav2_collision = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','collision_monitor_node.launch.py'
                )]),launch_arguments={'use_sim_time': 'true','params_file': nav2_col_params}.items()
    )


    waypoint_example = Node(
        package=None,  # No package specified
        executable=os.path.join(script_directory, 'example_waypoint_follower.py'),  # Full path to the Python script
        name='waypoint_example',
        output='screen'
    )    

    resource_node = Node(
        package=None,  # No package specified
        executable=os.path.join(script_directory, 'resource_check.py'),  # Full path to the Python script
        name='resource_check',
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        #obstacle_detector,
        twist_mux,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        slam,
        nav2_nav
        #nav2_bringup
        #localization
        #nav2_collision
        #waypoint_example
        #resource_node
    ])