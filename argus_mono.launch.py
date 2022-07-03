# ROS2 launch file for 1xCSI camera, plus 50% resize and rectification
# all using the Nvidia ISAAC accelerated functions

# Issues:
# Poor performance, due to CPU-GPU memory shuffling between nodes
# Incorrect image rectification

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    argusRight = Node(
        package='isaac_ros_argus_camera_mono',
        executable='isaac_ros_argus_camera_mono',
        name='camRight',
        remappings=[
            ("/image_raw", "/camRight/image_raw"),
            ("/image_raw/compressed", "/camRight/image_raw/compressed"),
            ("/image_raw/compressedDepth", "/camRight/image_raw/compressedDepth"),
            ("/camera_info", "/camRight/camera_info")
        ],
        parameters=[{
            'sensor': 4,
            'device': 0,
            'output_encoding': 'rgb8',
            'camera_info_url': 'file:///docker_map/right-720p-IMX219.yaml'
        }]
    )

    argusLeft = Node(
        package='isaac_ros_argus_camera_mono',
        executable='isaac_ros_argus_camera_mono',
        name='camLeft',
        remappings=[
            ("/image_raw", "/camLeft/image_raw"),
            ("/image_raw/compressed", "/camLeft/image_raw/compressed"),
            ("/image_raw/compressedDepth", "/camLeft/image_raw/compressedDepth"),
            ("/camera_info", "/camLeft/camera_info")
        ],
        parameters=[{
            'sensor': 4,
            'device': 1,
            'output_encoding': 'rgb8',
            'camera_info_url': 'file:///docker_map/left-720p-IMX219.yaml'
        }]
    )

    rectifyLeft = ComposableNode(
        name='rectifyLeft',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::RectifyNode',
        remappings=[('/image', '/camLeftSmall/image'),
                    ('/camera_info', '/camLeftSmall/camera_info'),
                    ('/image_rect', '/camLeftSmall/image_rect')],
        parameters=[{
            'backends': 'CUDA'
        }]
        )
    rectifyRight = ComposableNode(
        name='rectifyRight',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::RectifyNode',
        remappings=[('/image', '/camRightSmall/image'),
                    ('/camera_info', '/camRightSmall/camera_info'),
                    ('/image_rect', '/camRightSmall/image_rect')],
        parameters=[{
            'backends': 'CUDA'
        }]
        )

    resizeRight = ComposableNode(
        name='resizeRight',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ResizeNode',
        remappings=[('/image', '/camRight/image_raw'),
                    ('/camera_info', '/camRight/camera_info'),
                    ('/resized/image', '/camRightSmall/image'),
                    ('/resized/camera_info', '/camRightSmall/camera_info')],
        parameters=[{
            'backends': 'CUDA',
            'scale_height': 0.5,
            'scale_width': 0.5
        }]
        )
    resizeLeft = ComposableNode(
        name='resizeLeft',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ResizeNode',
        remappings=[('/image', '/camLeft/image_raw'),
                    ('/camera_info', '/camLeft/camera_info'),
                    ('/resized/image', '/camLeftSmall/image'),
                    ('/resized/camera_info', '/camLeftSmall/camera_info')],
        parameters=[{
            'backends': 'CUDA',
            'scale_height': 0.5,
            'scale_width': 0.5
        }]
        )

    argus_rectify_container = ComposableNodeContainer(
        name='argus_rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            #rectifyLeft,
            rectifyRight,
            resizeRight,
            #resizeLeft
        ],
        output='screen'
    )

    return launch.LaunchDescription([argusRight, argus_rectify_container])
