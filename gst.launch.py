# ROS2 launch file for 2x CSI Cameras on Jetson, with software
# rectification
# Using gscam2 so we can limit the framerate to a sensible number,
# as at 720p, we get 30+ FPS minimum

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    GSCamRight = Node(
        package='gscam2',
        executable='gscam_main',
        name='GSCamR',
        namespace='Right',
        remappings=[
        ],
        parameters=[{
            'gscam_config': 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=(int)1280,height=(int)720 ! nvvidconv flip-method=0 ! video/x-raw,format=(string)I420 ! videorate ! video/x-raw,framerate=10/1 ! videoconvert',
            'sync_sink': True,
            'frame_id': '/R_frame',
            #'camera_name': 'Rcam',
            'camera_info_url': 'file:///docker_map/right-720p-IMX219.yaml'
        }]
    )

    GSCamLeft = Node(
        package='gscam2',
        executable='gscam_main',
        name='GSCamL',
        namespace='Left',
        remappings=[
        ],
        parameters=[{
            'gscam_config': 'nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM),width=(int)1280,height=(int)720 ! nvvidconv flip-method=0 ! video/x-raw,format=(string)I420 ! videorate ! video/x-raw,framerate=10/1 ! videoconvert',
            'sync_sink': True,
            'frame_id': '/L_frame',
            #'camera_name': 'Lcam',
            'camera_info_url': 'file:///docker_map/left-720p-IMX219.yaml'
        }]
    )

    rectifyRight = ComposableNode(
        name='rectifyRight',
        package='image_proc',
        plugin='image_proc::RectifyNode',
        remappings=[('/image', '/Right/image_raw'),
                    ('/camera_info', '/Right/camera_info'),
                    ('/image_rect', '/Right/image_rect')],
        parameters=[{
            #'backends': 'CUDA'
        }]
        )

    rectifyLeft = ComposableNode(
        name='rectifyLeft',
        package='image_proc',
        plugin='image_proc::RectifyNode',
        remappings=[('/image', '/Left/image_raw'),
                    ('/camera_info', '/Left/camera_info'),
                    ('/image_rect', '/Left/image_rect')],
        parameters=[{
            #'backends': 'CUDA'
        }]
        )

    rectify_container = ComposableNodeContainer(
        name='rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rectifyLeft,
            rectifyRight
        ],
        output='screen'
    )

    return launch.LaunchDescription([GSCamLeft, GSCamRight, rectify_container])
