from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # QoS Setting Node - Basic version
        Node(
            package='rover_middleware',
            executable='qos_convert',
            name='qos_setting_basic',
            output='screen',
            remappings=[
                # Remap topics if needed
            ]
        ),
        
        # QoS Setting Node - Advanced version dengan image_transport
        Node(
            package='rover_middleware',
            executable='qos_setting_advanced',
            name='qos_setting_advanced_node',
            output='screen',
            parameters=[{
                'input_topic': '/camera_image_gray',
                'output_topic': '/topic_best_effort',
                'queue_size': 10,
                'input_reliable': True,
                'output_reliable': False,
                'transport_hint': 'raw'  # raw, compressed, atau auto
            }],
            remappings=[
                # Remap topics if needed
            ]
        ),
        
        # QoS Setting Node - Hybrid version dengan custom QoS + compressed support
        Node(
            package='rover_middleware',
            executable='qos_setting_hybrid',
            name='qos_setting_hybrid_node',
            output='screen',
            parameters=[{
                'input_topic': '/camera_image_gray',
                'output_topic': '/topic_best_effort',
                'queue_size': 10,
                'input_reliable': True,
                'output_reliable': False,
                'use_compressed': False  # true untuk compressed, false untuk raw
            }],
            remappings=[
                # Remap topics if needed
            ]
        ),
    ])
