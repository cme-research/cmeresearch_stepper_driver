from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmeresearch_stepper_driver',
            namespace='tf_drivers',
            executable='stepper_driver_node',
            name='cmeresearch_stepper_driver',
            remappings=[
            ('drive_input', '/cmexa_base/front_right/cmd_vel'),
            ('drive_output', '/cmexa_base/front_right/feedback')],
            parameters=[{'bricklet_host': '192.168.1.102',
                         'bricklet_port': 4223,
                         'brick_position': 'a',
                         'step_resolution': 12,
                         'interpolation': True,
                         'acceleration': 5000,
                         'deceleration': 5000,
                         'steps_per_revolution': 200,
                         'max_step_vel': 5000,
                         'wheel_name': 'front_right',
                         'hw_simulation': False,
                         'standstill_current': 200,
                         'motor_run_current': 800,
                         'standstill_delay_time': 300,
                         'power_down_time': 1000,
                         'stealth_threshold': 450,
                         'coolstep_threshold': 500,
                         'classic_threshold': 600,
                         'high_velocity_chopper_mode': False
            }]
        ),
    ])
