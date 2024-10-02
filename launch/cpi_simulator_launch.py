import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='attack_simulator',
            executable='cpi_simulator',
            parameters=[{'offset_x':2.0},
                        {'offset_y':0.7},
                        {'offset_z':0.0},
                        {'offset_yaw':0.0},
                        {'scale_x':1.0},
                        {'scale_y':1.0},
                        {'scale_z':1.0},
                        {'csv_path': '/home/awsim/Downloads/human.csv'}],
        )
    ])
