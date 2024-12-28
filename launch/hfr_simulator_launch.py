import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='attack_simulator',
            executable='hfr_simulator',
            parameters=[{'elimination_angle':0.0}
                        ,{'success_rate_chrono':1.0}
                        ,{'csv_path':'/home/awsimlab/Downloads/out.csv'}]
        )
    ])
