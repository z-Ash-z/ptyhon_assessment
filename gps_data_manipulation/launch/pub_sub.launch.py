import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

package_name = 'gps_data_manipulation'

def generate_launch_description():

    # Setting the parameter for changing the file name.
    file_name = LaunchConfiguration('file_name')
    file_name_arg = DeclareLaunchArgument(
        'file_name',
        default_value = "GPS_Dataset.csv",
        description = "Choose the file name amoungst the list of files in data folder."
    )
    print(f'This is what I see: {file_name.describe()}')

    # Creating the publisher node.
    publisher = Node(
        package = package_name,
        executable = "data_read",
        name = "gps_data_reader",
        parameters = [{
            "file_path" : PathJoinSubstitution([
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
                "data",
                file_name
            ])
        }]
    )
    
    # Creating the subscriber node.
    processor = Node(
        package = package_name,
        executable = 'data_process',
        name = 'gps_data_processor',
    )

    return LaunchDescription([
        file_name_arg,
        publisher,
        processor
    ])