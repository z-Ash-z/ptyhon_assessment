# Programming Challenge : ROS2

## Problem statement
Write a publisher and subscriber node in ROS2 Humble. The requirements are as follows:
1. The publisher should read this CSV file and publish each row as a separate message to a topic named “data” at a rate of 3 Hz. You only need to publish the Longitude, Latitude, Altitude, Time and Actual_Speed columns for the purpose of this assignment. You will need to create a custom message which encapsulates all these columns but the exact implementation of the message is left up to you. 

2.	The subscriber should then read each message and compute the difference between subsequent data points in the “Time” column and publish that as a float to the “diff” topic. While you may not use the “Time_diff” column directly, you can use it to test out your code. For example for three consecutive messages as below,
    ```
    MSG 1: -105.434187,41.233283,2606.7,2014-07-26 13:03:55+00:00
    MSG 2: -105.434177,41.233209,2607.6,2014-07-26 13:03:58+00:00
    MSG 3: -105.434183,41.233247,2607.2,2014-07-26 13:04:15+00:00
    ```
    The subscriber node should be able to compute the difference and publish the numbers 3.0(time difference between 1 & 2) and 17.0(time difference between 2 & 3) to “diff”.
3.	Please make sure you also use the ROS2 testing framework to write test cases for your publisher and subscriber nodes to see whether they perform as expected. We do not require an integration test to be written for the whole system, just 2 unit tests for the publisher and subscriber nodes respectively. 

4.	Please add a README file documenting the requirements for this project as well as instructions on how to run it.

## Solution

### Dependencies

- Ubuntu 22.04 LTS (I used the WSL version)
- ROS2 Humble

### Running the package

- Creating a workspace along with the source folder.
    ```
    mkdir -p sample_ws/src
    ```
- Open the `src` folder.
    ```
    cd sample_ws/src
    ```
- Clone the git from the main branch.
    ```
    git clone https://github.com/z-Ash-z/ptyhon_assessment.git
    ```
- Change directory to the workspace folder and source `ROS Humble`.
    ```
    cd ..
    source /opt/ros/humble/setup.bash
    ```
- Build the package.
    ```
    colcon build
    ```
- Source the package.
    ```
    source install/setup.bash
    ```

#### Running the publisher node
- Running the publisher. By default it runs using the [Test CSV file](/assessment_ros/gps_data_manipulation/data/GPS_Dataset_test.csv).
    ```
    ros2 run gps_data_manipulation data_read
    ```
    > To change the CSV file use the launch file.

#### Running the subscriber node
- In another terminal source ROS HUMBLE and the package, then run the subcriber node.
    ```
    ros2 run gps_data_manipulation data_process
    ```
    > The subscriber publishes the difference in time between two consecutive messages.
- While the subscriber is running, run the command `rqt_graph` to see the launched nodes and topics.
- Close this terminal we don't need it anymore.

## Using the launch file
- The launch file will launch both the `reader` and the `processor` node simultaneously.
    ```
    ros2 launch gps_data_manipulation pub_sub.launch.py
    ```
    > Use the parameter `file_name:=GPS_Dataset.csv` to choose amoungst the CSV files that are in the [data folder](/assessment_ros/gps_data_manipulation/data/). Add other files to this folder before building.


## Running tests
- From the workspace folder run the test command.
    ```
    colcon test --pytest-with-coverage
    ```
    > The `--pytest-with-coverage` flag will give us the coverage report that can be use in the GitHubCI. Find the `index.html` in the build folder to see the detailed report.

- You can see the results in the console while the test is running and also by using result command.
    ```
    colcon test-result
    ```