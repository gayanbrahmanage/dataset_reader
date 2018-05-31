This package outputs scanner data and odometry data from a raw log file to the topics 'scan' and 'odom' respectively.
Current works for intel.log which is the raw log data for Intel lab 2003. Needs ros kinetic, gmapping, and rviz installed 

Steps to run the program
1. Edit the reader.cpp to your scanner specs and format the way it reads your raw log data and places it in messages.
2. Compile the package with 'catkin_make --pkg dataset_reader'
3. Start roscore
4. Run the launch file with 'roslaunch dataset_reader dataset_reader.launch'
7. Add necessary settings to rviz

By Thomas Vy.