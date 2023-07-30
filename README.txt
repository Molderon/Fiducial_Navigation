!Under Development


⠀⠀⠀⣤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣤⠀⠀⠀⠀⠀⠀⠀⠀⣠⣦⡀⠀⠀⠀     #Warning: Performance is not tested on a Raspberry Pi or other single board computers.
⠀⠀⠛⣿⠛⠀⠀⠀⠀⠀⠀⠀⠀⠀⠛⣿⠛⠀⠀⠀⠀⠀⡀⠺⣿⣿⠟⢀⡀⠀     #Warning: Currently Compatible with ROS Humble & ROS2 Foxy    
⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⣿⣦⠈⠁⣴⣿⣿⡦                           [INTRODUCTION]      
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣦⡈⠻⠟⢁⣴⣦⡈⠻⠋⠀             - This package is a nagigation system oriantated for
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⡀⠺⣿⣿⠟⢀⡀⠻⣿⡿⠋⠀⠀⠀             indoor robots equiped with a web-camera and provided
⠀⣠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣶⡿⠿⣿⣦⡈⠁⣴⣿⣿⡦⠈⠀⠀⠀⠀⠀             with a *(if possible) pre-defined Aruco environment.
⠲⣿⠷⠂⠀⠀⠀⠀⠀⠀⢀⣴⡿⠋⣠⣦⡈⠻⣿⣦⡈⠻⠋⠀⠀⠀⠀⠀⠀⠀
⠀⠈⠀⠀⠀⠀⠀⠀⠀⠰⣿⣿⡀⠺⣿⣿⣿⡦⠈⣻⣿⡦⠀⠀⠀⠀⠀⠀⠀⠀             - It strives to provide a very cost effective solution
⠀⠀⠀⠀⠀⠀⠀⠀⣠⣦⡈⠻⣿⣦⡈⠻⠋⣠⣾⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀             for PASSIVE navigation of mobile robotic platforms.
⠀⠀⠀⠀⠀⠀⡀⠺⣿⣿⠟⢀⡈⠻⣿⣶⣾⡿⠋⣠⣦⡀⠀⢀⣠⣤⣀⡀⠀⠀             It relies on:
⠀⠀⠀⠀⣠⣾⣿⣦⠈⠁⣴⣿⣿⡦⠈⠛⠋⠀⠀⠈⠛⢁⣴⣿⣿⡿⠋⠀⠀⠀             -> https://clover.coex.tech/en/aruco.html
⠀⠀⣠⣦⡈⠻⠟⢁⣴⣦⡈⠻⠋⠀⠀⠀⠀⠀⠀⠀⣴⣿⣿⣿⣏⠀⠀⠀⠀⠀             -> https://github.com/opencv/opencv
⠀⠺⣿⣿⠟⢀⡀⠻⣿⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠰⣿⡿⠛⠁⠙⣷⣶⣦⠀⠀
⠀⠀⠈⠁⣴⣿⣿⡦⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠋⠀⠀⠀⠀⠻⠿⠟⠀⠀  
⠀⠀⠀⠀⠈⠻⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  

                                                                      _____   
        [General]                                                    |     |  zzzzz
Currently the package requiers only a pre-installed                  | | | |`
native version of OpenCV4.5 or above to run.                         |_____|
And a standard ROS2 evoke to engage the two nodes              ____ ___|_|___ ____
                                                              ()___)         ()___)
    ->Aruco_Vision.cpp  &&  ->Aruco_Server.cpp                // /|           |\ \\
                                                             // / |           | \ \\
[Aruco_Vision] does the Computer Vision and outputs         (___) |___________| (___)
it's estimation to [Aruco_Server] where your local          (___)   (_______)   (___)
framework of nodes could gain relative and absolute         (___)     (___)     (___)
data about your machines location in contrast to the        (___)      |_|      (___)
local-area fiducial Markers.                                (___)  ___/___\___   | |
                                                             | |  |           |  | |
         [Usage]                                             | |  |___________| /___\
1. Build a solution with: "Colcon Build"                    /___\  |||     ||| //   \\
2. exec -> ros2 run Fiducial_navig Aruco_Server            //   \\ |||     ||| \\   //
3. exec -> ros2 run Fiducial_navig Aruco_Vision            \\   // |||     |||  \\ //
                                                            \\ // ()__)   (__()
                                                                  ///       \\\
        [Pre-Deployment]                                         ///         \\\
1. Make sure Camera and Basic coeficients are                  _///___     ___\\\_    
Deployed to their respective folders and calibrated           |_______|   |_______|

2. Produce a passive Data-Base of the robot's environments
in order to corespond and self-localize it's self  via the
index embedded into the (Aruco) fiducial marker. 