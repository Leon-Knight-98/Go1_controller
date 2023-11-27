#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"

#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#endif // COMPILE_WITH_ROS


bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

void wait() 
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

// void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
//   // Get the range, intensity, and time stamp of the laserscan message
//   double range0 = msg->ranges[0];
//   double range1 = msg->ranges[1];
//   double range2 = msg->ranges[2];
//   double range3 = msg->ranges[3];
//   double range4 = msg->ranges[4];

//   int length = msg->ranges.size();
//   double intensity = msg->ranges[1];
//   std::cout<<"Length"<<length<<std::endl;
//   std::cout<<"scan_angle"<<msg->angle_min<<" to "<<msg->angle_max<<std::endl;
//   std::cout<<"scan range from "<<msg->range_min<<" to "<<msg->range_max<<std::endl;
//   std::cout<<range0<<" "<<range1<<" "<<range2<<" "<<range3<<" "<< range4<<std::endl;
//   std::copy(msg->ranges.begin(), msg->ranges.end(), std::ostream_iterator<int>(std::cout, " "));
//   std::cout << std::endl;
//   ros::Time stamp = msg->header->stamp;

  // Do something with the laserscan data
  // ...
// }

int main(int argc, char **argv)
{
    std::cout<<"Hello"<<std::endl;
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

    // ros::init(argc, argv, "laser");
    // ros::NodeHandle nh;
    // ros::Subscriber reader;
    // reader = nh.subscribe("leftLaserScan", 1, laserscan_callback);
    
    // ros::spin();
    

#ifdef RUN_ROS
    ros::init(argc, argv, "unitree_gazebo_servo");
#endif // RUN_ROS

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;

#ifdef COMPILE_WITH_SIMULATION
    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_SIMULATION

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002; // run at 500hz
    ctrlComp->running = &running;

#ifdef ROBOT_TYPE_Go1
    ctrlComp->robotModel = new Go1Robot();
#endif

    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.75, Vec4(0.25, 0.5, 0.75, 0));  //Walk

    ctrlComp->geneObj();

    ControlFrame ctrlFrame(ctrlComp);

    signal(SIGINT, ShutDown);
    
    std::string current = "pass";
    std::string stand = "fix";
    std::string walk = "trot";
    std::string cmd_list[2] = {stand, walk};
    int flag0 = 0;
    int flag1 = 0;
    int flag2 = 0;
    int i = 0;
    int time = 0;
    // ctrlFrame.run();
    // ctrlFrame.change_state("pass");

    while (running){
        ctrlFrame.run();
        if(!flag0){
            ctrlFrame.change_state(stand);
            flag0 = 1;
            std::cout<<"done!"<<std::endl;
        }
        
        
        while(i<1000){
            ctrlFrame.run();
            i++;
        }

        if(!flag1){
            ctrlFrame.change_state(walk);
            flag1 = 1;
        }

        // while(i<3000){
        //     ctrlFrame.run();
        //     i++;
        // }

        // ctrlFrame.run();

        // if(!flag2){
        //     ctrlFrame.change_state(current);
        //     flag2 = 1;
        // }

        // if(!flag2){
        //     ctrlFrame.set_v(1.0, 0.0, 0.8);
        //     flag2 = 1;
        // }
        
    }   
    

    delete ctrlComp;
    return 0;
}