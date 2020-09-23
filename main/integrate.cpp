#include <iostream>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>
#include <fstream>
#include "../src/imu.h"
#include "../src/utilities.h"

using  std::cout;
using  std::endl;

void LoadIMU(std::string filename, std::vector<MotionData>& pose)
{

    std::ifstream f;
    f.open(filename.c_str());

    if(!f.is_open())
    {
        std::cerr << " can't open LoadFeatures file "<<std::endl;
        return;
    }

    while (!f.eof()) {

        std::string s;
        std::getline(f,s);

        if(! s.empty())
        {
            std::stringstream ss;
            ss << s;

            MotionData data;
            double time;
            Eigen::Vector3d gyro;
            Eigen::Vector3d acc;

            ss >> time;
            ss >> acc(0);
            ss >> acc(1);
            ss >> acc(2);
            ss >> gyro(0);
            ss >> gyro(1);
            ss >> gyro(2);

            data.timestamp = time;
            data.imu_gyro = gyro;
            data.imu_acc = acc;
            pose.push_back(data);

        }
    }

}

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        cout << "usage: ./integrate imu.txt" << endl;
        return 1;
    }

    std::vector<MotionData>imudata;
    LoadIMU(argv[1], imudata);

    std::ofstream save_points;
    save_points.open("out.txt");

    double dt = 1.0 / 210;
    Eigen::Vector3d Pwb{0, 0, 0};              // position :    from  imu measurements
    Eigen::Quaterniond Qwb{1, 0, 0, 0};            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw{0, 0, 0};          // velocity  :   from imu measurements
    // Eigen::Vector3d Pwb{20, 5, 5};              // position :    from  imu measurements
    // Eigen::Quaterniond Qwb{0.99875, 0.0499792, 0, 0};  // quaterniond:  from imu measurements
    // Eigen::Vector3d Vw{-0, 6.28319, 3.14159};          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0, 0, -9.81);    // ENU frame
    
    // 中值积分 
    // for (int i = 0; i < imudata.size() - 1; ++i) 
    // { 
    //     // MotionData imupose = imudata[i]; 
    //     MotionData imupose = imudata[i]; 
    //     MotionData imupose_next = imudata[i + 1]; 

    //     //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z] 
    //     Eigen::Quaterniond dq; 
    //     Eigen::Vector3d dtheta_half = (imupose.imu_gyro  
    //         + imupose_next.imu_gyro) / 2.0 * dt / 2.0; 
    //     dq.w() = 1; 
    //     dq.x() = dtheta_half.x(); 
    //     dq.y() = dtheta_half.y(); 
    //     dq.z() = dtheta_half.z();   

    //     Eigen::Quaterniond Qwb_next = Qwb * dq; 
    //     Eigen::Vector3d acc_w = 1/2.0 * (Qwb * (imupose.imu_acc)  
    //     + Qwb_next * (imupose_next.imu_acc)) + gw; 
    //     Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w; 
    //     Vw = Vw + acc_w * dt; 
    //     Qwb = Qwb_next; 
    
    // 欧拉积分
    for (int i = 1; i < imudata.size(); ++i) {

        MotionData imupose = imudata[i];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half =  imupose.imu_gyro * dt /2.0;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();

        /// imu 动力学模型 欧拉积分
        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
        Qwb = Qwb * dq;
        Vw = Vw + acc_w * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;

        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points<<imupose.timestamp<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;

    }

    std::cout<<"test　end"<<std::endl;
}