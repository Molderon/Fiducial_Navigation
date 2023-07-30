#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <functional>
#include <memory>
#include <chrono>
#include <algorithm>

#include <unordered_map>
#include <iterator>
#include <utility>

#include <thread>
#include <mutex>
#include <condition_variable>

// OpenCV-4
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/core/core.hpp>

//Aruco
#include"include/aruco_ros/aruco/include/aruco/aruco.h"

// ROS2 Environment
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// namespace std - is temporary
using namespace std;
using namespace std::chrono_literals;
using namespace chrono;
using namespace cv;


#define AR_navigation true
#define Be_annoying true
#define magic_values true // strive to improve accuracy on a 1080p cam


struct Local_utils{
  //default metrics for distance estimation
  uint16_t Readings_range = 20;
  float Distance_Coef = 5.78f;
  float Markers_Size = 3.0f;

  string fCamera_calib, f_content;
  ifstream File;
  

  void config_me()
  {
    File.open("../configs/basic_configs.txt", ios::in);

    if(!File.is_open() || File.peek() == ifstream::traits_type::eof()){
      cout<<"[Fiducial Navigation]::Missing Config.txt File\n"
      <<"[Fiducial Navigation]::warning - Usage of Default Parameters\n\n";
    }
    else{
        File >> f_content;
        istringstream in_line(f_content);
        
        in_line >> this->Readings_range;
        in_line >> this->Distance_Coef;
        in_line >> this->Markers_Size;
        // other data if needed, append config.txt
    }
  }
};


Local_utils lo_util;




class Camera : public rclcpp::Node{
    public:
        size_t count_ = 0; string raw_data;

        float marker_length = lo_util.Markers_Size;
        bool Dispatch_Ready = false;
        Camera* self_ref;

        cv::VideoCapture Cam_Input;
        cv::Matx33d cam_matrix;
        cv::Mat dist_coeff;
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::DetectorParameters Detector = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        condition_variable m_cv;
        mutex m1;

        unordered_map<int,pair<uint16_t,Vec3d>> Found_Markers;
        string f_parameters = "../configs/Camera_Parameters.yaml";
        cv::Mat Object_Points;


        Camera(): Node("minimal_publisher"), count_(0)
        {    
            init_obj(Object_Points);
            Set_cam_Param(cam_matrix, dist_coeff);
            
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer_ = this->create_wall_timer(200ms, std::bind(&Camera::send_data, this));
        }


        void init_obj(Mat& Object_Points){

            cv::Mat base(4,1,CV_32FC3);
            this->Object_Points = base;

            this->Object_Points.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_length/2.f, marker_length/2.f, 0);
            this->Object_Points.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_length/2.f, marker_length/2.f, 0);
            this->Object_Points.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_length/2.f, -marker_length/2.f, 0);
            this->Object_Points.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_length/2.f, -marker_length/2.f, 0);
        }



        void Set_cam_Param(Matx33d& cam_matrix, Mat& dist_coeff){

            if (Is_File_Gone(this->f_parameters.c_str())){
                cv::FileStorage file(this->f_parameters.c_str(), FileStorage::READ);
                file["camera_matrix"] >> cam_matrix;
                file["distortion_coefficient"] >> dist_coeff;
                cout<<"[system]::Camera Calibrated!\n";

                file.release();
             }
             else{
                cerr<<"[system]::Calibration File - '"<<this->f_parameters<<"', was not found.\n";
                cerr<<"[system]::Default parameters loaded, beware!\n\n";

                cam_matrix = {632.29863082751251, 0, 319.5, 0, 632.29863082751251, 239.5, 0, 0, 1};
                dist_coeff = (Mat_<double>(5,1) << 0.070528331223347215, 0.26247385180956367, 0, 0, -1.0640942232949715);
             }
        }



        bool Is_File_Gone(const char* file){
             ifstream test(file);
             return test.good();
        }


        void raw_relative_data(cv::Vec3d& coordinates, int& id)
        {
          unordered_map<int,pair<uint16_t,Vec3d>>::iterator itr = this->Found_Markers.begin();

          if(this->Found_Markers.find(id) == this->Found_Markers.end()){
            Found_Markers[id] = make_pair(1, coordinates);
          }
          else{
            Found_Markers.at(id).first += 1;


            Found_Markers.at(id).second[0] += coordinates[0];
            Found_Markers.at(id).second[1] += coordinates[1];
            Found_Markers.at(id).second[2] += coordinates[2];
          }
          this->count_++;
        }


        void diminish_error(unique_lock<mutex>& m_lock, cv::Vec3d& coordinates, int& id, Camera* cam_object)
        {
            const unordered_map<int,pair<uint16_t,Vec3d>>::iterator itr = this->Found_Markers.find(id);

              if((*itr).second.first >= lo_util.Readings_range){
                   
                    Found_Markers.at(id).second[0] /= Found_Markers.at(id).first;
                    Found_Markers.at(id).second[1] /= Found_Markers.at(id).first;
                    Found_Markers.at(id).second[2] /= Found_Markers.at(id).first;
                    
                    Format_Relative_Data(id,coordinates, cam_object);                
              }
        }


        void Format_Relative_Data(int& id, Vec3d& coordinates, Camera* cam_object){
            cam_object->raw_data.append(to_string(id)+" "+ to_string(coordinates[0]) +
            " " + to_string(coordinates[1]) + " "+ to_string(coordinates[2])+ " |\n");
        }


        const char* Raw_tVec_transform(uint16_t& i, vector<Vec3d>& t_Vecs){
                string Z_axis;
                 int distance = std::round((t_Vecs[i][2]) * lo_util.Distance_Coef);
                 cout<<"single reading = "<<distance<<"\n";
                 if(magic_values == true){
                     if(distance > 10.00f){
                        Z_axis = to_string(distance*0.93);
                     }

                     else if(distance >= 9.50f && distance <10.00f){
                         Z_axis = to_string(distance * 0.96);
                     }

                     else if(distance >= 8.50f && distance <9.50f){
                         Z_axis = to_string(distance * 0.972);
                     }

                     else if(distance >= 7.50f && distance < 8.50f){
                         Z_axis = to_string(distance * 0.958);
                     }

                     else if(distance >= 6.50 && distance < 7.50){
                         Z_axis = to_string(distance * 0.966);
                     }
                }

                return Z_axis.c_str();
        }



        ~Camera(){
            cout<<"[System]::cam released!\n";
            rclcpp::shutdown();
        }
  
  bool Get_Ready_flag(){return this->Dispatch_Ready;}
  
  friend void Computer_Vision(Camera* cam_object);
  friend void timer_callback(Camera* cam_object);
   
   
  void get_self_ref(Camera* cam_object){
    this->self_ref = cam_object;
  }

  private:


    void refresh_algorithm(unique_lock<mutex>& m_lock, Camera* cam_object){
            cam_object->Dispatch_Ready = true;
            m_lock.unlock();
            m_cv.notify_one();

            m_lock.lock();

            cam_object->Found_Markers.clear();
            cam_object->count_ = 0;
            cam_object->m_cv.wait(m_lock, [&](){return Dispatch_Ready == false;});
    }

    void send_data(){

      auto message = std_msgs::msg::String();
      message.data = this->raw_data.c_str();
      if(message.data.empty()){return;}
      
        if(Be_annoying){
           RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        }

      publisher_->publish(message);
    }
};  




void timer_callback(Camera* cam_object){
    unique_lock<mutex>ul1(cam_object->m1);
    
    while(AR_navigation == true){
    cam_object->m_cv.wait(ul1, [&](){return cam_object->Get_Ready_flag();});

    cam_object->send_data();
    cam_object->raw_data.clear();
    
    cam_object->Dispatch_Ready = false;
    ul1.unlock();
    cam_object->m_cv.notify_one();
    ul1.lock();
  }
}



void Computer_Vision(Camera* cam_object){  
        std::vector<int> ids;
        vector<cv::Vec3d>r_Vecs, t_Vecs;
        std::vector<std::vector<cv::Point2f>> corners;
        Mat image, image_cpy; int m_Markers = 0;
        cv::aruco::ArucoDetector Detective(cam_object->dict, cam_object->Detector);
        
        unique_lock<mutex> m_lock(cam_object->m1);
        cam_object->Cam_Input.open(0);


        while(cam_object->Cam_Input.grab()){
            auto start = high_resolution_clock::now();
            
            if(cam_object->Cam_Input.retrieve(image) == false){exit(1);}

            image.copyTo(image_cpy);
            Detective.detectMarkers(image, corners, ids);
            
            if(ids.size()>0){ cv::aruco::drawDetectedMarkers(image, corners, ids);}
            else{continue;}

            m_Markers = corners.size();
            t_Vecs.resize(m_Markers);
            r_Vecs.resize(m_Markers);
   
            // Solve the orientation
            for(uint16_t i = 0; i< m_Markers; i++){
                solvePnP(cam_object->Object_Points, corners.at(i), cam_object->cam_matrix, cam_object->dist_coeff, r_Vecs.at(i), t_Vecs.at(i));
            }
            
            // Gain relative Distance and Draw frames
            if(Be_annoying){
                for(uint16_t i = 0; i<ids.size(); ++i){
                    cv::drawFrameAxes(image_cpy, cam_object->cam_matrix, cam_object->dist_coeff, r_Vecs[i], t_Vecs[i], 3);
                    cv::putText(image_cpy, cam_object->Raw_tVec_transform(i, t_Vecs), cv::Point(40,40), cv::FONT_HERSHEY_DUPLEX, 2, Scalar(30,255,130),2,false);

                    cam_object->raw_relative_data(t_Vecs[i], ids[i]);    
                }
                cv::aruco::drawDetectedMarkers(image_cpy,corners,ids);
                cv::imshow("[Distance and Pose - estimation]",image_cpy);
            }
                  
                else{for(uint16_t i = 0; i<ids.size(); ++i) cam_object->raw_relative_data(t_Vecs[i], ids[i]);}

            
            if((cam_object->count_ * cam_object->Found_Markers.size()) >= 
                (lo_util.Readings_range * cam_object->Found_Markers.size()))  
            {
              for(uint16_t i = 0; i<ids.size(); ++i){cam_object->diminish_error(m_lock, t_Vecs[i], ids[i], cam_object);}

              if(!cam_object->raw_data.empty()) cam_object->refresh_algorithm(m_lock, cam_object);
            }

            auto stop =high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            cout<<"Time for Execution: "<<duration.count()<<endl;

              char key = static_cast<char>(cv::waitKey(5));
                if(key == 27){break;}
        }       
}



void run_environment(){
      rclcpp::spin(std::make_shared<Camera>());
}



void run_cam(Camera* cam_object){
    thread thr_camera(Computer_Vision, cam_object);
    thread thr_com(timer_callback, cam_object);
    thread thr_spin(run_environment);
    
    if(thr_camera.joinable() && thr_com.joinable() && thr_spin.joinable())
    {thr_camera.join(); thr_com.join(); thr_spin.join();}
}



int main(int argc, char* argv[])
{    
    rclcpp::init(argc, argv);
    lo_util.config_me();

    Camera* cam_object = new Camera();
    run_cam(cam_object);


    delete cam_object;

  return 0;
} 