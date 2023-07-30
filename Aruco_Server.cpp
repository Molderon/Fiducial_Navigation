#include <iostream>
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <iterator>
#include <utility>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <array>
#include <list>

//unix environment
#include <signal.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using std::placeholders::_1;
#define BE_ANNOYING true

/*
      #TO-DEVELOP

    1.) sig_term does not behave as expected :/
    2.) Remove the std namespace;
*/

struct Thread_Control{
  condition_variable thread_cv;
  array<thread*, 2> th_holder;
  bool force_quit = false;
  bool threads_joined = false;
  mutex thr_mutex;
};


struct Common_Util{
  bool Node_Operational = true;
  bool data_ready = false;

  condition_variable CV_data;
  mutex mutex_data;

  string readings;
};

Common_Util gen_utils;
Thread_Control kill_command;


class Data_Base{
private:
    uint16_t marker_id; bool DB_created;

    struct Marker{
      string lab_sector;
      uint16_t My_ID;
      cv::Vec3d Orientation;

      Marker(cv::Vec3d& Orientation, string&area){
        this->lab_sector = area;
        this->Orientation[0] = Orientation[0];
        this->Orientation[1] = Orientation[1];
        this->Orientation[2] = Orientation[2];
        // and other relevant data...
      }
    };
    
    unordered_map<uint16_t, Marker*> Markers_Collection;


  public:

      Data_Base(){
        if(Load_DataBase(Markers_Collection)){DB_created = true;}
        else{DB_created = false;}
      }



      ~Data_Base(){
        if(DB_created == true){
          unordered_map<uint16_t,Marker*>::iterator itr = this->Markers_Collection.begin();
          for(itr ; itr != this->Markers_Collection.end(); ++itr){
              delete (*itr).second;
          }
        }
      }


    void Display_MarkerBase(){
       unordered_map<uint16_t, Marker*>::iterator itr = Markers_Collection.begin();

        for(itr; itr != Markers_Collection.end(); itr++){

          cout<<"Marker ID: "<<(*itr).first<<" - at:"<<(*itr).second->lab_sector<<endl;
          cout<<"Marker Orientation: X - "<<(*itr).second->Orientation[0]<<"\n";
          cout<<"Marker Orientation: Y - "<<(*itr).second->Orientation[1]<<"\n";
          cout<<"Marker Orientation: Z - "<<(*itr).second->Orientation[2]<<"\n";
      }
    }


  protected:
 
     bool Load_DataBase(unordered_map<uint16_t, Marker*>& Markers_Collection){
        string current_line, area, file_path = "../../MarkerBase/Test_Data.txt";
        ifstream File_stream; 

        cv::Vec3d current_orient; 
        float axis = 0.0f; uint16_t id = 0; 


        File_stream.open(file_path.c_str(), ios::in);
        if(!File_stream.is_open() || File_stream.peek() == ifstream::traits_type::eof())
        {
          cerr<<"[Aruco Server]::File is Damaged\n";
          this->DB_created = false;

          return false;
        }

        while(getline(File_stream, current_line))
        {
          istringstream in_line(current_line);
          //string coords;
          in_line >> id;
          in_line >> area;
          in_line >> current_orient[0] >> current_orient[1] >> current_orient[2];
          this->Markers_Collection[id] = new Marker(current_orient, area);
        }
        
        File_stream.close();
        return true;
     }
};



void sig_handler(int num){
    unique_lock<mutex>sig_lock(kill_command.thr_mutex);

    gen_utils.Node_Operational = false;
    sig_lock.unlock();
    kill_command.thread_cv.notify_one();
    sig_lock.lock();

    kill_command.thread_cv.wait(sig_lock, [&](){return kill_command.threads_joined == true;});
}


void thread_monitor()
{
    unique_lock<mutex> th_lock(kill_command.thr_mutex);
    kill_command.thread_cv.wait(th_lock, [&](){return kill_command.force_quit;});

    if(gen_utils.Node_Operational == false)
    {
          (*kill_command.th_holder.at(0)).~thread();
          (*kill_command.th_holder.at(1)).~thread();
    }

    kill_command.threads_joined = true;
    th_lock.unlock();
    kill_command.thread_cv.notify_one();
}



class CameraSubscriber : public rclcpp::Node
{
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


public:

  unordered_map<uint16_t, cv::Vec3d> relative_data_mapper;

  CameraSubscriber(): Node("camera_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&CameraSubscriber::topic_callback, this, _1));
  }

  ~CameraSubscriber(){
    rclcpp::shutdown();
  }



private:
  void topic_callback(const std_msgs::msg::String::SharedPtr message)
  {
    unique_lock<std::mutex> reciver_lock(gen_utils.mutex_data);
      if(BE_ANNOYING == true){
          RCLCPP_INFO(this->get_logger(), "I heard: '%s'", message->data.c_str());
      }

      gen_utils.readings = message->data;
      gen_utils.data_ready = true;

      reciver_lock.unlock();
      gen_utils.CV_data.notify_one();

    reciver_lock.lock();
    gen_utils.CV_data.wait(reciver_lock, [&](){return gen_utils.data_ready == false;});
  }
};



void Format_Transform(CameraSubscriber* sub_obj,
            vector<pair<uint16_t, cv::Vec3d>>& current_data)
{
    // Supper Bad algorithm - to be optimized
    stringstream ss(gen_utils.readings);
    vector<int>IDs; list<float>Coordinates;
    float value; cv::Vec3d current_marker;
    int id = 0;

    if(!gen_utils.readings.empty()){  
      while(ss>>id) {IDs.push_back(id); break;}
      while(ss>>value) {Coordinates.push_back(value);}
    }

    if(Coordinates.size()/3 != IDs.size() || IDs.size() == 0){
       cout<<"[System]::Corrupted Metrics\n";
       gen_utils.Node_Operational = false;
       exit(0); 
    }

    for(uint16_t i = 0; i<IDs.size(); ++i){
        for(uint16_t j = 0; j<3; j++){
            current_marker[j] = Coordinates.front();
            Coordinates.pop_front();
        }
        current_data.push_back(make_pair(IDs[i], current_marker));
    }
    gen_utils.readings.clear();
}



void Display_Findings(CameraSubscriber* sub_obj){
      unordered_map<uint16_t, cv::Vec3d>::iterator itr = sub_obj->relative_data_mapper.begin();
      for(itr; itr != sub_obj->relative_data_mapper.end(); itr++){
        cout<<"Marker ID:"<<(*itr).first<<", at -> X:"<<(*itr).second[0]<<" | Y:"<<(*itr).second[1]<<" | Z:"<<(*itr).second[2]<<"\n\n";
      }
}



void Assemble_Readings(CameraSubscriber* sub_obj)
{
     while(gen_utils.Node_Operational == true){
     unique_lock<mutex> Assembly_lock(gen_utils.mutex_data);
     
     vector<pair<uint16_t, cv::Vec3d>> current_data;
     unordered_map<uint16_t, cv::Vec3d>::iterator itr; 
     pair<uint16_t, cv::Vec3d> index; bool relevant_element = true;
     
     gen_utils.CV_data.wait(Assembly_lock, [&](){return gen_utils.data_ready;});
        
      Format_Transform(sub_obj, current_data);

      if(current_data.size() > 0){
          //Updating Relative Data
          for(uint16_t i = 0; i < current_data.size(); ++i){
            itr = sub_obj->relative_data_mapper.find(current_data[i].first);
            
            if(itr != sub_obj->relative_data_mapper.end())
            {
                sub_obj->relative_data_mapper.erase(itr);
                sub_obj->relative_data_mapper[current_data[i].first] = current_data[i].second;
            }
            else{sub_obj->relative_data_mapper[current_data[i].first] = current_data[i].second;}
          }


          //Removing old irelevant Readings
          for(itr = sub_obj->relative_data_mapper.begin(); itr != sub_obj->relative_data_mapper.end(); itr++){
            for(uint16_t i = 0; i<current_data.size(); ++i){
              if((*itr).first != current_data[i].first){
                relevant_element = false;
              }
              else{relevant_element = true; break;}
            }
            if(relevant_element == false){ sub_obj->relative_data_mapper.erase(itr);}
          }
      }

        else{
          sig_handler(1);
          cerr<<"Oops...\n";  exit(0);
        }
      

      gen_utils.data_ready = false;
      Assembly_lock.unlock();
      gen_utils.CV_data.notify_one();

      if(BE_ANNOYING == true){
        Display_Findings(sub_obj);  
      }

      Assembly_lock.lock();
    }
}



void run_environment(){
    rclcpp::spin(std::make_shared<CameraSubscriber>());
}



void run_node(CameraSubscriber* sub_obj){

   thread thr_env(run_environment);
   thread thr_data(Assemble_Readings, sub_obj);


   thread* ref_env = (&thr_env);
   kill_command.th_holder[0] = ref_env;
   thread* ref_data = (&thr_data);
   kill_command.th_holder[1] = ref_data;


   thread thr_monitor(thread_monitor);

   if(kill_command.threads_joined == false){
     if(thr_env.joinable() && thr_data.joinable() && thr_monitor.joinable())
     {
        thr_env.join(); thr_data.join(); thr_monitor.join();
     }
    }


   else{
      if(thr_monitor.joinable())
      {
         thr_monitor.join();
      }
    }
}



int main(int argc, char * argv[])
{
  signal(SIGINT, sig_handler);
  rclcpp::init(argc, argv);
 
  CameraSubscriber* sub_obj = new CameraSubscriber();
 
  run_node(sub_obj);

  delete sub_obj;
  return 0;
}