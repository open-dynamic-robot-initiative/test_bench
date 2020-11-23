/**
 * \file dg_teststand.cpp
 * \brief DGM wrapper around the teststand robot.
 * \author Julian Viereck
 * \date 2018
 */

#include <dynamic_graph_manager/ros_init.hh>
#include "dg_blmc_robots/dgm_test_bench.hpp"

namespace dg_blmc_robots
{

  DGMTeststand::DGMTeststand(): was_in_safety_mode_(false)
  {
  }

  DGMTeststand::~DGMTeststand()
  {
  }
  void DGMTeststand::initialization(){
      // initialize the communication with the can cards
      can_buses_ = std::make_shared<blmc_drivers::CanBus>("can0");
      can_motor_boards_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses_);
      motors_ = std::make_shared<blmc_drivers::Motor> (can_motor_boards_, 0);
      // wait until all board are ready and connected

      can_motor_boards_->wait_until_ready();
  }

  void DGMTeststand::initialize_hardware_communication_process()
  {
    try{
      std::vector<double> zero_to_index_angle = 
        params_["hardware_communication"]["calibration"]["zero_to_index_angle"].
          as<std::vector<double> >();
      assert(zero_to_index_angle.size() == zero_to_index_angle_from_file_.size());
      for(unsigned i=0; i<zero_to_index_angle_from_file_.size() ; ++i)
      {
        zero_to_index_angle_from_file_[i] = zero_to_index_angle[i];
      }
    }catch(...){
      throw std::runtime_error("Error in reading yaml param:"
                               "[hardware_communication][calibration]"
                               "[zero_to_index_angle]");
    }

    // get the hardware communication ros node handle
    ros::NodeHandle& ros_node_handle = dynamic_graph::ros_init(
      dynamic_graph::DynamicGraphManager::hw_com_ros_node_name_);

    /** initialize the user commands */
    ros_user_commands_.push_back(ros_node_handle.advertiseService(
        "calibrate", &DGMTeststand::calibrate_joint_position_callback, this));

    initialization();
  }

  bool DGMTeststand::is_in_safety_mode()
  {
    was_in_safety_mode_ |= teststand_.get_joint_velocities().cwiseAbs().maxCoeff() > 10000.;////////////////////////////////////
    if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode()) {
      was_in_safety_mode_ = true;
      return true;
    } else {
      return false;
    }
  }

  void DGMTeststand::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    try{
      teststand_.acquire_sensors();/////////////////////////////////////////////////////
      /**
        * Joint data
        */
      map.at("joint_positions") = teststand_.get_joint_positions();
      map.at("joint_velocities") = teststand_.get_joint_velocities();
      map.at("joint_torques") = teststand_.get_joint_torques();
      map.at("joint_target_torques") = teststand_.get_joint_target_torques();
      map.at("joint_encoder_index") = teststand_.get_joint_encoder_index();

      /**
        * Additional data
        */
      map.at("contact_sensors") = teststand_.get_contact_sensors_states();
      map.at("slider_positions") = teststand_.get_slider_positions();
      map.at("height_sensors") = teststand_.get_height_sensors();

      map.at("ati_force") = teststand_.get_ati_force();
      map.at("ati_torque") = teststand_.get_ati_torque();
    }catch(...){
      printf("Error in acquiring the sensors data\n");
      printf("Setting all of them 0.0\n");

      /**
        * Joint data
        */
      map.at("joint_positions").fill(0.0);
      map.at("joint_velocities").fill(0.0);
      map.at("joint_torques").fill(0.0);
      map.at("joint_target_torques").fill(0.0);
      map.at("joint_encoder_index").fill(0.0);

      /**
        * Additional data
        */
      map.at("contact_sensors").fill(0.0);
      map.at("slider_positions").fill(0.0);
      map.at("height_sensors").fill(0.0);

      map.at("ati_force").fill(0.0);
      map.at("ati_torque").fill(0.0);
    }
  }

  void DGMTeststand::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    try{
      ctrl_joint_torques_ = map.at("ctrl_joint_torques");
      teststand_.send_target_joint_torque(ctrl_joint_torques_);///////////////////////
    }catch(...){
      printf("Error sending controls\n");
    }
  }

  bool DGMTeststand::calibrate_joint_position_callback(
    dg_blmc_robots::TeststandCalibration::Request& req,
    dg_blmc_robots::TeststandCalibration::Response& res)
  {
    // parse and register the command for further call.
    add_user_command(std::bind(&DGMTeststand::calibrate_joint_position, 
                     this, req.mechanical_calibration, zero_to_index_angle_,
                     index_angle_));

    // return whatever the user want
    res.sanity_check = true;
    
    // the service has been executed properly
    return true;
  }

  void DGMTeststand::calibrate_joint_position(
    bool mechanical_calibration,
    std::array<double, 2>& zero_to_index_angle,
    std::array<double, 2>& index_angle)
  {
    index_angle.fill(0.0);
    if(mechanical_calibration)
    {
      zero_to_index_angle.fill(0.0);
    }else{
      zero_to_index_angle = zero_to_index_angle_from_file_;
    }

    teststand_.calibrate(zero_to_index_angle, index_angle, mechanical_calibration);////////////////////////////////////

    for(unsigned i=0 ; i<2 ; ++i)
    {
      rt_printf("zero_to_index_angle[%d] = %f\n", i, zero_to_index_angle[i]);
      rt_printf("index_angle[%d] = %f\n", i, index_angle[i]);
    }
  }

} // namespace dg_blmc_robots
