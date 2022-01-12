/*
  Copyright 2021 Changxuan Li
*/

#include "araig_gui/araig_gui.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <ros/ros.h>
#include <ros/master.h>
#include "nodelet/nodelet.h"
#include "std_msgs/Bool.h"
#include "araig_msgs/BoolStamped.h"
#include <QApplication>
#include <boost/algorithm/string.hpp>
#include <thread>
#include <chrono>
#include <stdlib.h>
#include <time.h>
#include <sstream>

AraigGui::AraigGui()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("AraigGui");
}

void AraigGui::initPlugin(qt_gui_cpp::PluginContext& context)
{
  QStringList argv = context.argv();
  widget_ = new QWidget();

  ui_.setupUi(widget_);
  context.addWidget(widget_);
  connect(ui_.pbTestStart, SIGNAL(clicked()), this, SLOT(on_pbTestStart_clicked()));
  connect(ui_.pbTestStop, SIGNAL(clicked()), this, SLOT(on_pbTestStop_clicked()));
  connect(ui_.pbTestReset, SIGNAL(clicked()), this, SLOT(on_pbTestReset_clicked()));
  connect(ui_.pbTestSucc, SIGNAL(clicked()), this, SLOT(on_pbTestSucc_clicked()));
  connect(ui_.pbTestFail, SIGNAL(clicked()), this, SLOT(on_pbTestFail_clicked()));

  nh_ = getNodeHandle();
  name_inputs_ = {"test_completed", "test_succeeded", "test_failed"};
  name_outputs_ = {"start_test", "interrupt_test", "reset_test", "check_succeed", "check_failed"};
  // gui node also subscribe to it self to get its text updated
  name_inputs_.insert(name_inputs_.end(), name_outputs_.begin(), name_outputs_.end());

  num_inputs_ = int(name_inputs_.size());
  num_outputs_ = int(name_outputs_.size());
  input_subs_.resize(static_cast<std::vector<ros::Publisher>::size_type>(num_inputs_));
  input_states_.resize(static_cast<std::vector<bool>::size_type>(num_inputs_));
  output_pubs_.resize(static_cast<std::vector<ros::Subscriber>::size_type>(num_outputs_));
  output_states_.resize(static_cast<std::vector<bool>::size_type>(num_outputs_));
  spawnPubs();
  spawnSubs();
  stateInit();
  outputTestState();
  ROS_INFO_STREAM("[GUI]: ARAIG_TEST_GUI initialized!");

}

void AraigGui::shutdownPlugin()
{
}

void AraigGui::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
}

void AraigGui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
}

void AraigGui::spawnPubs()
{
  for(int i=0; i<num_outputs_; i++)
  {
    std::string topicName = "/signal/ui/" + name_outputs_[static_cast<std::vector<std::string>::size_type>(i)];
    output_pubs_[static_cast<std::vector<ros::Publisher>::size_type>(i)]
        = nh_.advertise<araig_msgs::BoolStamped>(topicName, 10);
    ROS_INFO_STREAM("[GUI]: Spawned output topic publisher " << i << " for : " << topicName);
  }
}

void AraigGui::spawnSubs()
{
  int i;
  for(i=0; i<num_inputs_ - num_outputs_; i++)
  {
    std::string topicName = "/signal/runner/" + name_inputs_[i];
    input_subs_[i]
        = nh_.subscribe<araig_msgs::BoolStamped>(topicName, 10, boost::bind(&AraigGui::callbackBool, this, _1, i));
    ROS_INFO_STREAM("[GUI]: Spawned input topic subscriber " << i << " for : " << topicName);
  }
  for(; i<num_inputs_; i++)
  {
    std::string topicName = "/signal/ui/" + name_inputs_[i];
    input_subs_[i]
        = nh_.subscribe<araig_msgs::BoolStamped>(topicName, 10, boost::bind(&AraigGui::callbackBool, this, _1, i));
    ROS_INFO_STREAM("[GUI]: Spawned input topic subscriber " << i << " for : " << topicName);
  }
}

void AraigGui::callbackBool(const araig_msgs::BoolStamped::ConstPtr &msg, const int &topicIdx)
{
  input_states_[topicIdx] = msg->data? true:false;
  if(input_states_[5])
  {
    stateInit();
  }
  outputTestState();
}

void AraigGui::outputTestState()
{
  //input: 0: completed, 1: succeeded, 2:failed
  // 3:start, 4:stop, 5:reset, 6:check_succ, 7:check_failed
  completed_ = input_states_[0];
  succ_ = input_states_[1];
  failed_ = input_states_[2];
  test_ready_ = !input_states_[3];
  interrupt_test_ = input_states_[4];
  result_recorded_ = input_states_[6]||input_states_[7];

  if(test_ready_) // test ready
  {
    ui_.lbTestState->setText("Test ready!");
    ui_.lbTestResult->setText("Wait for start!");
  }
  else if(!completed_) // test running
  {
    ui_.lbTestState->setText("Test running!");
    ui_.lbTestResult->setText("Waiting for result...");
  }
  else
  {
    if(interrupt_test_) // test interrupted
    {
      ui_.lbTestState->setText("Test interrupted!");
      ui_.lbTestResult->setText("Test failed!");
    }
    else if(result_recorded_) // test recorded
    {
      ui_.lbTestState->setText("Please reset!");
      ui_.lbTestResult->setText("Result recorded!");
    }
    else if(failed_ == succ_) // error in result
    {
      ui_.lbTestState->setText("Test completed!");
      ui_.lbTestResult->setText("Error in result!");
    }
    else if(failed_) // test fails
    {
      ui_.lbTestState->setText("Test completed!");
      ui_.lbTestResult->setText("Test failed!");
    }
    else if(succ_) // test succeeds
    {
      ui_.lbTestState->setText("Test completed!");
      ui_.lbTestResult->setText("Test succeeded!");
    }
    else // otherwise
    {
      ui_.lbTestState->setText("Please reset!");
      ui_.lbTestResult->setText(" ");
    }
  }

//  if(!completed_ && !test_ready_ && !result_recorded_) // test is running
//  {
//    ui_.lbTestState->setText("Test running!");
//    ui_.lbTestResult->setText("Waiting for result...");
//  }
//  else if(completed_ && !result_recorded_ && !interrupt_test_) // test completed
//  {
//    if(failed_ == succ_)
//    {
//      ui_.lbTestState->setText("Test completed!");
//      ui_.lbTestResult->setText("Error in result!");
//    }
//    else if(failed_) // test fails
//    {
//      ui_.lbTestState->setText("Test completed!");
//      ui_.lbTestResult->setText("Test failed!");
//    }
//    else if(succ_) // test succeeds
//    {
//      ui_.lbTestState->setText("Test completed!");
//      ui_.lbTestResult->setText("Test succeeded!");
//    }
//  }
//  else if (test_ready_) // test is ready
//  {
//    ui_.lbTestState->setText("Test ready!");
//    ui_.lbTestResult->setText("Wait for start!");
//  }
//  else if (completed_ && result_recorded_ && !interrupt_test_) // test is recorded
//  {
//    ui_.lbTestState->setText("Please reset!");
//    ui_.lbTestResult->setText("Result recorded!");
//  }
//  else if (completed_ && interrupt_test_) // test interrupted
//  {
//    ui_.lbTestState->setText("Test interrupted!");
//    ui_.lbTestResult->setText("Test failed!");
//  }
//  else // otherwise
//  {
//    ui_.lbTestState->setText("Please reset!");
//    ui_.lbTestResult->setText(" ");
//  }
}

void AraigGui::pubPublish(int idx)
{
  araig_msgs::BoolStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.data = output_states_[idx]?true:false;
  output_pubs_[idx].publish(msg);
  ros::spinOnce();
}

void AraigGui::stateInit()
{
  for (auto it = output_states_.begin();it!=output_states_.end();it++) {
    *it = false;
  }
  for (auto it = input_states_.begin();it!=input_states_.end();it++) {
    *it = false;
  }
}

void AraigGui::on_pbTestStart_clicked()
{
  //output: 0:start, 1:stop, 2:reset, 3:succ, 4:failed
  //input: 0: completed, 1: succeeded, 2:failed
  output_states_[0] = true;
  output_states_[1] = false;
  output_states_[2] = false;
  pubPublish(0);
  ROS_INFO_STREAM("[GUI]: test started!");
  //test_ready_ = false;
  //outputTestState();
}

void AraigGui::on_pbTestStop_clicked()
{
  output_states_[0] = false;
  output_states_[1] = true;
  output_states_[2] = false;
  pubPublish(1);
  ROS_INFO_STREAM("[GUI]: test stopped!");
  //interrupt_test_ = true;
  //outputTestState();
}

void AraigGui::on_pbTestReset_clicked()
{
  output_states_[0] = false;
  output_states_[1] = false;
  output_states_[2] = true;
  pubPublish(2);
  ROS_INFO_STREAM("[GUI]: test reseted!");

  //stateInit();
  //outputTestState();
}

void AraigGui::on_pbTestSucc_clicked()
{
  output_states_[3] = true;
  output_states_[4] = false;
  pubPublish(3);
  //result_recorded_ = true;
  ROS_INFO_STREAM("[GUI]: Result recorded!");
  //outputTestState();
}

void AraigGui::on_pbTestFail_clicked()
{
  output_states_[3] = false;
  output_states_[4] = true;
  pubPublish(4);
  //result_recorded_ = true;
  ROS_INFO_STREAM("[GUI]: Result recorded!");
  //outputTestState();
}

PLUGINLIB_EXPORT_CLASS(AraigGui, rqt_gui_cpp::Plugin)

