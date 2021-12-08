/*
  Copyright 2016 Lucas Walter
*/

#include "araig_gui/my_plugin.h"
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

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file


  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
  connect(ui_.pbTestStart, SIGNAL(clicked()), this, SLOT(on_pbTestStart_clicked()));
  connect(ui_.pbTestStop, SIGNAL(clicked()), this, SLOT(on_pbTestStop_clicked()));
  connect(ui_.pbTestReset, SIGNAL(clicked()), this, SLOT(on_pbTestReset_clicked()));
  connect(ui_.pbTestSucc, SIGNAL(clicked()), this, SLOT(on_pbTestSucc_clicked()));
  connect(ui_.pbTestFail, SIGNAL(clicked()), this, SLOT(on_pbTestFail_clicked()));

  nh_ = getNodeHandle();
  name_inputs_ = {"start_test", "interrupt_test", "reset_test", "check_succeed", "check_failed"};
  name_outputs_ = {"test_completed", "test_succeeded", "test_failed"};
  num_inputs_ = int(name_inputs_.size());
  num_outputs_ = int(name_outputs_.size());
  input_pubs_.resize(static_cast<std::vector<ros::Publisher>::size_type>(num_inputs_));
  input_states_.resize(static_cast<std::vector<bool>::size_type>(num_inputs_));
  output_subs_.resize(static_cast<std::vector<ros::Subscriber>::size_type>(num_outputs_));
  output_states_.resize(static_cast<std::vector<bool>::size_type>(num_outputs_));
  spawnPubs();
  spawnSubs();
  stateInit();
  ROS_INFO_STREAM("[GUI]: ARAIG_TEST_GUI initialized!");

}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

void MyPlugin::spawnPubs()
{
  for(int i=0; i<num_inputs_; i++)
  {
    std::string topicName = "/signal/ui/" + name_inputs_[static_cast<std::vector<std::string>::size_type>(i)];
    input_pubs_[static_cast<std::vector<ros::Publisher>::size_type>(i)]
        = nh_.advertise<araig_msgs::BoolStamped>(topicName, 10);
    ROS_INFO_STREAM("[GUI]: Spawned input topic publisher " << i << " for : " << topicName);
  }
}

void MyPlugin::spawnSubs()
{
  for(int i=0; i<num_outputs_; i++)
  {
    std::string topicName = "/signal/runner/" + name_outputs_[static_cast<std::vector<std::string>::size_type>(i)];
    output_subs_[static_cast<std::vector<ros::Subscriber>::size_type>(i)]
        = nh_.subscribe<araig_msgs::BoolStamped>(topicName, 10, boost::bind(&MyPlugin::callbackBool, this, _1, topicName));
    ROS_INFO_STREAM("[GUI]: Spawned output topic subscriber " << i << " for : " << topicName);
  }
}

int MyPlugin::getIndexInVector(std::vector<std::string> vec, std::string topicName)
{
  auto it = find(vec.begin(), vec.end(), topicName);
  if(it != vec.end())
  {
    int idx = static_cast<int>(it - vec.begin());
    return idx;
  }
  else
  {
    return -1;
  }
}

void MyPlugin::callbackBool(const araig_msgs::BoolStamped::ConstPtr &msg, const std::string &topicName)
{
  std::vector<std::string> nameElems;
  boost::split(nameElems, topicName, [](char c){return c == '/';});
  ROS_INFO_STREAM("[GUI]: get data on topic "<<nameElems.back());

  int idx = getIndexInVector(name_outputs_, nameElems.back());
  if(idx !=-1)
  {
    output_states_[static_cast<std::vector<bool>::size_type>(idx)] = msg->data? true:false;
    outputTestState();
  }
}

void MyPlugin::outputTestState()
{
  //output: 0: completed, 1: succeeded, 2:failed
  bool completed = output_states_[0];
  bool succ = output_states_[1];
  bool failed = output_states_[2];

  ROS_INFO_STREAM("[GUI]: the test is completed? "<<completed);
  ROS_INFO_STREAM("[GUI]: the state of succ is "<<succ);
  ROS_INFO_STREAM("[GUI]: the state of failed is "<<failed);
  ROS_INFO_STREAM("[GUI]: the state of ready is "<<test_ready_);

  if(!completed && !test_ready_ && !result_recorded_) // test is running
  {
    ui_.lbTestState->setText("Test running!");
    ui_.lbTestResult->setText("Waiting for result...");
  }
  else if(completed && !result_recorded_) // test terminates
  {
    if(failed == succ)
    {
      ui_.lbTestState->setText("Test completed!");
      ui_.lbTestResult->setText("Error in result!");
    }
    else if(failed) // test fails
    {
      ui_.lbTestState->setText("Test completed!");
      ui_.lbTestResult->setText("Test failed!");
    }
    else if(succ) // test succeeds
    {
      ui_.lbTestState->setText("Test completed!");
      ui_.lbTestResult->setText("Test succeeded!");
    }
  }
  else if (test_ready_) // test is ready
  {
    ui_.lbTestState->setText("Test ready!");
    ui_.lbTestResult->setText("Wait for start!");
  }
  else if (completed && result_recorded_)
  {
    ui_.lbTestState->setText("Please reset!");
    ui_.lbTestResult->setText("Result recorded!");
  }
  else // otherwise
  {
    ui_.lbTestState->setText("Please reset!");
    ui_.lbTestResult->setText(" ");
  }
}

void MyPlugin::pubPublish(int idx)
{
  araig_msgs::BoolStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.data = input_states_[static_cast<std::vector<bool>::size_type>(idx)]?true:false;
  input_pubs_[static_cast<std::vector<ros::Publisher>::size_type>(idx)].publish(msg);
  ros::spinOnce();
}

void MyPlugin::stateInit()
{
  for (auto it = output_states_.begin();it!=output_states_.end();it++) {
    *it = false;
  }
  for (auto it = input_states_.begin();it!=input_states_.end();it++) {
    *it = false;
  }
  test_ready_ = true;
  result_recorded_ = false;
}

PLUGINLIB_EXPORT_CLASS(MyPlugin, rqt_gui_cpp::Plugin)

void MyPlugin::on_pbTestStart_clicked()
{
  //input: 0:start, 1:stop, 2:reset, 3:succ, 4:failed
  //output: 0: completed, 1: succeeded, 2:failed
  input_states_[0] = true;
  input_states_[1] = false;
  input_states_[2] = false;
  pubPublish(0);
  ROS_INFO_STREAM("[GUI]: test started!");
  test_ready_ = false;
  outputTestState();
}

void MyPlugin::on_pbTestStop_clicked()
{
  input_states_[0] = false;
  input_states_[1] = true;
  input_states_[2] = false;
  pubPublish(1);
  ROS_INFO_STREAM("[GUI]: test stopped!");
  output_states_[0] = true;
  srand(time(NULL));
  if(rand()%100 >= 50)
  {
    output_states_[1] = true;
  }
  else
  {
    output_states_[2] = true;
  }
  outputTestState();
}

void MyPlugin::on_pbTestReset_clicked()
{
  input_states_[0] = false;
  input_states_[1] = false;
  input_states_[2] = true;
  pubPublish(2);
  ROS_INFO_STREAM("[GUI]: test reseted!");

  stateInit();
  outputTestState();
}

void MyPlugin::on_pbTestSucc_clicked()
{
  input_states_[3] = true;
  input_states_[4] = false;
  pubPublish(3);
  result_recorded_ = true;
  ROS_INFO_STREAM("[GUI]: Result recorded!");
  outputTestState();
}

void MyPlugin::on_pbTestFail_clicked()
{
  input_states_[3] = false;
  input_states_[4] = true;
  pubPublish(4);
  result_recorded_ = true;
  ROS_INFO_STREAM("[GUI]: Result recorded!");
  outputTestState();
}
