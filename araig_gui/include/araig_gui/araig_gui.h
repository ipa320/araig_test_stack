/*
  Copyright 2021 Changxuan Li
*/
#ifndef ARAIG_GUI_ARAIG_GUI_H
#define ARAIG_GUI_ARAIG_GUI_H

#include <ros/macros.h>

#include <rqt_gui_cpp/plugin.h>
#include <ui_araig_gui.h>
#include <QWidget>
#include <QStringList>
#include <QSet>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "araig_msgs/BoolStamped.h"
#include "nodelet/nodelet.h"
#include <boost/bind.hpp>


class AraigGui
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  AraigGui();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

protected:
  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> input_subs_;
  std::vector<ros::Publisher> output_pubs_;
private:
  bool interrupt_test_ = false;
  bool test_ready_ = false;
  bool result_recorded_ = false;
  bool completed_ = false;
  bool succ_ = false;
  bool failed_ = false;
  int num_inputs_;
  int num_outputs_;
  std::vector<std::string> name_inputs_;
  std::vector<std::string> name_outputs_;
  std::vector<bool> input_states_;
  std::vector<bool> output_states_;

public:
  void callbackBool(const araig_msgs::BoolStamped::ConstPtr& msg, const int &topicIdx);
  void outputTestState();
  void spawnPubs();
  void spawnSubs();
  void pubPublish(int idx);
  void stateInit();


private slots:
  void on_pbTestStart_clicked();

  void on_pbTestStop_clicked();

  void on_pbTestReset_clicked();

  void on_pbTestSucc_clicked();

  void on_pbTestFail_clicked();

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
};
#endif  // ARAIG_GUI_MY_PLUGIN_H
