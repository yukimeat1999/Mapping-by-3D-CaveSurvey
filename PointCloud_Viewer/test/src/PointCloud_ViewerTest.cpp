﻿// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  PointCloud_ViewerTest.cpp
 * @brief PointCloud Viewer (test code)
 *
 */
// </rtc-template>

#include "PointCloud_ViewerTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* const pointcloud_viewer_spec[] =
  {
    "implementation_id", "PointCloud_ViewerTest",
    "type_name",         "PointCloud_ViewerTest",
    "description",       "PointCloud Viewer",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Viewer",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.DataLoadOption", "SameTime",
    "conf.default.FILE_NAME", "Your_PointCloud_File.ply",

    // Widget
    "conf.__widget__.DataLoadOption", "radio",
    "conf.__widget__.FILE_NAME", "text",
    // Constraints
    "conf.__constraints__.DataLoadOption", "(One_at_a_Time,SameTime,File)",

    "conf.__type__.DataLoadOption", "string",
    "conf.__type__.FILE_NAME", "string",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
PointCloud_ViewerTest::PointCloud_ViewerTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_new_PointCloudOut("new_PointCloud", m_new_PointCloud),
    m_merge_PointCloudOut("merge_PointCloud", m_merge_PointCloud),
    m_analyses_ClusterOut("analyses_Cluster", m_analyses_Cluster),
    m_LocalizationOut("Localization", m_Localization)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PointCloud_ViewerTest::~PointCloud_ViewerTest()
{
}



RTC::ReturnCode_t PointCloud_ViewerTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("new_PointCloud", m_new_PointCloudOut);
  addOutPort("merge_PointCloud", m_merge_PointCloudOut);
  addOutPort("analyses_Cluster", m_analyses_ClusterOut);
  addOutPort("Localization", m_LocalizationOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("DataLoadOption", m_DataLoadOption, "SameTime");
  bindParameter("FILE_NAME", m_FILE_NAME, "Your_PointCloud_File.ply");
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_ViewerTest::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_ViewerTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_ViewerTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t PointCloud_ViewerTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_ViewerTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_ViewerTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_ViewerTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_ViewerTest::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t PointCloud_ViewerTest::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_ViewerTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_ViewerTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool PointCloud_ViewerTest::runTest()
{
    return true;
}


extern "C"
{
 
  void PointCloud_ViewerTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(pointcloud_viewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<PointCloud_ViewerTest>,
                             RTC::Delete<PointCloud_ViewerTest>);
  }
  
}
