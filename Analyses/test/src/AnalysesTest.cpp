// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  AnalysesTest.cpp
 * @brief Analyses comp (test code)
 *
 */
// </rtc-template>

#include "AnalysesTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* const analyses_spec[] =
  {
    "implementation_id", "AnalysesTest",
    "type_name",         "AnalysesTest",
    "description",       "Analyses comp",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Analyses",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.epoch", "1000",
    "conf.default.MaxEpoch", "10000",

    // Widget
    "conf.__widget__.epoch", "text",
    "conf.__widget__.MaxEpoch", "text",
    // Constraints

    "conf.__type__.epoch", "int",
    "conf.__type__.MaxEpoch", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
AnalysesTest::AnalysesTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_merge_PointCloudOut("merge_PointCloud", m_merge_PointCloud),
    m_analyses_ClusterIn("analyses_Cluster", m_analyses_Cluster)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
AnalysesTest::~AnalysesTest()
{
}



RTC::ReturnCode_t AnalysesTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("analyses_Cluster", m_analyses_ClusterIn);
  
  // Set OutPort buffer
  addOutPort("merge_PointCloud", m_merge_PointCloudOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("epoch", m_epoch, "1000");
  bindParameter("MaxEpoch", m_MaxEpoch, "10000");
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AnalysesTest::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t AnalysesTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t AnalysesTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t AnalysesTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AnalysesTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AnalysesTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t AnalysesTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t AnalysesTest::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AnalysesTest::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t AnalysesTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t AnalysesTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool AnalysesTest::runTest()
{
    return true;
}


extern "C"
{
 
  void AnalysesTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(analyses_spec);
    manager->registerFactory(profile,
                             RTC::Create<AnalysesTest>,
                             RTC::Delete<AnalysesTest>);
  }
  
}
