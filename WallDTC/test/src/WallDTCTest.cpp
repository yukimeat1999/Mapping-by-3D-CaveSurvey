// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  WallDTCTest.cpp
 * @brief ModuleDescription (test code)
 *
 */
// </rtc-template>

#include "WallDTCTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* const walldtc_spec[] =
  {
    "implementation_id", "WallDTCTest",
    "type_name",         "WallDTCTest",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.MaxEdgeLength", "5",
    "conf.default.MaxConnDis", "10",
    "conf.default.Alpha", "0.5",

    // Widget
    "conf.__widget__.MaxEdgeLength", "text",
    "conf.__widget__.MaxConnDis", "text",
    "conf.__widget__.Alpha", "text",
    // Constraints

    "conf.__type__.MaxEdgeLength", "float",
    "conf.__type__.MaxConnDis", "float",
    "conf.__type__.Alpha", "float",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
WallDTCTest::WallDTCTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PCDOut("PCD", m_PCD),
    m_PlanWallIn("PlanWall", m_PlanWall)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
WallDTCTest::~WallDTCTest()
{
}



RTC::ReturnCode_t WallDTCTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("PlanWall", m_PlanWallIn);
  
  // Set OutPort buffer
  addOutPort("PCD", m_PCDOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("MaxEdgeLength", m_MaxEdgeLength, "5");
  bindParameter("MaxConnDis", m_MaxConnDis, "10");
  bindParameter("Alpha", m_Alpha, "0.5");
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t WallDTCTest::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t WallDTCTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t WallDTCTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t WallDTCTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t WallDTCTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t WallDTCTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t WallDTCTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t WallDTCTest::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t WallDTCTest::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t WallDTCTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t WallDTCTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool WallDTCTest::runTest()
{
    return true;
}


extern "C"
{
 
  void WallDTCTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(walldtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<WallDTCTest>,
                             RTC::Delete<WallDTCTest>);
  }
  
}
