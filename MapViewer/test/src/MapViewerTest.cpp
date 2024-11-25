// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  MapViewerTest.cpp
 * @brief Map Visualization (test code)
 *
 */
// </rtc-template>

#include "MapViewerTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* const mapviewer_spec[] =
  {
    "implementation_id", "MapViewerTest",
    "type_name",         "MapViewerTest",
    "description",       "Map Visualization",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.DataLoadOption", "SameTime",
    "conf.default.SameTimeViewerClosed", "false",
    "conf.default.Switching", "EMap_PlanWall",

    // Widget
    "conf.__widget__.DataLoadOption", "radio",
    "conf.__widget__.SameTimeViewerClosed", "radio",
    "conf.__widget__.Switching", "radio",
    // Constraints
    "conf.__constraints__.DataLoadOption", "(One_at_a_Time,SameTime)",
    "conf.__constraints__.SameTimeViewerClosed", "(true, false)",
    "conf.__constraints__.Switching", "(EMap, EMap_PlanWall, PlanWall)",

    "conf.__type__.DataLoadOption", "string",
    "conf.__type__.SameTimeViewerClosed", "string",
    "conf.__type__.Switching", "string",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
MapViewerTest::MapViewerTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PlanWallOut("PlanWall", m_PlanWall),
    m_ContourOut("Contour", m_Contour)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
MapViewerTest::~MapViewerTest()
{
}



RTC::ReturnCode_t MapViewerTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("PlanWall", m_PlanWallOut);
  addOutPort("Contour", m_ContourOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("DataLoadOption", m_DataLoadOption, "SameTime");
  bindParameter("SameTimeViewerClosed", m_SameTimeViewerClosed, "false");
  bindParameter("Switching", m_Switching, "EMap_PlanWall");
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MapViewerTest::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t MapViewerTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t MapViewerTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t MapViewerTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MapViewerTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MapViewerTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t MapViewerTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t MapViewerTest::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MapViewerTest::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t MapViewerTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t MapViewerTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool MapViewerTest::runTest()
{
    return true;
}


extern "C"
{
 
  void MapViewerTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(mapviewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<MapViewerTest>,
                             RTC::Delete<MapViewerTest>);
  }
  
}
