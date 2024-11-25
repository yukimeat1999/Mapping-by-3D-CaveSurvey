// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  FPSTest.cpp
 * @brief ModuleDescription (test code)
 *
 */
// </rtc-template>

#include "FPSTest.h"

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const fps_spec[] =
#else
static const char* fps_spec[] =
#endif
  {
    "implementation_id", "FPSTest",
    "type_name",         "FPSTest",
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
    "conf.default.FPS_Max", "10000",

    // Widget
    "conf.__widget__.FPS_Max", "text",
    // Constraints

    "conf.__type__.FPS_Max", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
FPSTest::FPSTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PCDOut("PCD", m_PCD),
    m_DownPCDIn("DownPCD", m_DownPCD)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
FPSTest::~FPSTest()
{
}



RTC::ReturnCode_t FPSTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("DownPCD", m_DownPCDIn);
  
  // Set OutPort buffer
  addOutPort("PCD", m_PCDOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("FPS_Max", m_FPS_Max, "10000");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t FPSTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t FPSTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t FPSTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t FPSTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t FPSTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t FPSTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t FPSTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t FPSTest::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t FPSTest::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t FPSTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t FPSTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool FPSTest::runTest()
{
    return true;
}


extern "C"
{
 
  void FPSTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(fps_spec);
    manager->registerFactory(profile,
                             RTC::Create<FPSTest>,
                             RTC::Delete<FPSTest>);
  }
  
}
