// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  ContourTest.cpp
 * @brief Elevation Map Generation (test code)
 *
 */
// </rtc-template>

#include "ContourTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* const contour_spec[] =
  {
    "implementation_id", "ContourTest",
    "type_name",         "ContourTest",
    "description",       "Elevation Map Generation",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.GridSize", "0.5",

    // Widget
    "conf.__widget__.GridSize", "text",
    // Constraints

    "conf.__type__.GridSize", "float",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ContourTest::ContourTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PCDOut("PCD", m_PCD),
    m_ContourIn("Contour", m_Contour)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
ContourTest::~ContourTest()
{
}



RTC::ReturnCode_t ContourTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("Contour", m_ContourIn);
  
  // Set OutPort buffer
  addOutPort("PCD", m_PCDOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("GridSize", m_GridSize, "0.5");
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ContourTest::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t ContourTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t ContourTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t ContourTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ContourTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ContourTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t ContourTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t ContourTest::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ContourTest::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t ContourTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t ContourTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool ContourTest::runTest()
{
    return true;
}


extern "C"
{
 
  void ContourTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(contour_spec);
    manager->registerFactory(profile,
                             RTC::Create<ContourTest>,
                             RTC::Delete<ContourTest>);
  }
  
}
