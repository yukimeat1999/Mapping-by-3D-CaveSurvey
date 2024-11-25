// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  Registration.h
 * @brief Registration comp
 *
 */
 // </rtc-template>

#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "pointcloudStub.h"
#include "ExtendedDataTypesStub.h"

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// User include
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <vtkOutputWindow.h>

// <rtc-template block="component_description">
/*!
 * @class Registration
 * @brief Registration comp
 *
 */
 // </rtc-template>
class Registration
	: public RTC::DataFlowComponentBase
{
public:
	/*!
	 * @brief constructor
	 * @param manager Maneger Object
	 */
	Registration(RTC::Manager* manager);

	/*!
	 * @brief destructor
	 */
	~Registration() override;

	// <rtc-template block="public_attribute">

	// </rtc-template>

	// <rtc-template block="public_operation">

	// </rtc-template>

	// <rtc-template block="activity">
	/***
	 *
	 * The initialize action (on CREATED->ALIVE transition)
	 *
	 * @return RTC::ReturnCode_t
	 *
	 *
	 */
	RTC::ReturnCode_t onInitialize() override;

	/***
	 *
	 * The finalize action (on ALIVE->END transition)
	 *
	 * @return RTC::ReturnCode_t
	 *
	 *
	 */
	RTC::ReturnCode_t onFinalize() override;

	/***
	 *
	 * The startup action when ExecutionContext startup
	 *
	 * @param ec_id target ExecutionContext Id
	 *
	 * @return RTC::ReturnCode_t
	 *
	 *
	 */
	 // RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id) override;

	 /***
	  *
	  * The shutdown action when ExecutionContext stop
	  *
	  * @param ec_id target ExecutionContext Id
	  *
	  * @return RTC::ReturnCode_t
	  *
	  *
	  */
	  // RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id) override;

	  /***
	   *
	   * The activated action (Active state entry action)
	   *
	   * @param ec_id target ExecutionContext Id
	   *
	   * @return RTC::ReturnCode_t
	   *
	   *
	   */
	RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id) override;

	/***
	 *
	 * The deactivated action (Active state exit action)
	 *
	 * @param ec_id target ExecutionContext Id
	 *
	 * @return RTC::ReturnCode_t
	 *
	 *
	 */
	RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id) override;

	/***
	 *
	 * The execution action that is invoked periodically
	 *
	 * @param ec_id target ExecutionContext Id
	 *
	 * @return RTC::ReturnCode_t
	 *
	 *
	 */
	RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id) override;

	/***
	 *
	 * The aborting action when main logic error occurred.
	 *
	 * @param ec_id target ExecutionContext Id
	 *
	 * @return RTC::ReturnCode_t
	 *
	 *
	 */
	 // RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id) override;

	 /***
	  *
	  * The error action in ERROR state
	  *
	  * @param ec_id target ExecutionContext Id
	  *
	  * @return RTC::ReturnCode_t
	  *
	  *
	  */
	RTC::ReturnCode_t onError(RTC::UniqueId ec_id) override;

	/***
	 *
	 * The reset action that is invoked resetting
	 *
	 * @param ec_id target ExecutionContext Id
	 *
	 * @return RTC::ReturnCode_t
	 *
	 *
	 */
	RTC::ReturnCode_t onReset(RTC::UniqueId ec_id) override;

	/***
	 *
	 * The state update action that is invoked after onExecute() action
	 *
	 * @param ec_id target ExecutionContext Id
	 *
	 * @return RTC::ReturnCode_t
	 *
	 *
	 */
	 // RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id) override;

	 /***
	  *
	  * The action that is invoked when execution context's rate is changed
	  *
	  * @param ec_id target ExecutionContext Id
	  *
	  * @return RTC::ReturnCode_t
	  *
	  *
	  */
	  // RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id) override;
	  // </rtc-template>


protected:
	// <rtc-template block="protected_attribute">

	// </rtc-template>

	// <rtc-template block="protected_operation">

	// </rtc-template>

	// Configuration variable declaration
	// <rtc-template block="config_declare">

	// </rtc-template>

	// DataInPort declaration
	// <rtc-template block="inport_declare">
	PointCloudTypes::PointCloud m_new_PointCloud;
	/*!
	 */
	RTC::InPort<PointCloudTypes::PointCloud> m_new_PointCloudIn;

	// </rtc-template>


	// DataOutPort declaration
	// <rtc-template block="outport_declare">
	PointCloudTypes::PointCloud m_merge_PointCloud;
	/*!
	 */
	RTC::OutPort<PointCloudTypes::PointCloud> m_merge_PointCloudOut;
	RTC::TimedPose3D m_Localization;
	/*!
	 */
	RTC::OutPort<RTC::TimedPose3D> m_LocalizationOut;

	// </rtc-template>

	// CORBA Port declaration
	// <rtc-template block="corbaport_declare">

	// </rtc-template>

	// Service declaration
	// <rtc-template block="service_declare">

	// </rtc-template>

	// Consumer declaration
	// <rtc-template block="consumer_declare">

	// </rtc-template>


private:
	// <rtc-template block="private_attribute">

	// </rtc-template>

	// <rtc-template block="private_operation">

	// </rtc-template>

};


extern "C"
{
	DLL_EXPORT void RegistrationInit(RTC::Manager* manager);
};

#endif // REGISTRATION_H
