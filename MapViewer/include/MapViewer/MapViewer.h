// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  MapViewer.h
 * @brief Map Visualization
 *
 */
// </rtc-template>

#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "ClusterStub.h"
#include "pointcloudStub.h"

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "gng.h"
#include <pcl/visualization/cloud_viewer.h>
#include <vtkOutputWindow.h>

// 型定義
using PointT = pcl::PointXYZ;               // XYZ形式の点群を使用
using ColorPointT = pcl::PointXYZRGB;       // XYZRGB形式の点群を使用
using CloudT = pcl::PointCloud<PointT>;
using ColorCloudT = pcl::PointCloud<ColorPointT>;

// Function
void RedirectVTKOutputWindow();
void visualizeCloud(CloudT::Ptr cloud, const std::string& PointName);
void visualizeCloud(ColorCloudT::Ptr cloud, const std::string& PointName);
void GNGView(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,
	struct gng* net,
	const std::string& PointName);

// <rtc-template block="component_description">
/*!
 * @class MapViewer
 * @brief Map Visualization
 *
 */
// </rtc-template>
class MapViewer
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  MapViewer(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~MapViewer() override;

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
  /*!
   * データポートから読み込んだデータの表示方法を選択する．
   * One_at_a_Time: データポートのデータを一つずつ表示
   * SameTime: データポートのデータを画面に同時に配置
   * - Name: DataLoadOption DataLoadOption
   * - DefaultValue: SameTime
   * - Constraint: (One_at_a_Time,SameTime)
   */
  std::string m_DataLoadOption;
  /*!
   * To close the window, switch to "false".
   * - Name: SameTimeViewerClosed SameTimeViewerClosed
   * - DefaultValue: false
   * - Constraint: (true, false)
   */
  std::string m_SameTimeViewerClosed;
  /*!
   * Please switch the display contents.
   * EMap: Only the elevation map is displayed.

   * EMap_PlanWall: Display both.

   * PlanWall: Only the walls of the cave are displayed.
   * - Name: Switching Switching
   * - DefaultValue: EMap_PlanWall
   * - Constraint: (EMap, EMap_PlanWall, PlanWall)
   */
  std::string m_Switching;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  ClusterTypes::ClusterData m_PlanWall;
  /*!
   */
  RTC::InPort<ClusterTypes::ClusterData> m_PlanWallIn;
  PointCloudTypes::PointCloud m_Contour;
  /*!
   */
  RTC::InPort<PointCloudTypes::PointCloud> m_ContourIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
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
     void init_m_gng(ClusterTypes::ClusterData& m_PlanWall);
     void free_m_gng(ClusterTypes::ClusterData& m_PlanWall);
	 int Switch(const std::string& sw);
     bool STVClosed(const std::string& sw);
};


extern "C"
{
  DLL_EXPORT void MapViewerInit(RTC::Manager* manager);
};

#endif // MAPVIEWER_H
