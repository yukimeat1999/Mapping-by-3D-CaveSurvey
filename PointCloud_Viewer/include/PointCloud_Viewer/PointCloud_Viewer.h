﻿// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  PointCloud_Viewer.h
 * @brief PointCloud Viewer
 *
 */
// </rtc-template>

#ifndef POINTCLOUD_VIEWER_H
#define POINTCLOUD_VIEWER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "pointcloudStub.h"
#include "ClusterStub.h"
#include "ExtendedDataTypesStub.h"

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// User include
#include "gng.h"
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkOutputWindow.h>

void RedirectVTKOutputWindow();
void PointCloudView(const std::string& FileName);
void PointCloudView(const pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, const std::string& PortName);
void PointCloudView(const pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud,
                    const std::string& PortName,
                    const pcl::PointXYZ Point);
void PointCloudView(const pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud0,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud1,
                    struct gng* net,
                    const pcl::PointXYZ Point);
void GNGView(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,
             struct gng* net,
             const std::string& PointName);

// <rtc-template block="component_description">
/*!
 * @class PointCloud_Viewer
 * @brief PointCloud Viewer
 *
 */
// </rtc-template>
class PointCloud_Viewer
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  PointCloud_Viewer(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~PointCloud_Viewer() override;

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
  // RTC::ReturnCode_t onError(RTC::UniqueId ec_id) override;

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
   * データポートまたはファイルから読み込むかを選択する．
   * One_at_a_Time: データポートから点群を一つずつ表示
   * SameTime: データポートから点群を画面に同時に配置
   * File: ファイルから読み込む
   * - Name: DataLoadOption DataLoadOption
   * - DefaultValue: SameTime
   */
  std::string m_DataLoadOption;
  /*!
   * Only ".ply" format files are supported.
   * - Name: FILE_NAME FILE_NAME
   * - DefaultValue: Your_PointCloud_File.ply
   */
  std::string m_FILE_NAME;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  PointCloudTypes::PointCloud m_new_PointCloud;
  /*!
   */
  RTC::InPort<PointCloudTypes::PointCloud> m_new_PointCloudIn;
  PointCloudTypes::PointCloud m_merge_PointCloud;
  /*!
   */
  RTC::InPort<PointCloudTypes::PointCloud> m_merge_PointCloudIn;
  ClusterTypes::ClusterData m_analyses_Cluster;
  /*!
   */
  RTC::InPort<ClusterTypes::ClusterData> m_analyses_ClusterIn;
  RTC::TimedPose3D m_Localization;
  /*!
   */
  RTC::InPort<RTC::TimedPose3D> m_LocalizationIn;
  
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
     void init_m_gng(ClusterTypes::ClusterData& m_analyses_Cluster);
     void free_m_gng(ClusterTypes::ClusterData& m_analyses_Cluster);
};


extern "C"
{
  DLL_EXPORT void PointCloud_ViewerInit(RTC::Manager* manager);
};

#endif // POINTCLOUD_VIEWER_H
