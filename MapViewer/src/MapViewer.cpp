// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  MapViewer.cpp
 * @brief Map Visualization
 *
 */
// </rtc-template>

#include "MapViewer.h"

#define MAX_PC_COMM 170000
#define MAX_CPC_COMM 80000

// Time
std::chrono::system_clock::time_point startTime1;
std::chrono::system_clock::time_point startTime2;
std::chrono::system_clock::duration maxTimeDifference = std::chrono::seconds(5);

// GNG
struct gng* gng_net;
bool viewer_closed;

bool Contour_flag;
bool Contour_1th_flag;
bool Contour_read_flag;
bool Contour_output_flag;
bool PlanWall_flag;
bool PlanWall_1th_flag;
bool PlanWall_read_flag;
bool PlanWall_output_flag;
int  InputCloud_times1;
int  InputCloud_times2;
bool View_flag1;
bool View_flag2;
bool Contour_closed;
bool PlanWall_closed;
bool SameTimeViewerClosed;
int Switching;
float voxelSize;

// Point Cloud
ColorCloudT::Ptr Contour_cloud;
ColorCloudT::Ptr Empty_cloud;

// Module specification
// <rtc-template block="module_spec">
static const char* const mapviewer_spec[] =
  {
    "implementation_id", "MapViewer",
    "type_name",         "MapViewer",
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
MapViewer::MapViewer(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PlanWallIn("PlanWall", m_PlanWall),
    m_ContourIn("Contour", m_Contour)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
MapViewer::~MapViewer()
{
}

void adjustContourCloudZAxis(ColorCloudT::Ptr cloud) {
    if (!cloud || cloud->points.empty()) {
        std::cerr << "[ERROR] Contour_cloud is empty or not initialized." << std::endl;
        return;
    }

    // z軸の最大値を調べる
    float maxZ = std::numeric_limits<float>::lowest();
    for (const auto& point : cloud->points) {
        if (point.z > maxZ) {
            maxZ = point.z;
        }
    }

    // z軸の最大値が0以上の正の値の場合、点群を移動させる
    if (maxZ > 0 + voxelSize + 0.1) {
        for (auto& point : cloud->points) {
            point.z -= maxZ + voxelSize + 0.1;
        }
        std::cerr << "[INFO] Contour_cloud has been adjusted by " << maxZ + voxelSize + 0.1 << " units in the Z-axis." << std::endl;
    }
    else {
        std::cerr << "[INFO] No adjustment needed for Contour_cloud." << std::endl;
    }
}

void RedirectVTKOutputWindow() {
    // デフォルトのvtkOutputWindowを無効にする
    vtkOutputWindow::SetInstance(nullptr);
    vtkObject::GlobalWarningDisplayOff();
}

// 色付きの点群の可視化
void visualizeCloud(ColorCloudT::Ptr cloud, const std::string& PointName) {
    RedirectVTKOutputWindow();
    std::string ViewerName = PointName + " Viewer";
    pcl::visualization::PCLVisualizer viewer(ViewerName);
    std::cerr << "[INFO] Colored " << ViewerName << " is opened." << std::endl;
    int c0 = 0;
    viewer.removeAllShapes(c0);
    viewer.removeAllPointClouds(c0);
    viewer.createViewPort(0.0, 0.0, 1.0, 1.0, c0);
    viewer.setBackgroundColor(1.0, 1.0, 1.0, c0);
    viewer.addCoordinateSystem(1.0, "coordinate system", c0);
    viewer.initCameraParameters();
    viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // 点ごとにキューブを描画
    int cubeId = 0; // キューブのID
    for (const auto& point : *cloud) {
        // キューブの範囲を計算 (中心を点の座標に設定)
        float x_min = point.x - voxelSize / 2.0f;
        float x_max = point.x + voxelSize / 2.0f;
        float y_min = point.y - voxelSize / 2.0f;
        float y_max = point.y + voxelSize / 2.0f;
        float z_min = point.z - voxelSize / 2.0f;
        float z_max = 0.0 - 0.1; // point.z + voxelSize / 2.0f;

        // RGB情報を取得
        float r = point.r / 255.0f; // 0.0～1.0の範囲に正規化
        float g = point.g / 255.0f;
        float b = point.b / 255.0f;

        // キューブを追加
        std::string cubeName = "cube_" + std::to_string(cubeId++);
        viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, r, g, b, cubeName);
    }
    std::cerr << "[INFO] Please window closed..." << std::endl;
    viewer.spin();
}

void GNGView(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,
    struct gng* net,
    const std::string& PointName) {
    RedirectVTKOutputWindow();
    std::string ViewerName = PointName + " Viewer";
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(ViewerName));
    std::cerr << "[INFO] " << ViewerName << " is opened." << std::endl;
    int c0 = 0;
    viewer->removeAllShapes(c0);
    viewer->removeAllPointClouds(c0);
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, c0);
    viewer->setBackgroundColor(1.0, 1.0, 1.0, c0);
    viewer->addCoordinateSystem(1.0, "coordinate system", c0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // 点ごとにキューブを描画
    int cubeId = 0; // キューブのID
    for (const auto& point : *cloud1) {
        // キューブの範囲を計算 (中心を点の座標に設定)
        float x_min = point.x - voxelSize / 2.0f;
        float x_max = point.x + voxelSize / 2.0f;
        float y_min = point.y - voxelSize / 2.0f;
        float y_max = point.y + voxelSize / 2.0f;
        float z_min = point.z - voxelSize / 2.0f;
        float z_max = 0.0 - 0.1; // point.z + voxelSize / 2.0f;
        
        // RGB情報を取得
        float r = point.r / 255.0f; // 0.0～1.0の範囲に正規化
        float g = point.g / 255.0f;
        float b = point.b / 255.0f;

        // キューブを追加
        std::string cubeName = "cube_" + std::to_string(cubeId++);
        viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max, r, g, b, cubeName);
    }

    // GNGのノードを点として描画
    if (net->node_n > 0) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr nodes(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (int i = 0; i < net->node_n; i++) {
            pcl::PointXYZRGB point;
            point.x = net->node[i][0];
            point.y = net->node[i][1];
            point.z = net->node[i][2];
            point.r = static_cast<uint8_t>(net->node[i][3] * 255);
            point.g = static_cast<uint8_t>(net->node[i][4] * 255);
            point.b = static_cast<uint8_t>(net->node[i][5] * 255);
            nodes->points.push_back(point);
        }
        viewer->addPointCloud(nodes, "nodes");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "nodes");
    }
    else {
        std::cerr << "[INFO] No nodes to display." << std::endl;
    }

    int edges = 0;
    // GNGのエッジを線として描画
    bool edges_exist = false;
    for (int i = 0; i < net->node_n; i++) {
        for (int j = 0; j < net->node_n; j++) {
            if (net->edge[i][j] == 1 && net->edge[j][i] == 1) {
                edges_exist = true;
                pcl::PointXYZ p1, p2;
                p1.x = net->node[i][0];
                p1.y = net->node[i][1];
                p1.z = net->node[i][2];
                p2.x = net->node[j][0];
                p2.y = net->node[j][1];
                p2.z = net->node[j][2];
                edges++;

                std::stringstream ss;
                ss << "line" << i << "_" << j;
                viewer->addLine(p1, p2,
                    net->node[i][3] * 255.0,
                    net->node[i][4] * 255.0,
                    net->node[i][5] * 255.0,
                    ss.str(), c0);
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, ss.str());
            }
        }
    } if (!edges_exist) {
        std::cerr << "[INFO] No edges to display." << std::endl;
    }

    // Start the viewer
    std::cerr << "[INFO] " << PointName << " is shown." << std::endl;
    std::cerr << "[INFO] Please window closed..." << std::endl;
    viewer->spin();
    // Update viewer_closed to true when the window is closed
    viewer_closed = true;
}

void MapViewer::init_m_gng(ClusterTypes::ClusterData& m_PlanWall)
{
    // ポートの初期化
    m_PlanWall.node.length(GNGN); // ノード数の設定
    m_PlanWall.edge.length(GNGN); // エッジ配列の設定
    m_PlanWall.node_n = 0;

    // 各ノードの次元ごとにメモリを確保
    for (int i = 0; i < GNGN; i++) {
        m_PlanWall.node[i].length(DIM);  // ノードの次元
        m_PlanWall.edge[i].length(GNGN); // エッジの次元
    }
    return;
}

void MapViewer::free_m_gng(ClusterTypes::ClusterData& m_PlanWall)
{
    // 各ノードの配列を解放
    for (int i = 0; i < GNGN; i++) {
        m_PlanWall.node[i].length(0); // ノード配列の解放
        m_PlanWall.edge[i].length(0); // エッジ配列の解放
    }

    // メイン配列の解放
    m_PlanWall.node.length(0); // ノード配列全体の解放
    m_PlanWall.edge.length(0); // エッジ配列全体の解放
    m_PlanWall.node_n = 0;

    // メモリ解放が完了
    return;
}

int MapViewer::Switch(const std::string& sw) {
    if (sw == "EMap") {
        return 0;
    } else if (sw == "EMap_PlanWall") {
        return 1;
    } else if (sw == "PlanWall") {
        return 2;
    } else {
        return -1;
    }
}

bool MapViewer::STVClosed(const std::string& sw) {
    if (sw == "false") {
        return false;
    } else if (sw == "true") {
        return true;
    } else {
        return -1;
    }
}

RTC::ReturnCode_t MapViewer::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("PlanWall", m_PlanWallIn);
  addInPort("Contour", m_ContourIn);
  
  // Set OutPort buffer

  
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
  init_m_gng(m_PlanWall);
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MapViewer::onFinalize()
{
    free_m_gng(m_PlanWall);
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t MapViewer::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t MapViewer::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t MapViewer::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating MapViewer...." << std::endl;
    Contour_flag = true;
    Contour_1th_flag = true;
    Contour_read_flag = false;
    Contour_output_flag = false;
    PlanWall_flag = true;
    PlanWall_1th_flag = true;
    PlanWall_read_flag = false;
    PlanWall_output_flag = false;
    View_flag1 = false;
    View_flag2 = false;
    Contour_closed = true;
    PlanWall_closed = true;
    SameTimeViewerClosed = false;
    Switching = 1;
    InputCloud_times1 = 0;
    InputCloud_times2 = 0;
    viewer_closed = false;
    voxelSize = 0.5; // (float)m_GridSize;

    if (gng_net == NULL) {
        gng_net = init_gng();
    }

    Contour_cloud = ColorCloudT::Ptr(new ColorCloudT);
	Empty_cloud   = ColorCloudT::Ptr(new ColorCloudT);
    std::cerr << "[INFO] Activated MapViewer OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t MapViewer::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Deactivating MapViewer...." << std::endl;
    View_flag1 = false;
    View_flag2 = false;
    Contour_closed = true;
    PlanWall_closed = true;
    SameTimeViewerClosed = false;
    Contour_cloud.reset();
	Empty_cloud.reset();
    free_gng(gng_net);
    std::cerr << "[INFO] Deactivated MapViewer OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t MapViewer::onExecute(RTC::UniqueId /*ec_id*/)
{

    if (m_DataLoadOption == "One_at_a_Time") {
        if (Contour_flag) {
            ColorCloudT::Ptr cloud(new ColorCloudT);
            // Contourからデータを読み込む
            if (m_ContourIn.isNew()) {
                m_ContourIn.read();
                if (Contour_1th_flag) {
                    std::cerr << "[INFO] \"Contour\" received!" << std::endl;
                    Contour_1th_flag = false;
                    Contour_closed = false;
                }
                InputCloud_times1++;
                std::cerr << "[INFO] " << InputCloud_times1 << " \"Contour\" is being read..." << std::endl;
                
                // RTCPCLからPCLに型変換
                cloud->is_dense = m_Contour.is_dense;
                cloud->points.resize(m_Contour.width * m_Contour.height);
                float* src = (float*)m_Contour.data.get_buffer();
                for (size_t i = 0; i < cloud->points.size(); i++) {
                    cloud->points[i].x = src[0];
                    cloud->points[i].y = src[1];
                    cloud->points[i].z = src[2];
					cloud->points[i].rgb = src[3];
                    src += 4;
                }
                Contour_read_flag = true;
                std::cerr << "[INFO] " << cloud->points.size() << " points is being read..." << std::endl;
            }
            

            // 現在時刻を取得
            auto currentTime = std::chrono::system_clock::now();

            // 最新データ受信時間の更新
            if (Contour_read_flag) {
                startTime1 = currentTime;
            }
            auto timeDifference = currentTime - startTime1;

            // 一定時間以内に次の点群が受信された場合、Contour_cloudに追加
            if (timeDifference <= maxTimeDifference) {
                *Contour_cloud += *cloud;
                Contour_read_flag = false;
                Contour_output_flag = true;
            }
            else if (Contour_output_flag) {
                // 指定した時間を超えたらループを終了
                Contour_flag = false;
                Contour_output_flag = false;
                View_flag1 = true;
                adjustContourCloudZAxis(Contour_cloud);
            }
            cloud.reset();

            if (View_flag1 && (!Contour_closed)) {
                // Viewer show
                visualizeCloud(Contour_cloud, "Elevation Map");
                Contour_flag = true;
                Contour_1th_flag = true;
                Contour_read_flag = false;
                Contour_output_flag = false;
                View_flag1 = false;
                Contour_closed = true;
                InputCloud_times1 = 0;
                Contour_cloud.reset();
                Contour_cloud = ColorCloudT::Ptr(new ColorCloudT);
            }
        }

        if (PlanWall_flag) {
            // PlanWallからデータを読み込む
            if (m_PlanWallIn.isNew()) {
                m_PlanWallIn.read();
                if (PlanWall_1th_flag) {
                    std::cerr << "[INFO] \"PlanWall\" received!" << std::endl;
                    PlanWall_1th_flag = false;
                    PlanWall_closed = false;
                }
                InputCloud_times2++;
                std::cerr << "[INFO] " << InputCloud_times2 << " \"PlanWall\" is being read..." << std::endl;

                // PlanWallからgng_netに格納
                gng_net->node_n = m_PlanWall.node_n;
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < DIM; j++) {
                        gng_net->node[i][j] = m_PlanWall.node[i][j];
                    }
                }
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < GNGN; j++) {
                        gng_net->edge[i][j] = m_PlanWall.edge[i][j];
                    }
                }
                PlanWall_read_flag = true;
            }

            // 現在時刻を取得
            auto currentTime = std::chrono::system_clock::now();

            // 最新データ受信時間の更新
            if (PlanWall_read_flag) {
                startTime2 = currentTime;
            }
            auto timeDifference = currentTime - startTime2;

            // 一定時間以内に次の点群が受信された場合
            if (timeDifference <= maxTimeDifference) {
                PlanWall_read_flag = false;
                PlanWall_output_flag = true;
            }
            else if (PlanWall_output_flag) {
                // 指定した時間を超えたらループを終了
                PlanWall_flag = false;
                PlanWall_output_flag = false;
                View_flag2 = true;
            }

            if (View_flag2 && (!PlanWall_closed)) {
                // Viewer show
                GNGView(Empty_cloud, gng_net, "PlanWall");

                PlanWall_flag = true;
                PlanWall_1th_flag = true;
                PlanWall_read_flag = false;
                PlanWall_output_flag = false;
                View_flag2 = false;
                PlanWall_closed = true;
                InputCloud_times2 = 0;
            }
        }
    }
    else if (m_DataLoadOption == "SameTime") {
        if (Contour_flag) {
            ColorCloudT::Ptr cloud(new ColorCloudT);
            // Contourからデータを読み込む
            if (m_ContourIn.isNew()) {
                m_ContourIn.read();
                if (Contour_1th_flag) {
                    std::cerr << "[INFO] \"Contour\" received!" << std::endl;
                    Contour_1th_flag = false;
                }
                InputCloud_times1++;
                std::cerr << "[INFO] " << InputCloud_times1 << " \"Contour\" is being read..." << std::endl;

                // RTCPCLからPCLに型変換
                cloud->is_dense = m_Contour.is_dense;
                cloud->points.resize(m_Contour.width * m_Contour.height);
                float* src = (float*)m_Contour.data.get_buffer();
                for (size_t i = 0; i < cloud->points.size(); i++) {
                    cloud->points[i].x = src[0];
                    cloud->points[i].y = src[1];
                    cloud->points[i].z = src[2];
                    cloud->points[i].rgb = src[3];
                    src += 4;
                }
                Contour_read_flag = true;
            }

            // 現在時刻を取得
            auto currentTime = std::chrono::system_clock::now();

            // 最新データ受信時間の更新
            if (Contour_read_flag) {
                startTime1 = currentTime;
            }
            auto timeDifference = currentTime - startTime1;

            // 一定時間以内に次の点群が受信された場合、Contour_cloudに追加
            if (timeDifference <= maxTimeDifference) {
                *Contour_cloud += *cloud;
                Contour_read_flag = false;
                Contour_output_flag = true;
            }
            else if (Contour_output_flag) {
                // 指定した時間を超えたらループを終了
                Contour_flag = false;
                Contour_output_flag = false;
                View_flag1 = true;
                adjustContourCloudZAxis(Contour_cloud);
            }
            cloud.reset();
        }

        if (PlanWall_flag) {
            // PlanWallからデータを読み込む
            if (m_PlanWallIn.isNew()) {
                m_PlanWallIn.read();
                if (PlanWall_1th_flag) {
                    std::cerr << "[INFO] \"PlanWall\" received!" << std::endl;
                    PlanWall_1th_flag = false;
                }
                InputCloud_times2++;
                std::cerr << "[INFO] " << InputCloud_times2 << " \"PlanWall\" is being read..." << std::endl;

                // PlanWallからgng_netに格納
                gng_net->node_n = m_PlanWall.node_n;
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < DIM; j++) {
                        gng_net->node[i][j] = m_PlanWall.node[i][j];
                    }
                }
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < GNGN; j++) {
                        gng_net->edge[i][j] = m_PlanWall.edge[i][j];
                    }
                }
                PlanWall_read_flag = true;
            }

            // 現在時刻を取得
            auto currentTime = std::chrono::system_clock::now();

            // 最新データ受信時間の更新
            if (PlanWall_read_flag) {
                startTime2 = currentTime;
            }
            auto timeDifference = currentTime - startTime2;

            // 一定時間以内に次の点群が受信された場合
            if (timeDifference <= maxTimeDifference) {
                PlanWall_read_flag = false;
                PlanWall_output_flag = true;
            }
            else if (PlanWall_output_flag) {
                // 指定した時間を超えたらループを終了
                PlanWall_flag = false;
                PlanWall_output_flag = false;
                View_flag2 = true;
            }
        }
    }

    SameTimeViewerClosed = STVClosed(m_SameTimeViewerClosed);
    if ((m_DataLoadOption == "SameTime")
        && View_flag1 && View_flag2
        && !SameTimeViewerClosed) {

        // Viewer show
        if (!SameTimeViewerClosed) {
            std::cerr << "[INFO] If you do not want to continue the display, switch setting “SameTimeViewerClosed”." << std::endl;
            std::cerr << "[INFO] Please switch the display method setting “Switching”." << std::endl;
            Switching = Switch(m_Switching);
            if (Switching == 0) {
                // Elevation Map
                visualizeCloud(Contour_cloud, "Elevation Map");
            } else if (Switching == 1) {
                // Elevation Map and Wall DTC
                GNGView(Contour_cloud, gng_net, "Elevation Map and PlanWall");
            } else if (Switching == 2) {
                // Wall DTC
                GNGView(Empty_cloud, gng_net, "PlanWall");
            }
        } else{
            Contour_flag = true;
            Contour_1th_flag = true;
            Contour_read_flag = false;
            Contour_output_flag = false;
            PlanWall_flag = true;
            PlanWall_1th_flag = true;
            PlanWall_read_flag = false;
            PlanWall_output_flag = false;
            View_flag1 = false;
            View_flag2 = false;
            Contour_closed = true;
            PlanWall_closed = true;
            InputCloud_times1 = 0;
            InputCloud_times2 = 0;
            Contour_cloud.reset();
            Contour_cloud = ColorCloudT::Ptr(new ColorCloudT);
        }
        Sleep(2 * 1000);
    }

    return RTC::RTC_OK;
}


//RTC::ReturnCode_t MapViewer::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t MapViewer::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MapViewer::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t MapViewer::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t MapViewer::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void MapViewerInit(RTC::Manager* manager)
  {
    coil::Properties profile(mapviewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<MapViewer>,
                             RTC::Delete<MapViewer>);
  }
  
}
