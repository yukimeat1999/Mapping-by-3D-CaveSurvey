// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  PointCloud_Viewer.cpp
 * @brief PointCloud Viewer
 *
 */
// </rtc-template>

#include "PointCloud_Viewer.h"

// Localization
pcl::PointXYZ Localization_;
std::string FILE_NAME;

// PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr new_PointCloud_;
pcl::PointCloud<pcl::PointXYZ>::Ptr merge_PointCloud_;
pcl::PointCloud<pcl::PointXYZ>::Ptr analyses_PointCloud_;
pcl::PointCloud<pcl::PointXYZ>::Ptr Empty_cloud;

// Time
std::chrono::system_clock::time_point startTime1;
std::chrono::system_clock::time_point startTime2;
std::chrono::system_clock::time_point startTime3;
std::chrono::system_clock::duration maxTimeDifference = std::chrono::seconds(3);

// GNG
struct gng* gng_net;
bool viewer_closed;

bool new_PC_flag;
bool new_PC_1th_flag;
bool new_PC_read_flag;
bool new_PC_output_flag;
bool merge_PC_flag;
bool merge_PC_1th_flag;
bool merge_PC_read_flag;
bool merge_PC_output_flag;
bool analyses_PC_flag;
bool analyses_PC_1th_flag;
bool analyses_PC_read_flag;
bool analyses_PC_output_flag;
int  InputCloud_times1;
int  InputCloud_times2;
int  InputCloud_times3;
bool View_flagf;
bool View_flag1;
bool View_flag2;
bool View_flag3;
bool ST_closed;
bool new_closed;
bool merge_closed;
bool analyses_closed;
bool Scan_1th;


// Module specification
// <rtc-template block="module_spec">
static const char* const pointcloud_viewer_spec[] =
  {
    "implementation_id", "PointCloud_Viewer",
    "type_name",         "PointCloud_Viewer",
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
PointCloud_Viewer::PointCloud_Viewer(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_new_PointCloudIn("new_PointCloud", m_new_PointCloud),
    m_merge_PointCloudIn("merge_PointCloud", m_merge_PointCloud),
    m_analyses_ClusterIn("analyses_Cluster", m_analyses_Cluster),
    m_LocalizationIn("Localization", m_Localization)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PointCloud_Viewer::~PointCloud_Viewer()
{
}



RTC::ReturnCode_t PointCloud_Viewer::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("new_PointCloud", m_new_PointCloudIn);
  addInPort("merge_PointCloud", m_merge_PointCloudIn);
  addInPort("analyses_Cluster", m_analyses_ClusterIn);
  addInPort("Localization", m_LocalizationIn);
  
  // Set OutPort buffer

  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("DataLoadOption", m_DataLoadOption, "SameTime");
  bindParameter("FILE_NAME", m_FILE_NAME, "Your_PointCloud_File.ply");
  // </rtc-template>
  init_m_gng(m_analyses_Cluster);
  
  return RTC::RTC_OK;
}


void RedirectVTKOutputWindow() {
    vtkOutputWindow::SetInstance(nullptr); // デフォルトのvtkOutputWindowを無効にする
    vtkObject::GlobalWarningDisplayOff();
}

void PointCloudView(const std::string& FileName) {
    std::cerr << "[INFO] New PointCloud File received!" << std::endl;
    std::cerr << "[INFO] PointCloud File is being read...." << std::endl;
    std::cerr << "[INFO] Read " << FileName << std::endl;

    RedirectVTKOutputWindow();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // PLYファイルから点群データを読み込む
    pcl::io::loadPLYFile(FileName, *cloud);
    std::cerr << "[INFO] Reading of PointCloud File is completed!" << std::endl;

    // PCLビューアーを作成
    pcl::visualization::CloudViewer viewer("PointCloud File Viewer");
    std::cerr << "[INFO] Viewer is opened." << std::endl;

    // ウィンドウ上に点群データを描画
    viewer.showCloud(cloud);
    std::cerr << "[INFO] PointCloud File is shown!" << std::endl;

    std::cerr << "[INFO] Please Window is closed." << std::endl;
    while (!viewer.wasStopped()) {
        // ウィンドウが閉じられるまで待機
    }
}

void PointCloudView(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& PortName) {

    RedirectVTKOutputWindow();

    std::cerr << "[INFO] Reading of " << PortName << " is completed!" << std::endl;

    // Create viewer name
    std::string ViewerName = PortName + " Viewer";

    // Create PCL viewer
    pcl::visualization::CloudViewer viewer(ViewerName);
    std::cerr << "[INFO] " << PortName << " Viewer is opened." << std::endl;

    // ウィンドウ上に点群データを描画
    viewer.showCloud(cloud);// cloud);
    std::cerr << "[INFO] " << PortName << " is shown." << std::endl;

    std::cerr << "[INFO] Please Window is closed." << std::endl;
    while (!viewer.wasStopped()) {
        // ウィンドウが閉じられるまで待機
    }
}

void PointCloudView(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    const std::string& PortName,
                    const pcl::PointXYZ Point) {

    RedirectVTKOutputWindow();

    std::cerr << "[INFO] Reading of " << PortName << " is completed!" << std::endl;

    // Create viewer name
    std::string ViewerName = PortName + " Viewer";

    // Create PCL viewer
    pcl::visualization::PCLVisualizer viewer(ViewerName);
    std::cerr << "[INFO] " << PortName << " Viewer is opened." << std::endl;

    int c0 = 0;

    // Set the background color of both viewports
    viewer.createViewPort(0.0, 0.0, 1.0, 1.0, c0);

    // Set background color
    viewer.setBackgroundColor(0.2, 0.2, 0.2, c0);
    viewer.addCoordinateSystem(1.0, "coordinate system", 0);

    // 以前の点群データを削除
    viewer.removePointCloud("cloud", c0);
    viewer.removePointCloud("red_sphere", c0);

    // Add the loaded point cloud to the viewer
    viewer.addPointCloud(cloud, "cloud", c0);// cloud

    // Set sphere radius
    double sphere_radius = 0.05;

    // Add a red sphere at the specified position
    viewer.addSphere(Point, sphere_radius, 1.0, 0.0, 0.0, "red_sphere", c0);

    // Start the viewer
    viewer.spin();
    std::cerr << "[INFO] " << PortName << " is shown." << std::endl;

    std::cerr << "[INFO] Please Window is closed." << std::endl;
    while (!viewer.wasStopped()) {
        // ウィンドウが閉じられるまで待機
    }
}

void PointCloudView(const pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud0,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud1,
                    struct gng* net,
                    const pcl::PointXYZ Point) {

    RedirectVTKOutputWindow();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    std::cerr << "[INFO] Reading of PointClouds is completed!" << std::endl;

    // Create PCL viewer
    pcl::visualization::PCLVisualizer viewer("SameTime Viewer");
    std::cerr << "[INFO] PointClouds Viewer is opened." << std::endl;

    int c0 = 0;
    int c1 = 1;
    int c2 = 2;
    int c3 = 3;

    // Set the background color of both viewports
    viewer.createViewPort(0.0, 0.5, 0.5, 1.0, c0);
    viewer.createViewPort(0.5, 0.5, 1.0, 1.0, c1);
    viewer.createViewPort(0.0, 0.0, 0.5, 0.5, c2);
    viewer.createViewPort(0.5, 0.0, 1.0, 0.5, c3);

    // Set background color
    viewer.setBackgroundColor(0.2, 0.2, 0.2, c0);
    viewer.setBackgroundColor(0.0, 0.0, 0.0, c1);
    viewer.setBackgroundColor(0.2, 0.2, 0.2, c2);
    viewer.setBackgroundColor(0.2, 0.2, 0.2, c3);

    // 以前の点群データを削除
    viewer.removePointCloud("cloud0");
    viewer.removePointCloud("cloud1");
    viewer.removePointCloud("cloud3");
    viewer.removePointCloud("red_sphere");
    // 古いノードとエッジを削除
    viewer.removeAllShapes(c3);
    viewer.removeAllPointClouds(c3);

    // Add the loaded point cloud to the viewer
    viewer.addPointCloud(PointCloud0, "cloud0", c0);
    viewer.addPointCloud(cloud3     , "cloud3", c1);
    viewer.addPointCloud(PointCloud1, "cloud1", c2);
    viewer.addCoordinateSystem(1.0, "coordinate system", c0);
    viewer.addCoordinateSystem(1.0, "coordinate system", c2);
    viewer.addCoordinateSystem(1.0, "coordinate system", c3);

    // Set sphere radius
    double sphere_radius = 0.05;

    // Add a red sphere at the specified position
    viewer.addSphere(Point, sphere_radius, 1.0, 0.0, 0.0, "red_sphere", c2);

    // Set GNG
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
        viewer.addPointCloud(nodes, "nodes", c3);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "nodes", c3);
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
                viewer.addLine(p1, p2,
                    net->node[i][3] * 255.0,
                    net->node[i][4] * 255.0,
                    net->node[i][5] * 255.0,
                    ss.str(), c3);
                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, ss.str(), c3);
            }
        }
    } if (!edges_exist) {
        std::cerr << "[INFO] No edges to display." << std::endl;
    }

    // Start the viewer
    viewer.spin();
    std::cerr << "[INFO] PointClouds is shown." << std::endl;

    std::cerr << "[INFO] Please Window is closed." << std::endl;
    while (!viewer.wasStopped()) {
        // ウィンドウが閉じられるまで待機
    }
}

// 点群とGNGのノードとエッジを表示
void GNGView(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
             struct gng* net,
             const std::string& PointName) {
    RedirectVTKOutputWindow();
    // Create viewer name
    std::string ViewerName = PointName + " Viewer";
    // Create PCL viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(ViewerName));
    std::cerr << "[INFO] " << ViewerName << " is opened." << std::endl;

    int c0 = 0;
    // 古いノードとエッジを削除
    viewer->removeAllShapes(c0);
    viewer->removeAllPointClouds(c0);
    // Setting viewport borders
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, c0);
    // Set background color
    viewer->setBackgroundColor(0.2, 0.2, 0.2, c0);
    // Delete previous PointCloud data
    viewer->removePointCloud("cloud1", c0);
    // Add the loaded PointCloud to the viewer
    viewer->addPointCloud(cloud1, "cloud1", c0);
    // Set point size
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1", c0);
    viewer->addCoordinateSystem(1.0, "coordinate system", 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0.0, 0.0, 100.0, // カメラ位置 (x, y, z)
                              0.0, 0.0, 0.0,   // 注視点 (x, y, z)
                              0.0, 1.0, 0.0);  // カメラの上方向ベクトル (x, y, z)

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
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "nodes", c0);
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
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, ss.str(), c0);
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

void PointCloud_Viewer::init_m_gng(ClusterTypes::ClusterData& m_analyses_Cluster)
{
    // ポートの初期化
    m_analyses_Cluster.node.length(GNGN); // ノード数の設定
    m_analyses_Cluster.edge.length(GNGN); // エッジ配列の設定
    m_analyses_Cluster.node_n = 0;

    // 各ノードの次元ごとにメモリを確保
    for (int i = 0; i < GNGN; i++) {
        m_analyses_Cluster.node[i].length(DIM);  // ノードの次元
        m_analyses_Cluster.edge[i].length(GNGN); // エッジの次元
    }
    return;
}

void PointCloud_Viewer::free_m_gng(ClusterTypes::ClusterData& m_analyses_Cluster)
{
    // 各ノードの配列を解放
    for (int i = 0; i < GNGN; i++) {
        m_analyses_Cluster.node[i].length(0); // ノード配列の解放
        m_analyses_Cluster.edge[i].length(0); // エッジ配列の解放
    }

    // メイン配列の解放
    m_analyses_Cluster.node.length(0); // ノード配列全体の解放
    m_analyses_Cluster.edge.length(0); // エッジ配列全体の解放
    m_analyses_Cluster.node_n = 0;

    // メモリ解放が完了
    return;
}

RTC::ReturnCode_t PointCloud_Viewer::onFinalize()
{
    free_m_gng(m_analyses_Cluster);
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_Viewer::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_Viewer::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t PointCloud_Viewer::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating PointCloud_Viewer...." << std::endl;

    if (Scan_1th) {
        Localization_.x = 0;
        Localization_.y = 0;
        Localization_.z = 0;
    }

    new_PC_flag        = true;
    new_PC_1th_flag    = true;
    new_PC_read_flag   = false;
    new_PC_output_flag = false;
    merge_PC_flag        = true;
    merge_PC_1th_flag    = true;
    merge_PC_read_flag   = false;
    merge_PC_output_flag = false;
    analyses_PC_flag        = true;
    analyses_PC_1th_flag    = true;
    analyses_PC_read_flag   = false;
    analyses_PC_output_flag = false;
    View_flagf = true;
    View_flag1 = false;
    View_flag2 = false;
    View_flag3 = false;
    ST_closed  = false;
    new_closed      = true;
    merge_closed    = true;
    analyses_closed = true;
    viewer_closed   = false;

    InputCloud_times1 = 0;
    InputCloud_times2 = 0;
    InputCloud_times3 = 0;

    if (gng_net == NULL) {
        gng_net = init_gng();
    }

    new_PointCloud_      = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    merge_PointCloud_    = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    Empty_cloud          = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "[INFO] Activated PointCloud_Viewer OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_Viewer::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Deactivating PointCloud_Viewer...." << std::endl;
    View_flagf = true;
    View_flag1 = false;
    View_flag2 = false;
    View_flag3 = false;
    ST_closed  = false;
    new_closed      = true;
    merge_closed    = true;
    analyses_closed = true;

    new_PointCloud_.reset();
    merge_PointCloud_.reset();
    analyses_PointCloud_.reset();
    Empty_cloud.reset();
    free_gng(gng_net);
    std::cerr << "[INFO] Deactivated PointCloud_Viewer OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_Viewer::onExecute(RTC::UniqueId /*ec_id*/)
{
    FILE_NAME = m_FILE_NAME;
    if (m_DataLoadOption == "One_at_a_Time") {
        if (new_PC_flag) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            // new_PointCloudからデータを読み込む
            if (m_new_PointCloudIn.isNew()) {
                m_new_PointCloudIn.read();
                if (new_PC_1th_flag) {
                    std::cerr << "[INFO] \"new_PointCloud\" received!" << std::endl;
                    new_PC_1th_flag = false;
                    new_closed      = false;
                }
                InputCloud_times1++;
                std::cerr << "[INFO] " << InputCloud_times1 << " \"new_PointCloud\" is being read..." << std::endl;

                // RTCPCLからPCLに型変換
                cloud->is_dense = m_new_PointCloud.is_dense;
                cloud->points.resize(m_new_PointCloud.width * m_new_PointCloud.height);
                float* src = (float*)m_new_PointCloud.data.get_buffer();
                for (size_t i = 0; i < cloud->points.size(); i++) {
                    cloud->points[i].x = src[0];
                    cloud->points[i].y = src[1];
                    cloud->points[i].z = src[2];
                    src += 3;
                }
                new_PC_read_flag = true;
            }

            // 現在時刻を取得
            auto currentTime = std::chrono::system_clock::now();

            // 最新データ受信時間の更新
            if (new_PC_read_flag) {
                startTime1 = currentTime;
            }
            auto timeDifference = currentTime - startTime1;

            // 一定時間以内に次の点群が受信された場合、new_PointCloud_に追加
            if (timeDifference <= maxTimeDifference) {
                *new_PointCloud_ += *cloud;
                new_PC_read_flag   = false;
                new_PC_output_flag = true;
            }
            else if (new_PC_output_flag) {
                // 指定した時間を超えたらループを終了
                new_PC_flag        = false;
                new_PC_output_flag = false;
                View_flag1         = true;
            }
            cloud.reset();

            if (View_flag1 && (!new_closed)) {
                // Viewer show
                PointCloudView(new_PointCloud_, "new_PointCloud");
                new_PC_flag        = true;
                new_PC_1th_flag    = true;
                new_PC_read_flag   = false;
                new_PC_output_flag = false;
                View_flag1         = false;
                new_closed         = true;
                InputCloud_times1 = 0;
                new_PointCloud_.reset();
                new_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            }
        }

        if (merge_PC_flag) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            // merge_PointCloudからデータを読み込む
            if (m_merge_PointCloudIn.isNew()) {
                m_merge_PointCloudIn.read();
                if (merge_PC_1th_flag) {
                    std::cerr << "[INFO] \"merge_PointCloud\" received!" << std::endl;
                    merge_PC_1th_flag = false;
                    merge_closed      = false;
                }
                InputCloud_times2++;
                std::cerr << "[INFO] " << InputCloud_times2 << " \"merge_PointCloud\" is being read..." << std::endl;

                // RTCPCLからPCLに型変換
                cloud->is_dense = m_merge_PointCloud.is_dense;
                cloud->points.resize(m_merge_PointCloud.width * m_merge_PointCloud.height);
                float* src = (float*)m_merge_PointCloud.data.get_buffer();
                for (size_t i = 0; i < cloud->points.size(); i++) {
                    cloud->points[i].x = src[0];
                    cloud->points[i].y = src[1];
                    cloud->points[i].z = src[2];
                    src += 3;
                }
                merge_PC_read_flag = true;
            }

            // 現在時刻を取得
            auto currentTime = std::chrono::system_clock::now();

            if (merge_PC_read_flag) {
                // 最新データ受信時間の更新
                startTime2 = currentTime;

                // Localizationの読み込み
                if (m_LocalizationIn.isNew()) {
                    m_LocalizationIn.read();
                    Localization_.x = m_Localization.data.position.x;
                    Localization_.y = m_Localization.data.position.y;
                    Localization_.z = m_Localization.data.position.z;
                }
            }
            auto timeDifference = currentTime - startTime2;

            // 一定時間以内に次の点群が受信された場合、merge_PointCloud_に追加
            if (timeDifference <= maxTimeDifference) {
                *merge_PointCloud_ += *cloud;
                merge_PC_read_flag   = false;
                merge_PC_output_flag = true;
            }
            else if (merge_PC_output_flag) {
                // 指定した時間を超えたらループを終了
                merge_PC_flag        = false;
                merge_PC_output_flag = false;
                View_flag2           = true;
            }
            cloud.reset();

            if (View_flag2 && (!merge_closed)) {
                // Viewer show
                PointCloudView(merge_PointCloud_, "merge_PointCloud", Localization_);
                merge_PC_flag        = true;
                merge_PC_1th_flag    = true;
                merge_PC_read_flag   = false;
                merge_PC_output_flag = false;
                merge_closed         = true;
                View_flag2           = false;
                InputCloud_times2 = 0;
                merge_PointCloud_.reset();
                merge_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            }
        }

        if (analyses_PC_flag) {
            // analyses_Clusterからデータを読み込む
            if (m_analyses_ClusterIn.isNew()) {
                m_analyses_ClusterIn.read();
                if (analyses_PC_1th_flag) {
                    std::cerr << "[INFO] \"analyses\" received!" << std::endl;
                    analyses_PC_1th_flag = false;
                    analyses_closed      = false;
                }
                InputCloud_times2++;
                std::cerr << "[INFO] " << InputCloud_times2 << " \"analyses\" is being read..." << std::endl;

                // analysesからgng_netに格納
                gng_net->node_n = m_analyses_Cluster.node_n;
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < DIM; j++) {
                        gng_net->node[i][j] = m_analyses_Cluster.node[i][j];
                    }
                }
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < GNGN; j++) {
                        gng_net->edge[i][j] = m_analyses_Cluster.edge[i][j];
                    }
                }
                analyses_PC_read_flag = true;
            }

            // 現在時刻を取得
            auto currentTime = std::chrono::system_clock::now();

            // 最新データ受信時間の更新
            if (analyses_PC_read_flag) {
                startTime3 = currentTime;
            }
            auto timeDifference = currentTime - startTime3;

            // 一定時間以内に次の点群が受信された場合、analyses_PointCloud_に追加
            if (timeDifference <= maxTimeDifference) {
                analyses_PC_read_flag   = false;
                analyses_PC_output_flag = true;
            }
            else if (analyses_PC_output_flag) {
                // 指定した時間を超えたらループを終了
                analyses_PC_flag        = false;
                analyses_PC_output_flag = false;
                View_flag3              = true;
            }

            if (View_flag3 && (!analyses_closed)) {
                // Viewer show
                GNGView(Empty_cloud, gng_net, "analyses_Cluster");
                analyses_PC_flag        = true;
                analyses_PC_1th_flag    = true;
                analyses_PC_read_flag   = false;
                analyses_PC_output_flag = false;
                analyses_closed         = true;
                View_flag3              = false;
                InputCloud_times3 = 0;
                analyses_PointCloud_.reset();
                analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            }
        }
    }
    else if (m_DataLoadOption == "SameTime") {
        if (!ST_closed) {
            if (new_PC_flag) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                // new_PointCloudからデータを読み込む
                if (m_new_PointCloudIn.isNew()) {
                    m_new_PointCloudIn.read();
                    if (new_PC_1th_flag) {
                        std::cerr << "[INFO] \"new_PointCloud\" received!" << std::endl;
                        new_PC_1th_flag = false;
                    }
                    InputCloud_times1++;
                    std::cerr << "[INFO] " << InputCloud_times1 << " \"new_PointCloud\" is being read..." << std::endl;
                
                    // RTCPCLからPCLに型変換
                    cloud->is_dense = m_new_PointCloud.is_dense;
                    cloud->points.resize(m_new_PointCloud.width * m_new_PointCloud.height);
                    float* src = (float*)m_new_PointCloud.data.get_buffer();
                    for (size_t i = 0; i < cloud->points.size(); i++) {
                        cloud->points[i].x = src[0];
                        cloud->points[i].y = src[1];
                        cloud->points[i].z = src[2];
                        src += 3;
                    }
                    new_PC_read_flag = true;
                }
            
                // 現在時刻を取得
                auto currentTime = std::chrono::system_clock::now();

                // 最新データ受信時間の更新
                if (new_PC_read_flag) {
                    startTime1 = currentTime;
                }
                auto timeDifference = currentTime - startTime1;

                // 一定時間以内に次の点群が受信された場合、new_PointCloud_に追加
                if (timeDifference <= maxTimeDifference) {
                    *new_PointCloud_ += *cloud;
                    new_PC_read_flag   = false;
                    new_PC_output_flag = true;
                }
                else if (new_PC_output_flag) {
                    // 指定した時間を超えたらループを終了
                    new_PC_flag        = false;
                    new_PC_output_flag = false;
                    View_flag1         = true;
                }
                cloud.reset();
            }

            if (merge_PC_flag) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                // merge_PointCloudからデータを読み込む
                if (m_merge_PointCloudIn.isNew()) {
                    m_merge_PointCloudIn.read();
                    if (merge_PC_1th_flag) {
                        std::cerr << "[INFO] \"merge_PointCloud\" received!" << std::endl;
                        merge_PC_1th_flag = false;
                    }
                    InputCloud_times2++;
                    std::cerr << "[INFO] " << InputCloud_times2 << " \"merge_PointCloud\" is being read..." << std::endl;

                    // RTCPCLからPCLに型変換
                    cloud->is_dense = m_merge_PointCloud.is_dense;
                    cloud->points.resize(m_merge_PointCloud.width * m_merge_PointCloud.height);
                    float* src = (float*)m_merge_PointCloud.data.get_buffer();
                    for (size_t i = 0; i < cloud->points.size(); i++) {
                        cloud->points[i].x = src[0];
                        cloud->points[i].y = src[1];
                        cloud->points[i].z = src[2];
                        src += 3;
                    }
                    merge_PC_read_flag = true;
                }

                // 現在時刻を取得
                auto currentTime = std::chrono::system_clock::now();

                if (merge_PC_read_flag) {
                    // 最新データ受信時間の更新
                    startTime2 = currentTime;

                    // Localizationの読み込み
                    if (m_LocalizationIn.isNew()) {
                        m_LocalizationIn.read();
                        Localization_.x = m_Localization.data.position.x;
                        Localization_.y = m_Localization.data.position.y;
                        Localization_.z = m_Localization.data.position.z;
                    }
                }
                auto timeDifference = currentTime - startTime2;

                // 一定時間以内に次の点群が受信された場合、merge_PointCloud_に追加
                if (timeDifference <= maxTimeDifference) {
                    *merge_PointCloud_ += *cloud;
                    merge_PC_read_flag   = false;
                    merge_PC_output_flag = true;
                }
                else if (merge_PC_output_flag) {
                    // 指定した時間を超えたらループを終了
                    merge_PC_flag        = false;
                    merge_PC_output_flag = false;
                    View_flag2           = true;
                }
                cloud.reset();
            }

            if (analyses_PC_flag) {
                // analyses_Clusterからデータを読み込む
                if (m_analyses_ClusterIn.isNew()) {
                    m_analyses_ClusterIn.read();
                    if (analyses_PC_1th_flag) {
                        std::cerr << "[INFO] \"analyses\" received!" << std::endl;
                        analyses_PC_1th_flag = false;
                        analyses_closed = false;
                    }
                    InputCloud_times2++;
                    std::cerr << "[INFO] " << InputCloud_times2 << " \"analyses\" is being read..." << std::endl;

                    // analysesからgng_netに格納
                    gng_net->node_n = m_analyses_Cluster.node_n;
                    for (int i = 0; i < GNGN; i++) {
                        for (int j = 0; j < DIM; j++) {
                            gng_net->node[i][j] = m_analyses_Cluster.node[i][j];
                        }
                    }
                    for (int i = 0; i < GNGN; i++) {
                        for (int j = 0; j < GNGN; j++) {
                            gng_net->edge[i][j] = m_analyses_Cluster.edge[i][j];
                        }
                    }
                    analyses_PC_read_flag = true;
                }

                // 現在時刻を取得
                auto currentTime = std::chrono::system_clock::now();

                // 最新データ受信時間の更新
                if (analyses_PC_read_flag) {
                    startTime3 = currentTime;
                }
                auto timeDifference = currentTime - startTime3;

                // 一定時間以内に次の点群が受信された場合、analyses_PointCloud_に追加
                if (timeDifference <= maxTimeDifference) {
                    analyses_PC_read_flag   = false;
                    analyses_PC_output_flag = true;
                }
                else if (analyses_PC_output_flag) {
                    // 指定した時間を超えたらループを終了
                    analyses_PC_flag        = false;
                    analyses_PC_output_flag = false;
                    View_flag3              = true;
                }
            }
        }

        if (View_flag1 && (!View_flag2) && (!View_flag3) && (!Scan_1th)) {
            // Viewer show on new_PointCloud_ only
            PointCloudView(new_PointCloud_, merge_PointCloud_, gng_net, Localization_);

            new_PC_flag        = true;
            new_PC_1th_flag    = true;
            new_PC_read_flag   = false;
            new_PC_output_flag = false;
            View_flag1 = false;
            ST_closed  = false;
            new_closed = true;
            Scan_1th   = true;
            InputCloud_times1 = 0;
            new_PointCloud_.reset();
            new_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        } else if (View_flag1 && View_flag2 && View_flag3) {
            // Viewer show
            PointCloudView(new_PointCloud_, merge_PointCloud_, gng_net, Localization_);

            new_PC_flag        = true;
            new_PC_1th_flag    = true;
            new_PC_read_flag   = false;
            new_PC_output_flag = false;
            merge_PC_flag        = true;
            merge_PC_1th_flag    = true;
            merge_PC_read_flag   = false;
            merge_PC_output_flag = false;
            analyses_PC_flag        = true;
            analyses_PC_1th_flag    = true;
            analyses_PC_read_flag   = false;
            analyses_PC_output_flag = false;
            View_flagf = true;
            View_flag1 = false;
            View_flag2 = false;
            View_flag3 = false;
            ST_closed  = false;
            new_closed      = true;
            merge_closed    = true;
            analyses_closed = true;
            InputCloud_times1 = 0;
            InputCloud_times2 = 0;
            InputCloud_times3 = 0;
            new_PointCloud_.reset();
            merge_PointCloud_.reset();
            analyses_PointCloud_.reset();
            new_PointCloud_      = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            merge_PointCloud_    = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            Sleep(2 * 1000);
            //ST_closed  = true;
        }
    }
    else if (m_DataLoadOption == "File") {
        if (View_flagf) {
            FILE_NAME = m_FILE_NAME;
            PointCloudView(FILE_NAME);
            View_flagf = false;
        }
    }
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_Viewer::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_Viewer::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t PointCloud_Viewer::onReset(RTC::UniqueId /*ec_id*/)
{
    Scan_1th = false;

    Localization_.x = 0;
    Localization_.y = 0;
    Localization_.z = 0;

    return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_Viewer::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_Viewer::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void PointCloud_ViewerInit(RTC::Manager* manager)
  {
    coil::Properties profile(pointcloud_viewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<PointCloud_Viewer>,
                             RTC::Delete<PointCloud_Viewer>);
  }
  
}
