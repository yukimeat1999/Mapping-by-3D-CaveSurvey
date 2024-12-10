// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  Contour.cpp
 * @brief Elevation Map Generation
 *
 */
// </rtc-template>

#include "Contour.h"

#define MAX_PC_COMM 170000
#define MAX_CPC_COMM 80000

// PointCloud
CloudT::Ptr new_cloud;
ColorCloudT::Ptr out_cloud;
CloudT::Ptr Empty_cloud;

// Time
std::chrono::system_clock::time_point startTime;
std::chrono::system_clock::duration maxTimeDifference = std::chrono::seconds(3);

bool new_PC_flag;
bool new_PC_1th_flag;
bool new_PC_read_flag;
bool new_PC_output_flag;
bool PC_Read_flag;
bool viewer_closed;
int  InputCloud_times;
float voxelSize;

// Module specification
// <rtc-template block="module_spec">
static const char* const contour_spec[] =
  {
    "implementation_id", "Contour",
    "type_name",         "Contour",
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
Contour::Contour(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PCDIn("PCD", m_PCD),
    m_ContourOut("Contour", m_Contour)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Contour::~Contour()
{
}

void test(const CloudT::Ptr& input_cloud, CloudT::Ptr& output_cloud) {
    CloudT::Ptr cloud_test;
    cloud_test = CloudT::Ptr(new CloudT);
    cloud_test = input_cloud;
    output_cloud = cloud_test;
    cloud_test.reset();
}

// 疑似カラーの色を計算
void calculateColor(float value, float min, float max, uint8_t& r, uint8_t& g, uint8_t& b) {
    float ratio = 2.0f * (value - min) / (max - min);
    b = static_cast<uint8_t>(std::max(0.0f, 255.0f * (1 - ratio)));
    r = static_cast<uint8_t>(std::max(0.0f, 255.0f * (ratio - 1)));
    g = 255 - b - r;
}

RTC::ReturnCode_t Contour::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("PCD", m_PCDIn);
  
  // Set OutPort buffer
  addOutPort("Contour", m_ContourOut);

  
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


RTC::ReturnCode_t Contour::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t Contour::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Contour::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t Contour::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating Contour...." << std::endl;
    
    new_PC_flag        = true;
    new_PC_1th_flag    = true;
    new_PC_read_flag   = false;
    new_PC_output_flag = false;
    PC_Read_flag       = false;
    InputCloud_times   = 0;
    viewer_closed = false;

    voxelSize = (float)m_GridSize;

    new_cloud = CloudT::Ptr(new CloudT);
    out_cloud = ColorCloudT::Ptr(new ColorCloudT);
    Empty_cloud = CloudT::Ptr(new CloudT);
    std::cerr << "[INFO] Activated Contour OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t Contour::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Deactivating Contour...." << std::endl;
    PC_Read_flag = false;
    std::cerr << "[INFO] Deactivated Contour OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t Contour::onExecute(RTC::UniqueId /*ec_id*/)
{
    // 点群の読み込み /////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    if (new_PC_flag) {
        CloudT::Ptr cloud(new CloudT);
        // m_PCDからデータを読み込む
        if (m_PCDIn.isNew()) {
            m_PCDIn.read();
            if (new_PC_1th_flag) {
                std::cerr << "[INFO] \"m_PCD\" received!" << std::endl;
                new_PC_1th_flag = false;
            }
            InputCloud_times++;
            std::cerr << "[INFO] " << InputCloud_times << " \"m_PCD\" is being read..." << std::endl;

            // RTCPCLからPCLに型変換
            cloud->is_dense = m_PCD.is_dense;
            cloud->points.resize(m_PCD.width * m_PCD.height);
            float* src = (float*)m_PCD.data.get_buffer();
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

        if (new_PC_read_flag) {
            // 最新データ受信時間の更新
            startTime = currentTime;
        }
        auto timeDifference = currentTime - startTime;

        // 一定時間以内に次の点群が受信された場合、new_cloudに追加
        if (timeDifference <= maxTimeDifference) {
            *new_cloud += *cloud;
            new_PC_read_flag   = false;
            new_PC_output_flag = true;
        }
        else if (new_PC_output_flag) {
            // 指定した時間を超えたらループを終了
            new_PC_flag        = false;
            new_PC_output_flag = false;
            PC_Read_flag       = true;
        }
        cloud.reset();

        
        if (PC_Read_flag) {
            // 処理の追加 /////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            std::cout << "[INFO] Point cloud loading complete.\n[INFO] Number of points: " << new_cloud->points.size() << std::endl;
            std::cout << "[INFO] Start processing..." << std::endl;

            // ボクセルグリッドフィルタを適用
            CloudT::Ptr voxelizedCloud(new CloudT);
            pcl::VoxelGrid<PointT> voxelGrid;
            voxelGrid.setInputCloud(new_cloud);
            voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);

            // フィルタリング実行
            voxelGrid.filter(*voxelizedCloud);

            // ボクセルの中心点を格納する点群データ
            CloudT::Ptr voxelCenters(new CloudT);
            
            // 各ボクセルの中心点を計算
            for (const auto& point : voxelizedCloud->points) {
                PointT center;
                center.x = std::floor(point.x / voxelSize) * voxelSize + voxelSize / 2.0f;
                center.y = std::floor(point.y / voxelSize) * voxelSize + voxelSize / 2.0f;
                center.z = std::floor(point.z / voxelSize) * voxelSize + voxelSize / 2.0f;
                voxelCenters->points.push_back(center);
            }
            voxelCenters->width = voxelCenters->points.size();
            voxelCenters->height = 1;
            voxelCenters->is_dense = true;
            std::cout << "[INFO] Voxelized point cloud has " << voxelCenters->size() << " points." << std::endl;

            // xy座標が同じ点の中でz値が最小のものだけを残す処理
            std::map<std::pair<int, int>, PointT> minZMap;

            for (const auto& point : voxelCenters->points) {
                // 各点のxyボクセルキーを計算
                int xKey = static_cast<int>(std::floor(point.x / voxelSize));
                int yKey = static_cast<int>(std::floor(point.y / voxelSize));
                std::pair<int, int> xyKey = { xKey, yKey };

                // z値が小さい場合に更新
                if (minZMap.find(xyKey) == minZMap.end() || point.z < minZMap[xyKey].z) {
                    minZMap[xyKey] = point;
                }
            }

            // 最小z値を持つ点を結果の点群に追加
            for (const auto& entry : minZMap) {
                pcl::PointXYZRGB point;
                point.x = entry.second.x;
                point.y = entry.second.y;
                point.z = entry.second.z;
                point.r = 0;
                point.g = 0;
                point.b = 0;
                out_cloud->points.push_back(point);
            }

            // out_cloudのwidth, height, is_denseを設定
            out_cloud->width = out_cloud->points.size();
            out_cloud->height = 1; // 1行にすべてのポイントを格納
            out_cloud->is_dense = true;

            // z軸の最小値と最大値を取得
            float maxZ = std::numeric_limits<float>::lowest();
            float minZ = std::numeric_limits<float>::max();
            for (const auto& point : out_cloud->points) {
                if (point.z < minZ) minZ = point.z;
                if (point.z > maxZ) maxZ = point.z;
            }

            // カラー付き点群の作成
            for (size_t i = 0; i < out_cloud->points.size(); ++i) {
                // z軸に基づく疑似カラーを計算
                uint8_t r, g, b;
                calculateColor(out_cloud->points[i].z, minZ, maxZ, r, g, b);
                out_cloud->points[i].r = r;
                out_cloud->points[i].g = g;
                out_cloud->points[i].b = b;
            }

            // バグ回避のため
            test(voxelizedCloud, Empty_cloud);
            voxelizedCloud.reset();

            int Process_OK = 0;

            // 点群の出力 /////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////

            if (Process_OK == 0) {
                // 点群の配列数を取得
                size_t Contour_PC_size = out_cloud->width * out_cloud->height;
                
                // 点群を分割して送信
                
                std::cerr << "[INFO] \"Contour\" is divided..." << std::endl;
                int OutputCloud_times = 0;
                
                // 点群を分割して通信
                for (size_t j = 0; j < Contour_PC_size; j += MAX_CPC_COMM) {
                    size_t endIdx = std::min(j + MAX_CPC_COMM, Contour_PC_size);
                
                    // 部分点群の作成
                    PointCloudTypes::PointCloud subCloud;
                
                    // 点群データの有無の確認
                    subCloud.is_dense = (::CORBA::Boolean)out_cloud->is_dense;
                    // subCloudのサイズを設定
                    subCloud.width = endIdx - j;
                    subCloud.height = out_cloud->height;
                
                    // subCloudのデータを格納するためにバッファを確保
                    subCloud.data.length(sizeof(float) * (endIdx - j) * 4);
                
                    // バッファへのポインタを取得
                    float* dest = (float*)subCloud.data.get_buffer();
                
                    // out_cloudからsubCloudにデータをコピー
                    for (size_t i = j; i < endIdx; i++) {
                        dest[0] = out_cloud->points[i].x;
                        dest[1] = out_cloud->points[i].y;
                        dest[2] = out_cloud->points[i].z;
                        dest[3] = out_cloud->points[i].rgb;
                        dest += 4;
                    }
                
                    // OutPortデータへ代入
                    // 点群データの有無の確認
                    m_Contour.is_dense = subCloud.is_dense;
                    // subCloudのサイズを設定
                    m_Contour.width  = subCloud.width;
                    m_Contour.height = subCloud.height;
                    m_Contour.data   = subCloud.data;
                
                    // memory release for RTCPCL_Cloud
                    subCloud.data.release();
                
                    OutputCloud_times++;
                    std::cerr << "[INFO] " << OutputCloud_times << " \"Contour\" is output..." << std::endl;
                
                    // OutPortから出力する
                    while (!m_ContourOut.write()) {
                        RTC::DataPortStatusList stat = m_ContourOut.getStatusList();
                
                        for (size_t i(0), len(stat.size()); i < len; ++i) {
                            if (stat[i] != RTC::DataPortStatus::PORT_OK) {
                                std::cout << "[ERRO] Error in connector number " << i << " with status: " << RTC::toString(stat[i]) << std::endl;
                                Sleep(0.5 * 1000);
                            }
                        }
                        //Sleep(0.5 * 1000);
                        //break;
                    }
                    Sleep(0.5 * 1000);
                }
                new_cloud.reset();
                out_cloud.reset();
                minZMap.clear();
                new_cloud = CloudT::Ptr(new CloudT);
                out_cloud = ColorCloudT::Ptr(new ColorCloudT);
                Process_OK = 1;
                new_PC_flag        = true;
                new_PC_1th_flag    = true;
                new_PC_read_flag   = false;
                new_PC_output_flag = false;
                PC_Read_flag       = false;
                InputCloud_times   = 0;
            }
        }
    }
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t Contour::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t Contour::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Contour::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t Contour::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Contour::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void ContourInit(RTC::Manager* manager)
  {
    coil::Properties profile(contour_spec);
    manager->registerFactory(profile,
                             RTC::Create<Contour>,
                             RTC::Delete<Contour>);
  }
  
}
