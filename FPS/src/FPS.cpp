// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  FPS.cpp
 * @brief ModuleDescription
 *
 */
// </rtc-template>

#include "FPS.h"

#define MAX_PC_COMM 170000
bool Read_flag = false;

// Time
std::chrono::system_clock::time_point startTime;
std::chrono::system_clock::duration maxTimeDifference = std::chrono::seconds(3);

bool new_PC_flag;
bool new_PC_1th_flag;
bool new_PC_read_flag;
bool new_PC_output_flag;
bool PC_Read_flag;
int  InputCloud_times;
int  FPS_Max = 10000;

// Point Cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud;

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const fps_spec[] =
#else
static const char* fps_spec[] =
#endif
  {
    "implementation_id", "FPS",
    "type_name",         "FPS",
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
FPS::FPS(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PCDIn("PCD", m_PCD),
    m_DownPCDOut("DownPCD", m_DownPCD)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
FPS::~FPS()
{
}

// 進捗表示の追加
void Progress(int i, int all) {
    int progress = i * 100 / all;
	if (i == all){
		fprintf(stderr, "\r100%%\n");
		fflush(stderr);
    } else {
        fprintf(stderr, "\r%3d%%", progress);
        fflush(stderr); // 出力のフラッシュ
    }

}

// FarthestPointSampling関数
int FarthestPointSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    std::cerr << "[INFO] Downsampling has started..." << std::endl;
    int size = input_cloud->points.size();

    if (size == 0) {
        std::cerr << "[ERRO]: SampledPoints has no points." << std::endl;
        size = 1;
        return -1;
    }

    if (FPS_Max >= size) {
        std::cerr << "[INFO] Number of samples requested is greater than or equal to the number of points in the input cloud. \nReturning the original cloud." << std::endl;
        out_cloud = input_cloud;
        return 0;
    }

    std::vector<float> min_distances(size, std::numeric_limits<float>::max()); // float型の最大値で埋める
    std::vector<int> selected_indices;
    selected_indices.reserve(FPS_Max);

    // ランダムに最初の点を選択
    int first_index = random(0, size - 1);
    selected_indices.push_back(first_index);
    out_cloud->points.push_back(input_cloud->points[first_index]);

    for (int i = 1; i < FPS_Max; ++i) {
        int farthest_index = -1;
        float max_dist = -1;

        for (int j = 0; j < size; ++j) {
            float dist = pcl::euclideanDistance(input_cloud->points[selected_indices.back()], input_cloud->points[j]);
            min_distances[j] = std::min(min_distances[j], dist);
            // 距離が最大となる点を選択
            if (min_distances[j] > max_dist) {
                max_dist = min_distances[j];
                farthest_index = j;
            }
        }

        selected_indices.push_back(farthest_index);
        out_cloud->points.push_back(input_cloud->points[farthest_index]);

		Progress(i, FPS_Max);
    }
    Progress(FPS_Max, FPS_Max);

    out_cloud->width = FPS_Max;
    out_cloud->height = 1;
    out_cloud->is_dense = input_cloud->is_dense;
    std::cerr << "[INFO] Downsampling has been completed..." << std::endl;
    return 0;
}

RTC::ReturnCode_t FPS::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("PCD", m_PCDIn);
  
  // Set OutPort buffer
  addOutPort("DownPCD", m_DownPCDOut);

  
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
RTC::ReturnCode_t FPS::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t FPS::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t FPS::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t FPS::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating PointCloud_Reader...." << std::endl;
    new_PC_flag        = true;
    new_PC_1th_flag    = true;
    new_PC_read_flag   = false;
    new_PC_output_flag = false;
    PC_Read_flag       = false;
    InputCloud_times   = 0;
    FPS_Max            = m_FPS_Max;

    new_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    std::cerr << "[INFO] Activated PointCloud_Reader OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t FPS::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Deactivating PointCloud_Reader...." << std::endl;
    PC_Read_flag = false;
    std::cerr << "[INFO] Deactivated PointCloud_Reader OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t FPS::onExecute(RTC::UniqueId /*ec_id*/)
{
    // 点群の読み込み //////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    if (new_PC_flag) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // new_cloudからデータを読み込む
        if (m_PCDIn.isNew()) {
            m_PCDIn.read();
            if (new_PC_1th_flag) {
                std::cerr << "[INFO] \"new_cloud\" received!" << std::endl;
                new_PC_1th_flag = false;
            }
            InputCloud_times++;
            std::cerr << "[INFO] " << InputCloud_times << " \"new_cloud\" is being read..." << std::endl;

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
            // 処理の追加 //////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////

            // FPSでダウンサンプリング
            int Process_OK = FarthestPointSampling(new_cloud);
            new_cloud.reset();

            // 点群の出力 //////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////

            if (Process_OK == 0) {
                // 点群の配列数を取得
                size_t out_PC_size = out_cloud->width * out_cloud->height;

                // 点群を分割して送信

                std::cerr << "[INFO] \"out_cloud\" is divided..." << std::endl;
                int OutputCloud_times = 0;

                // 点群を分割して通信
                for (size_t j = 0; j < out_PC_size; j += MAX_PC_COMM) {
                    size_t endIdx = std::min(j + MAX_PC_COMM, out_PC_size);

                    // 部分点群の作成
                    PointCloudTypes::PointCloud subCloud;

                    // 点群データの有無の確認
                    subCloud.is_dense = (::CORBA::Boolean)out_cloud->is_dense;
                    // subCloudのサイズを設定
                    subCloud.width = endIdx - j;
                    subCloud.height = out_cloud->height;

                    // subCloudのデータを格納するためにバッファを確保
                    subCloud.data.length(sizeof(float) * (endIdx - j) * 3);

                    // バッファへのポインタを取得
                    float* dest = (float*)subCloud.data.get_buffer();

                    // out_cloudからsubCloudにデータをコピー
                    for (size_t i = j; i < endIdx; i++) {
                        dest[0] = out_cloud->points[i].x;
                        dest[1] = out_cloud->points[i].y;
                        dest[2] = out_cloud->points[i].z;
                        dest += 3;
                    }

                    // OutPortデータへ代入
                    // 点群データの有無の確認
                    m_DownPCD.is_dense = subCloud.is_dense;
                    // subCloudのサイズを設定
                    m_DownPCD.width = subCloud.width;
                    m_DownPCD.height = subCloud.height;
                    m_DownPCD.data = subCloud.data;

                    // memory release for RTCPCL_Cloud
                    subCloud.data.release();

                    OutputCloud_times++;
                    std::cerr << "[INFO] " << OutputCloud_times << " \"out_cloud\" is output..." << std::endl;

                    // OutPortから出力する
                    while (!m_DownPCDOut.write()) {
                        RTC::DataPortStatusList stat = m_DownPCDOut.getStatusList();
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
                out_cloud.reset();

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


//RTC::ReturnCode_t FPS::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t FPS::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t FPS::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t FPS::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t FPS::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void FPSInit(RTC::Manager* manager)
  {
    coil::Properties profile(fps_spec);
    manager->registerFactory(profile,
                             RTC::Create<FPS>,
                             RTC::Delete<FPS>);
  }
  
}
