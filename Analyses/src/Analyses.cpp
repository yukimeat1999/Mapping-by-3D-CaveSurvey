// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  Analyses.cpp
 * @brief Analyses comp
 *
 */
// </rtc-template>

#include "Analyses.h"
#include "gng.h"

#define MAX_PC_COMM 170000

// PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr merge_PointCloud_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr analyses_PointCloud_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_analyses_PointCloud_;

// Time
std::chrono::system_clock::time_point startTime;
std::chrono::system_clock::duration maxTimeDifference = std::chrono::seconds(3);

bool merge_PC_flag;
bool merge_PC_1th_flag;
bool merge_PC_read_flag;
bool merge_PC_output_flag;
bool PC_Read_flag;
int  InputCloud_times;

struct gng* gng_net;
double **pointcloud_data;

int epoch;
int MaxEpoch;
int epoch_times;
int edges_num;

// Module specification
// <rtc-template block="module_spec">
static const char* const analyses_spec[] =
  {
    "implementation_id", "Analyses",
    "type_name",         "Analyses",
    "description",       "Analyses comp",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Analyses",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.epoch", "1000",
    "conf.default.MaxEpoch", "10000",

    // Widget
    "conf.__widget__.epoch", "text",
    "conf.__widget__.MaxEpoch", "text",
    // Constraints

    "conf.__type__.epoch", "int",
    "conf.__type__.MaxEpoch", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Analyses::Analyses(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_merge_PointCloudIn("merge_PointCloud", m_merge_PointCloud),
    m_analyses_ClusterOut("analyses_Cluster", m_analyses_Cluster)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Analyses::~Analyses()
{
}

void Analyses::init_m_gng(ClusterTypes::ClusterData& m_analyses_Cluster)
{
    // ポートの初期化
    m_analyses_Cluster.node.length(GNGN);     // ノード数の設定
    m_analyses_Cluster.edge.length(GNGN);     // エッジ配列の設定
    m_analyses_Cluster.node_n = 0;

    // 各ノードの次元ごとにメモリを確保
    for (int i = 0; i < GNGN; i++) {
        m_analyses_Cluster.node[i].length(DIM);  // ノードの次元
        m_analyses_Cluster.edge[i].length(GNGN); // エッジの次元
    }
    return;
}

void Analyses::free_m_gng(ClusterTypes::ClusterData& m_analyses_Cluster)
{
    // 各ノードの配列を解放
    for (int i = 0; i < GNGN; i++) {
        m_analyses_Cluster.node[i].length(0);  // ノード配列の解放
        m_analyses_Cluster.edge[i].length(0);  // エッジ配列の解放
    }

    // メイン配列の解放
    m_analyses_Cluster.node.length(0);       // ノード配列全体の解放
    m_analyses_Cluster.edge.length(0);       // エッジ配列全体の解放
    m_analyses_Cluster.node_n = 0;

    // メモリ解放が完了
    return;
}

RTC::ReturnCode_t Analyses::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("merge_PointCloud", m_merge_PointCloudIn);
  
  // Set OutPort buffer
  addOutPort("analyses_Cluster", m_analyses_ClusterOut);

  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("epoch", m_epoch, "1000");
  bindParameter("MaxEpoch", m_MaxEpoch, "10000");
  // </rtc-template>
  init_m_gng(m_analyses_Cluster);
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Analyses::onFinalize()
{
    free_m_gng(m_analyses_Cluster);
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t Analyses::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Analyses::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t Analyses::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating Analyses...." << std::endl;
    
    merge_PC_flag        = true;
    merge_PC_1th_flag    = true;
    merge_PC_read_flag   = false;
    merge_PC_output_flag = false;
    PC_Read_flag         = false;
    InputCloud_times     = 0;

    epoch       = m_epoch;
    MaxEpoch    = m_MaxEpoch;
    epoch_times = 0;
    edges_num   = 0;

    merge_PointCloud_    = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (gng_net == NULL) {
        gng_net = init_gng();
        gng_net->weight[0] = 1;//x
        gng_net->weight[1] = 1;//y
        gng_net->weight[2] = 1;//z
        gng_net->weight[3] = 0;//r
        gng_net->weight[4] = 0;//g
        gng_net->weight[5] = 0;//b
    }

    if(pointcloud_data == NULL){
        pointcloud_data = malloc2d_double(1000000, LDIM);
    }

    std::cerr << "[INFO] Activated Analyses OK!" << std::endl;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t Analyses::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Deactivating Analyses...." << std::endl;
    
    PC_Read_flag = false;
    merge_PointCloud_.reset();

    std::cerr << "[INFO] Deactivated Analyses OK!" << std::endl;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t Analyses::onExecute(RTC::UniqueId /*ec_id*/)
{
    // 点群の読み込み ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    if (merge_PC_flag) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // merge_PointCloudからデータを読み込む
        if (m_merge_PointCloudIn.isNew()) {
            m_merge_PointCloudIn.read();
            if (merge_PC_1th_flag) {
                std::cerr << "[INFO] \"merge_PointCloud\" received!" << std::endl;
                merge_PC_1th_flag = false;
            }
            InputCloud_times++;
            std::cerr << "[INFO] " << InputCloud_times << " \"merge_PointCloud\" is being read..." << std::endl;

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
            startTime = currentTime;
        }
        auto timeDifference = currentTime - startTime;

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
            PC_Read_flag         = true;
        }
        cloud.reset();

        if (PC_Read_flag) {
            // 処理の追加 //////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
           
            int datasize = 0;
            for (size_t i = 0; i < merge_PointCloud_->points.size(); i++) {
                if (merge_PointCloud_->points[i].x == 0
                    && merge_PointCloud_->points[i].y == 0
                    && merge_PointCloud_->points[i].z == 0) {
                    continue;
                }

                pointcloud_data[datasize][0] = merge_PointCloud_->points[i].x;
                pointcloud_data[datasize][1] = merge_PointCloud_->points[i].y;
                pointcloud_data[datasize++][2] = merge_PointCloud_->points[i].z;
            }
            for (int i = 0; i < gng_net->node_n; i++) {
                gng_net->edge_ct[i] = 0;
                for (int j = 0; j < gng_net->node_n; j++) {
                    gng_net->edge[i][j] = 0;
                }
            }
            gng_net->edge[1][0] = 1;
            gng_net->edge[0][1] = 1;
            gng_net->edge_ct[0] = 1;
            gng_net->edge_ct[1] = 1;
            gng_net->node_n = 2;

            printf("# of data is %d\n", datasize);
            int maxnode = GNGN;
            if (datasize < GNGN) {
                maxnode = datasize / 2;
            }
			
            do {
                do {
                    for (int i = 0; i < epoch; i++) {
                        gng_main(gng_net, pointcloud_data, datasize);
                        epoch_times++;
                    }
                    if (epoch_times >= MaxEpoch) {
                        break;
                    }
                } while (gng_net->node_n < GNGN - 3);
                break;
            } while (epoch_times < MaxEpoch);

            filtered_analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            int a_currentsize = analyses_PointCloud_->size();
            analyses_PointCloud_->resize(a_currentsize + gng_net->node_n);
            for (int i = 0; i < gng_net->node_n; i++) {
                analyses_PointCloud_->points[a_currentsize + i].x = gng_net->node[i][0];
                analyses_PointCloud_->points[a_currentsize + i].y = gng_net->node[i][1];
                analyses_PointCloud_->points[a_currentsize + i].z = gng_net->node[i][2];
                analyses_PointCloud_->points[a_currentsize + i].r = 0;
                analyses_PointCloud_->points[a_currentsize + i].g = 0;
                analyses_PointCloud_->points[a_currentsize + i].b = 0;
                if (gng_net->node[i][7] != -10.0) {
                    double threshold = 0.1;
                    analyses_PointCloud_->points[a_currentsize + i].r = (int)(gng_net->node[i][7] / threshold * 255.0);

                    if (analyses_PointCloud_->points[a_currentsize + i].r > 100) {
                        pcl::PointXYZRGB point;
                        point.x = gng_net->node[i][0];
                        point.y = gng_net->node[i][1];
                        point.z = gng_net->node[i][2];
                        point.r = (int)(gng_net->node[i][7] / threshold * 255.0);
                        point.g = 0;
                        point.b = 0;
                        if (analyses_PointCloud_->points[a_currentsize + i].r > 255) {
                            analyses_PointCloud_->points[a_currentsize + i].r = 255;
                            point.r = 255;
                        }
                        filtered_analyses_PointCloud_->points.push_back(point);
                    }
                }
                else {
                    analyses_PointCloud_->points[a_currentsize + i].g = 255;
                }
            }
            filtered_analyses_PointCloud_->width = static_cast<uint32_t>(filtered_analyses_PointCloud_->points.size());
            filtered_analyses_PointCloud_->height = 1;
            filtered_analyses_PointCloud_->is_dense = true;

            pcl::io::savePLYFile("res.ply", *analyses_PointCloud_);
            analyses_PointCloud_.reset();
            analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

            int f_currentsize = filtered_analyses_PointCloud_->size();
            analyses_PointCloud_->resize(f_currentsize);
            for (int i = 0; i < f_currentsize; i++) {
                analyses_PointCloud_->points[i].x = filtered_analyses_PointCloud_->points[i].x;
                analyses_PointCloud_->points[i].y = filtered_analyses_PointCloud_->points[i].y;
                analyses_PointCloud_->points[i].z = filtered_analyses_PointCloud_->points[i].z;
            }

            // 出力する点群が無い場合，データポートから0を出力
            if (analyses_PointCloud_->size() == 0) {
                analyses_PointCloud_->resize(1);
                analyses_PointCloud_->points[0].x = NULL;
                analyses_PointCloud_->points[0].y = NULL;
                analyses_PointCloud_->points[0].z = NULL;
            }

            std::cerr << "[INFO] Current number of epochs: " << epoch_times << std::endl;
            std::cerr << "[INFO] Current number of Nodes: "  << gng_net->node_n + 1 << std::endl;
            std::cerr << "[INFO] Current number of Edges: "  << edges_num << std::endl;

            int Process_OK = 0; // 関数等からの戻り値によって点群の出力を開始

            // 点群の出力 //////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////

            if (Process_OK == 0) {
                // GNGをanalyses_Clusterに格納
                m_analyses_Cluster.node_n = gng_net->node_n;
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < DIM; j++) {
                        m_analyses_Cluster.node[i][j] = gng_net->node[i][j];
                    }
                }
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < GNGN; j++) {
                        m_analyses_Cluster.edge[i][j] = gng_net->edge[i][j];
                    }
                }

                // OutPortから出力する
                while (!m_analyses_ClusterOut.write()) {
                    RTC::DataPortStatusList stat = m_analyses_ClusterOut.getStatusList();
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

                Process_OK = 1;
                merge_PC_flag        = true;
                merge_PC_1th_flag    = true;
                merge_PC_read_flag   = false;
                merge_PC_output_flag = false;
                PC_Read_flag         = false;
                InputCloud_times = 0;
                merge_PointCloud_.reset();
                analyses_PointCloud_.reset();
                filtered_analyses_PointCloud_.reset();
                merge_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            }
        }
    }
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t Analyses::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t Analyses::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Analyses::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t Analyses::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Analyses::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void AnalysesInit(RTC::Manager* manager)
  {
    coil::Properties profile(analyses_spec);
    manager->registerFactory(profile,
                             RTC::Create<Analyses>,
                             RTC::Delete<Analyses>);
  }
  
}
