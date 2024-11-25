// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  WallDTC.cpp
 * @brief ModuleDescription
 *
 */
// </rtc-template>

#include "WallDTC.h"

#define MAX_PC_COMM 170000

// Time
std::chrono::system_clock::time_point startTime;
std::chrono::system_clock::duration maxTimeDifference = std::chrono::seconds(3);

// GNG
struct gng* gng_net;

bool new_PC_flag;
bool new_PC_1th_flag;
bool new_PC_read_flag;
bool new_PC_output_flag;
bool PC_Read_flag;
int  InputCloud_times;

bool Read_flag;
bool View_flag;
bool Save_flag;
bool viewer_closed;

float max_edge_length;
float max_conn_dis;
float alpha;

// Point Cloud
CloudT::Ptr new_cloud;
ColorCloudT::Ptr out_cloud;
CloudT::Ptr Empty_cloud;


// Module specification
// <rtc-template block="module_spec">
static const char* const walldtc_spec[] =
  {
    "implementation_id", "WallDTC",
    "type_name",         "WallDTC",
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
    "conf.default.MaxEdgeLength", "5",
    "conf.default.MaxConnDis", "10",
    "conf.default.Alpha", "0.5",

    // Widget
    "conf.__widget__.MaxEdgeLength", "text",
    "conf.__widget__.MaxConnDis", "text",
    "conf.__widget__.Alpha", "text",
    // Constraints

    "conf.__type__.MaxEdgeLength", "float",
    "conf.__type__.MaxConnDis", "float",
    "conf.__type__.Alpha", "float",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
WallDTC::WallDTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PCDIn("PCD", m_PCD),
    m_PlanWallOut("PlanWall", m_PlanWall)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
WallDTC::~WallDTC()
{
}

void test(const CloudT::Ptr& input_cloud, CloudT::Ptr& output_cloud) {
    CloudT::Ptr cloud_test;
    cloud_test = CloudT::Ptr(new CloudT);
    cloud_test = input_cloud;
    output_cloud = cloud_test;
    cloud_test.reset();
}

void WallDTC::init_m_gng(ClusterTypes::ClusterData& m_PlanWall)
{
    // ポートの初期化
    m_PlanWall.node.length(GNGN);     // ノード数の設定
    m_PlanWall.edge.length(GNGN);     // エッジ配列の設定
	m_PlanWall.node_n = 0;

    // 各ノードの次元ごとにメモリを確保
    for (int i = 0; i < GNGN; i++) {
        m_PlanWall.node[i].length(DIM);  // ノードの次元
        m_PlanWall.edge[i].length(GNGN); // エッジの次元
    }
    return;
}

void WallDTC::free_m_gng(ClusterTypes::ClusterData& m_PlanWall)
{
    // 各ノードの配列を解放
    for (int i = 0; i < GNGN; i++) {
        m_PlanWall.node[i].length(0);  // ノード配列の解放
        m_PlanWall.edge[i].length(0);  // エッジ配列の解放
    }

    // メイン配列の解放
    m_PlanWall.node.length(0);       // ノード配列全体の解放
    m_PlanWall.edge.length(0);       // エッジ配列全体の解放
	m_PlanWall.node_n = 0;

    // メモリ解放が完了
    return;
}

RTC::ReturnCode_t WallDTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("PCD", m_PCDIn);
  
  // Set OutPort buffer
  addOutPort("PlanWall", m_PlanWallOut);

  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("MaxEdgeLength", m_MaxEdgeLength, "5");
  bindParameter("MaxConnDis", m_MaxConnDis, "10");
  bindParameter("Alpha", m_Alpha, "0.5");
  // </rtc-template>
  init_m_gng(m_PlanWall);
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t WallDTC::onFinalize()
{
    free_m_gng(m_PlanWall);
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t WallDTC::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t WallDTC::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t WallDTC::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating WallDTC...." << std::endl;
    new_PC_flag        = true;
    new_PC_1th_flag    = true;
    new_PC_read_flag   = false;
    new_PC_output_flag = false;
    PC_Read_flag       = false;
    InputCloud_times   = 0;

    Read_flag = false;
    View_flag = false;
    Save_flag = false;
    viewer_closed = false;

    if (gng_net == NULL) {
        gng_net = init_gng();
    }

    max_edge_length = m_MaxEdgeLength;
    max_conn_dis = m_MaxConnDis;
    alpha = m_Alpha;

    new_cloud = CloudT::Ptr(new CloudT);
    out_cloud = ColorCloudT::Ptr(new ColorCloudT);
    Empty_cloud = CloudT::Ptr(new CloudT);
    std::cerr << "[INFO] Activated WallDTC OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t WallDTC::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Deactivating WallDTC...." << std::endl;
    Read_flag = false;
    View_flag = false;
    Save_flag = false;
    PC_Read_flag = false;
    free_gng(gng_net);
    std::cerr << "[INFO] Deactivated WallDTC OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t WallDTC::onExecute(RTC::UniqueId /*ec_id*/)
{
    // 点群の読み込み //////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    if (new_PC_flag) {
        CloudT::Ptr cloud(new CloudT);
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
            // 処理の追加 /////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            int Process_OK = -1;

            // 点群のz値を0にして平面に圧縮
            for (auto& point : new_cloud->points) {
                point.z = 0.0f;
            }
            std::cerr << "[INFO] The PointCloud is compressed into a plane." << std::endl;

            // 凹部も含めた外枠を捉えるため、Concave Hull（Alpha Shapes）を使用
            CloudT::Ptr concave_hull(new CloudT);
            pcl::ConcaveHull<pcl::PointXYZ> hull;
            hull.setInputCloud(new_cloud);   // 圧縮された点群を入力
            hull.setAlpha(alpha);            // Alpha値を設定して、凹部を表現する精度を調整
            hull.reconstruct(*concave_hull); // 凹部を含む外枠を計算し、結果をconcave_hullに格納

            std::cerr << "[INFO] Concave Hull points: " << concave_hull->points.size() << std::endl;

            // ノードを順番に接続してエッジを作成
            for (size_t i = 0; i < concave_hull->points.size(); ++i) {
                // 次のノードのインデックス（最後のノードの場合は最初のノードを指す）
                int next_idx = (i + 1) % concave_hull->points.size();

                // ノード間の距離を計算
                float distance = sqrt(
                    pow(concave_hull->points[i].x - concave_hull->points[next_idx].x, 2) +
                    pow(concave_hull->points[i].y - concave_hull->points[next_idx].y, 2) +
                    pow(concave_hull->points[i].z - concave_hull->points[next_idx].z, 2)
                );

                // 距離が閾値以下の場合のみ接続
                if (distance <= max_edge_length && i != next_idx) {
                    // ノード i と次のノード next_idx を接続
                    gng_net->edge[i][next_idx] = 1;
                    gng_net->edge[next_idx][i] = 1;
                    gng_net->edge_ct[i]++;
                    gng_net->edge_ct[next_idx]++;
                }

                // ノードを追加 (座標情報を gng_net に格納)
                gng_net->node[i][0] = concave_hull->points[i].x;
                gng_net->node[i][1] = concave_hull->points[i].y;
                gng_net->node[i][2] = concave_hull->points[i].z;
                gng_net->node[i][3] = 0;  // 色情報 (R) 1
                gng_net->node[i][4] = 0;  // 色情報 (G)
                gng_net->node[i][5] = 0;  // 色情報 (B)
            }
            // ノード数を更新
            gng_net->node_n = concave_hull->points.size();

            CloudT::Ptr disconnected(new CloudT);
            std::vector<int> disconnectedIdx;
            for (size_t i = 0; i < concave_hull->points.size(); ++i) {
                // 枠が途切れている箇所を探索
                if (gng_net->edge_ct[i] == 1) {
                    disconnected->push_back(concave_hull->points[i]);
					disconnectedIdx.push_back(i);
                    gng_net->node[i][3] = 0;  // 色情報 (R)
                    gng_net->node[i][4] = 0;  // 色情報 (G) 1
                    gng_net->node[i][5] = 0;  // 色情報 (B)
                }
            }

            for (size_t i = 0; i < disconnected->points.size(); i++) {
                if (gng_net->edge_ct[disconnectedIdx[i]] == 1) {
					std::vector<float> distances(disconnected->points.size());
                    for (size_t j = 0; j < disconnected->points.size(); ++j) {
                        if (i == j) {
                            // i == j のときにfloat型の最大値を代入
                            distances[j] = std::numeric_limits<float>::max();
                        }
                        else {
                            distances[j] = sqrt(
                                pow(disconnected->points[i].x - disconnected->points[j].x, 2) +
                                pow(disconnected->points[i].y - disconnected->points[j].y, 2) +
                                pow(disconnected->points[i].z - disconnected->points[j].z, 2)
                            );
                        }
                    }
                    
                    // iより大きいインデックスの距離を検索
                    auto min_it = std::min_element(distances.begin() + i, distances.end());
                    int min_index = std::distance(distances.begin(), min_it);
                    float min_value = *min_it;

                    // 接続先のエッジが一本
                    if (gng_net->edge_ct[disconnectedIdx[min_index]] == 1) {
                        // 最小の距離がしきい値未満の場合のみ接続を行う
                        if (min_value < max_conn_dis && min_index > i) {
					        // 未接続箇所を接続
					        gng_net->edge[disconnectedIdx[i]][disconnectedIdx[min_index]] = 1;
					        gng_net->edge[disconnectedIdx[min_index]][disconnectedIdx[i]] = 1;
					        gng_net->edge_ct[disconnectedIdx[i]]++;
					        gng_net->edge_ct[disconnectedIdx[min_index]]++;
                        }
                    } else if (gng_net->edge_ct[disconnectedIdx[min_index]] > 1) {
						// 接続先のエッジが複数本の場合
                        // 最小値を一時的に最大値に設定する
                        *min_it = std::numeric_limits<float>::max();  // 最大値に置き換える

                        // 二番目に小さい要素を見つける
                        auto second_min_it = std::min_element(distances.begin() + i, distances.end());
                        int second_min_index = std::distance(distances.begin(), second_min_it);
                        float second_min_value = *second_min_it;
                        
                        // 最小の距離がしきい値未満の場合のみ接続を行う
                        if (second_min_value < max_conn_dis && second_min_index > i) {
                            // 未接続箇所を接続
                            gng_net->edge[disconnectedIdx[i]][disconnectedIdx[second_min_index]] = 1;
                            gng_net->edge[disconnectedIdx[second_min_index]][disconnectedIdx[i]] = 1;
                            gng_net->edge_ct[disconnectedIdx[i]]++;
                            gng_net->edge_ct[disconnectedIdx[second_min_index]]++;
                        }
                    }
                }
            }

            // バグ回避のため
            test(concave_hull, Empty_cloud);
            concave_hull.reset();

            Process_OK = 0;

            // 点群の出力 /////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////

            if (Process_OK == 0) {
				// GNGをPlanWallに格納
                m_PlanWall.node_n = gng_net->node_n;
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < DIM; j++) {
                        m_PlanWall.node[i][j] = gng_net->node[i][j];
                    }
                }
                for (int i = 0; i < GNGN; i++) {
                    for (int j = 0; j < GNGN; j++) {
						m_PlanWall.edge[i][j]  = gng_net->edge[i][j];
                    }
                }

                // OutPortから出力する
                while (!m_PlanWallOut.write()) {
                    RTC::DataPortStatusList stat = m_PlanWallOut.getStatusList();
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
				new_cloud.reset();
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


//RTC::ReturnCode_t WallDTC::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t WallDTC::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t WallDTC::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t WallDTC::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t WallDTC::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void WallDTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(walldtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<WallDTC>,
                             RTC::Delete<WallDTC>);
  }
  
}
