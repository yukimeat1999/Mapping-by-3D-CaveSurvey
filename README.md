# Mapping-by-3D-CaveSurvey
Development of RT component for drawing system using 3D point clouds of caves.<br>
Please check [this page](https://openrtm.org/openrtm/ja/project/contest2024-si2024-0109).<br>
<br>
## 目的
　洞窟での測量は，地理学や地質学的な観点から重要なタスクの一つであり，センサ技術を利用したデジタル化が望まれている一分野である．特に，3 次元距離計測は洞窟内の地図構築や洞窟の形状把握が可能となるため，小型化された計測システムと解析技術の発展が望まれている．そこで本研究では，昨年作成した測量システムを拡張し，洞窟で計測された3次元点群データをもとに図面化するシステムをRTコンポーネント（RTC）により開発することで，容易に洞窟の図面化が可能となるシステムの有効性を示すコンポーネント群である．<br>
<br>
## 概要
・ LiDARセンサーからのデータを出力<br>
・ 3次計測を行い点群データを生成<br>
・ 複数の点群データを統合<br>
・ 統合された点群データでGrowing Neural Gas(GNG)を用いた解析<br>
・ 各種点群データの表示<br>
・ 統合された点群データをFarthest Point Sampling (FPS)でダウンサンプリング<br>
・ 洞壁のクラスタを生成<br>
・ エレベーションマップの生成<br>
・ 洞壁のクラスタとエレベーションマップの統合と可視化<br>
<br>
## 特徴
・ 各計測地点における点群データを保存<br>
・ 複数の点群データを統合<br>
・ 統合された点群データでGNGを用いた解析<br>
・ 各種点群データの表示<br>
・ FPSでダウンサンプリング<br>
・ 洞壁のクラスタを生成<br>
・ エレベーションマップの生成<br>
・ 洞壁のクラスタとエレベーションマップの統合と可視化<br>
<br>
## 仕様
・ 言語: C++<br>
・ OS: Windows 10<br>
<br>
## コンポーネント群
### 再利用
・ EtheURG：2DLiDARセンサーを読取り出力（北陽電機 URGセンサ　UTM-30LX-EW）<br>
・ MeasurementSystem：Dynamixelを用いてLiDARを回転させ，3D点群を生成し保存・出力を行う<br>
・ Registration：複数の場所で計測された点群データを一つの点群データとして統合する<br>
<br>
### 修正
・ Analyses：統合された点群データでGNGを用いた解析が行われ，その解析結果を出力する<br>
・ PointCloud_Viewer：MeasurementSystem，Registration，Analysesから受け取った点群データを表示する<br>
<br>
### 新規作成
・ FPS RTC：FPSを行うことで点群の粗密に関係なく均等にダウンサンプリング<br>
・ WallDTC RTC：鉛直(z軸)方向上から見た洞壁をクラスタとして生成・出力する<br>
・ Contour RTC：洞床の高低差を疑似カラーを用いたエレベーションマップで表現<br>
・ MapViewer RTC：洞壁のクラスタとエレベーションマップを統合して可視化する<br>
<br>
### オプションRTC
・ PointCloud_Reader：.ply形式の点群データを読み込み，File_PointCloudポートから出力する<br>
<br>
## ドキュメント
・ [マニュアル]()<br>
