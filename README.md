<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Mapping-by-3D-CaveSurvey</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            line-height: 1.8;
        }
        h1, h2, h3 {
            color: #333;
        }
        p {
            margin: 10px 0;
        }
        .image-wrapper {
            text-align: center;
            margin-bottom: 20px;
        }
        .image-wrapper img {
            margin: 10px;
        }
        .section {
            margin-bottom: 30px;
        }
    </style>
</head>
<body>

    <h1>Mapping-by-3D-CaveSurvey</h1>
    <p>Development of RT component for drawing system using 3D point clouds of caves.<br>
    Please check <a href="https://openrtm.org/openrtm/ja/project/contest2024-si2024-0109">this page</a>.</p>

    <div class="image-wrapper">
        <img src="https://openrtm.org/openrtm/sites/default/files/PointCloud_Viewer3.jpg" alt="PointCloud Viewer" width="600">
        <img src="https://openrtm.org/openrtm/sites/default/files/FPS.jpg" alt="FPS" width="300">
        <img src="https://openrtm.org/openrtm/sites/default/files/EMap_PlanWall_0.jpg" alt="EMap_PlanWall" width="600">
    </div>

    <div class="section">
        <h2>目的</h2>
        <p>洞窟での測量は，地理学や地質学的な観点から重要なタスクの一つであり，センサ技術を利用したデジタル化が望まれている一分野である．特に，3 次元距離計測は洞窟内の地図構築や洞窟の形状把握が可能となるため，小型化された計測システムと解析技術の発展が望まれている．そこで本研究では，昨年作成した測量システムを拡張し，洞窟で計測された3次元点群データをもとに図面化するシステムをRTコンポーネント（RTC）により開発することで，容易に洞窟の図面化が可能となるシステムの有効性を示すコンポーネント群である．</p>
    </div>

    <div class="section">
        <h2>概要</h2>
        <ul>
            <li>LiDARセンサーからのデータを出力</li>
            <li>3次計測を行い点群データを生成</li>
            <li>複数の点群データを統合</li>
            <li>統合された点群データでGrowing Neural Gas(GNG)を用いた解析</li>
            <li>各種点群データの表示</li>
            <li>統合された点群データをFarthest Point Sampling (FPS)でダウンサンプリング</li>
            <li>洞壁のクラスタを生成</li>
            <li>エレベーションマップの生成</li>
            <li>洞壁のクラスタとエレベーションマップの統合と可視化</li>
        </ul>
    </div>

    <div class="section">
        <h2>特徴</h2>
        <ul>
            <li>各計測地点における点群データを保存</li>
            <li>複数の点群データを統合</li>
            <li>統合された点群データでGNGを用いた解析</li>
            <li>各種点群データの表示</li>
            <li>FPSでダウンサンプリング</li>
            <li>洞壁のクラスタを生成</li>
            <li>エレベーションマップの生成</li>
            <li>洞壁のクラスタとエレベーションマップの統合と可視化</li>
        </ul>
    </div>

    <div class="section">
        <h2>仕様</h2>
        <ul>
            <li>言語: C++</li>
            <li>OS: Windows 10</li>
        </ul>
    </div>

    <div class="section">
        <h2>コンポーネント群</h2>
        <h3>再利用</h3>
        <ul>
            <li>EtheURG：2DLiDARセンサーを読取り出力（北陽電機 URGセンサ　UTM-30LX-EW）</li>
            <li>MeasurementSystem：Dynamixelを用いてLiDARを回転させ，3D点群を生成し保存・出力を行う</li>
            <li>Registration：複数の場所で計測された点群データを一つの点群データとして統合する</li>
        </ul>
        <h3>修正</h3>
        <ul>
            <li>Analyses：統合された点群データでGNGを用いた解析が行われ，その解析結果を出力する</li>
            <li>PointCloud_Viewer：MeasurementSystem，Registration，Analysesから受け取った点群データを表示する</li>
        </ul>
        <h3>新規作成</h3>
        <ul>
            <li>FPS RTC：FPSを行うことで点群の粗密に関係なく均等にダウンサンプリング</li>
            <li>WallDTC RTC：鉛直(z軸)方向上から見た洞壁をクラスタとして生成・出力する</li>
            <li>Contour RTC：洞床の高低差を疑似カラーを用いたエレベーションマップで表現</li>
            <li>MapViewer RTC：洞壁のクラスタとエレベーションマップを統合して可視化する</li>
        </ul>
        <h3>オプションRTC</h3>
        <ul>
            <li>PointCloud_Reader：.ply形式の点群データを読み込み，File_PointCloudポートから出力する</li>
        </ul>
    </div>

    <div class="section">
        <h2>ドキュメント</h2>
        <ul>
            <li><a href="https://github.com/yukimeat1999/Mapping-by-3D-CaveSurvey/blob/main/%E6%B4%9E%E7%AA%9F%E3%81%AE3%E6%AC%A1%E5%85%83%E7%82%B9%E7%BE%A4%E3%82%92%E7%94%A8%E3%81%84%E3%81%9F%E5%9B%B3%E9%9D%A2%E5%8C%96%E3%82%B7%E3%82%B9%E3%83%86%E3%83%A0%E3%81%AERT%E3%82%B3%E3%83%B3%E3%83%9D%E3%83%BC%E3%83%8D%E3%83%B3%E3%83%88%E9%96%8B%E7%99%BA.pdf">マニュアル</a></li>
        </ul>
    </div>

</body>
</html>
