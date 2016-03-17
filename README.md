# SLAM with Camera and IMU for Python
画像センサとIMUを用いたSLAMのためのPythonプログラム（日本語説明は後半）

>SLAM = Simultaneous Localization and Mapping

## Overview

[![](http://img.youtube.com/vi/IDZ5fxp_XdY/0.jpg)](https://www.youtube.com/watch?v=IDZ5fxp_XdY)

![](https://github.com/knagara/miscellaneous/blob/master/Overview.png)

- Android App (See -> [SLAMwithCameraIMUforAndroid](https://github.com/knagara/SLAMwithCameraIMUforAndroid))
- SLAM program (This page)
- MQTT Broker (See -> [MQTT](http://mqtt.org/), [mosquitto](http://mosquitto.org/), [Apollo](https://activemq.apache.org/apollo/))

## How to use
1.Setup Android App (See -> [SLAMwithCameraIMUforAndroid](https://github.com/knagara/SLAMwithCameraIMUforAndroid))

2.Setup MQTT Broker (See -> [MQTT](http://mqtt.org/), [mosquitto](http://mosquitto.org/), [Apollo](https://activemq.apache.org/apollo/))

3.Install MQTT package of Python
~~~
>pip install paho-mqtt
~~~
4.Run Main.py 
~~~
>python Main.py
~~~
5.Start Android App

6.Now the program is receiving sensor data and estimate smartphone location, also publishing it to MQTT broker.

7.If you want to save estimated data as CSV, run GetOutputData.py
~~~
>python ./data/GetOutputData.py
~~~

## Important Files

|File name|Explanation|
|:--|:--|
|Main.py|The entry point of SLAM program<br />Receive sensor data<br />Publish estimated location|
|image_RBPF.py|Parse sensor data of camera|
|landmark.py|Landmark (Keypoint in 3D space) class<br />Initialize landmark parameters<br />Observation model|
|particle.py|Particle class|
|particle_filter_RBPF.py|Particle filter|
|sensor.py|Parse sensor data of IMU|
|state_RBPF.py|Manage state variable|
|data/GetOutputData.py|Receive estimated data and save them as CSV|

## Data Flow

![](https://github.com/knagara/miscellaneous/blob/master/dataflow_.png)

<br /><br />

## 概要

[![](http://img.youtube.com/vi/IDZ5fxp_XdY/0.jpg)](https://www.youtube.com/watch?v=IDZ5fxp_XdY)

![](https://github.com/knagara/miscellaneous/blob/master/Overview.png)

- Androidアプリ（ここを参照 -> [SLAMwithCameraIMUforAndroid](https://github.com/knagara/SLAMwithCameraIMUforAndroid)）
- SLAMプログラム（このページ）
- MQTTブローカー（ここを参照 -> [MQTTについて詳しく知る](https://sango.shiguredo.jp/mqtt), [MQTTについてのまとめ](http://tdoc.info/blog/2014/01/27/mqtt.html), [MQTT Broker比較](http://acro-engineer.hatenablog.com/entry/2015/12/09/120500)）

## 使い方
1.Androidアプリをセットアップ（ここを参照 -> [SLAMwithCameraIMUforAndroid](https://github.com/knagara/SLAMwithCameraIMUforAndroid)）

2.MQTTブローカーをセットアップ（ここを参照 -> [sango](https://sango.shiguredo.jp/)）

3.PythonのMQTTパッケージをインストール
~~~
>pip install paho-mqtt
~~~
4.Main.pyを起動
~~~
>python Main.py
~~~
5.Androidアプリを起動

6.センサデータを受信し、位置の推定が始まります。推定結果はMQTTブローカーに送信されます。

7.推定結果をCSVファイルに保存したい場合は、GetOutputData.pyを起動
~~~
>python ./data/GetOutputData.py
~~~

## 重要なファイル

|File name|Explanation|
|:--|:--|
|Main.py|SLAMプログラムのエントリポイント<br />センサデータの受信<br />推定結果の送信|
|image_RBPF.py|画像センサデータのパース|
|landmark.py|ランドマーク（3D空間中の特徴点）クラス<br />ランドマークのパラメータの初期化<br />観測モデル|
|particle.py|パーティクルクラス|
|particle_filter_RBPF.py|パーティクルフィルタ|
|sensor.py|IMUセンサデータのパース|
|state_RBPF.py|状態変数の管理|
|data/GetOutputData.py|推定結果を受信してCSVで保存|

## データフロー

![](https://github.com/knagara/miscellaneous/blob/master/dataflow_.png)
