# AIチャレンジ参加者用環境構築

## セットアップ

*※予選で使用した環境が既に存在している場合は、一度予選で設定された環境を削除して頂きます様お願いします*

### autowareのセットアップ
https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/Source-Build  
### git-lfsのインストール
https://packagecloud.io/github/git-lfs/install  
### 本レポジトリのcloneとビルド

_autowareを"~/autoware.ai"にセットアップした場合_

```
source ~/autoware.ai/install/local_setup.bash

mkdir -p ~/aichallenge_ws/src  
cd ~/aichallenge_ws
colcon build  
cd src  
git lfs clone https://github.com/aichallenge2020/aichallenge-bringup-final.git  
cd ../  
colcon build
```
### シミュレータのダウンロードと起動
下記リンクからシミュレータのバイナリをダウンロード  
https://www.jsae.or.jp/jaaic/docu/online_final/scenarios.zip 

上記リンクにあるzipファイルを伸長して頂くと  

- build_linux20200928a.zip(linux環境用シミュレータ)
- build_windows20200928a.zip(windows環境用シミュレータ)

と環境に応じて用意しております。実行環境に合わせて各zipファイル内のシミュレータをご使用下さい。  

*本シミュレータに関する免責事項について
参加者のみなさまが本ファイルをダウンロードすることによって、みなさまのコンピュータ、またはネットワーク環境  
等に支障・障害が生じた場合、本大会運営事務局はいかなる理由によるものでも一切責任を負いません。  
また、これらの事象によって生じた損害等についても、本大会運営事務局は一切責任を負いません。*  


linux環境でシミュレータを実行する場合は
下記の様にシミュレータのバイナリに実行属性を付与して下さい。
```
chmod a+x simulator
```


![画面](/image/initial.png)  

Web UIでアカウント登録を行った後mapダウンロード画面に移行する　　

![画面](/image/map.png)  

add newをクリックしたあとMap NameにBorregas Avenue,Map URLに[url](https://assets.lgsvlsimulator.com/65fac0d499d716bb9a7ca89803f672d97b366cf9/environment_BorregasAve)を入力

その後Vehicleダウンロード画面に移行する
![画面](/image/vehicle.png)  
Vehicle NameにLexus,Vechiel URLに[url](https://assets.lgsvlsimulator.com/b587cfe6b973c0d0a62959dae1eba561ded337da/vehicle_Lexus2016RXHybrid)を入力

![画面](/image/setting.png)  
Bridge TypeにROSを選択、本レポジトリのdata以下にあるlexus.jsonの中身をSensorsに貼り付ける

その後simulation画面に移行する
![画面](/image/simulation_general.png)
Simulation Nameに適当な名前をつけたあと、Map&Vehiclesを設定する。

![画面](/image/simulation_map_vehicles.png)
MapはBorregas Avenueを選択、VehicleはLexusを設定して(ROSの動作するPCのIPアドレス):9090の形式でrosbridgeに対する接続設定を実施

![画面](/image/simulation_run.png)
右下の赤色の三角をクリック、シミュレーションを開始

ターミナルを開いて以下のコマンドを入力  
```
source ~/autoware.ai/install/local_setup.bash  
source aichallenge_ws/install/local_setup.bash  
roslaunch aichallenge_bringup aichallenge_bringup.launch
```

別なターミナルを開いて以下のコマンドを入力
```
rviz
```
![画面](/image/rviz.png)

rvizのGUIが表示されたら左上のFileからOpen Configを選択し、aichallenge-bringup-finalパッケージのdataの中にあるaichallenge.rvizを選ぶ。

## シナリオを実行して点数を算出する
### LGSVL Simulator Python APIのセットアップ
https://github.com/lgsvl/PythonAPI
この記述に従ってPythonAPIをインストール

### シミュレータの操作
Web UIのSimulationタブを開いてシミュレーションを選択しレンチのボタンをクリック、API Onlyにチェックを入れる
![画面](/image/simulator_scenario.png)
以下のコマンドを叩いてシナリオを実行

シナリオ
```
roscd aichallnge_bringup/scenario
python3 scenario.train.py
```

### 点数の算出

競技内容に関しては[オンライン決勝](https://www.jsae.or.jp/jaaic/online_final.html#gnav)をご確認下さい。  

点数は/aichalle/scoreトピックにpublishされます。
確認方法は下記の通りです。

シミュレータの操作を開始した後に
```
source ~/autoware.ai/install/local_setup.bash
rostopic echo /aichalle/score
```
topicをechoさせる事で確認を取る事が出来ます。

又、publishされるタイミングは下記の場合になります。
1. ゴール到達時
1. 他車両や人・障害物に接触した場合
1. 路面外へ出た場合
1. scenaio.train.pyを実行し、所定の時間が経過した場合

/aichalle/scoreがpublishされた時点で点数の算出となります。

/aichalle/scoreの内容については下記の通りとなります
- seconds : 経過時間
- isGoal : ゴールに到達したらTrue
- isPassAllWayPoint : 設定してあるWayPointを全て通過していたらTrue
- isHitObject : 他車両や人・障害物に接触した場合にTrue
- isRootIgnore : 路面外へ出た場合にTrue
- isOverSpeed : 所定の速度を越えた場合にTrue

正式な点数は
isGoal : True
isPassAllWayPoint : True
isHitObject : False
isRootIgnore : False
isOverSpeed : False
の時のsecondsとなります。  


# 車両の制御に関して

## 車両の制御コマンド
車両にはautoware_msgs/VehicleCmd型で制御コマンドを送ることができます。  
本コンテストにおいてサポートしているコマンドはVehicleCmdの中のctrl_cmd内部に存在するlinear_acceleration,steering_angleとなります。  
また、gearの値は必ず64（ドライブ）に設定してください。 
gearの値が0の状態でも走り出しますが、意図しない制御になります為ご注意下さい。  
本コンテストのタスクにはバック走行が無いためバックギアはサポートいたしません。   

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
autoware_msgs/SteerCmd steer_cmd
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  int32 steer
autoware_msgs/AccelCmd accel_cmd
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  int32 accel
autoware_msgs/BrakeCmd brake_cmd
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  int32 brake
autoware_msgs/LampCmd lamp_cmd
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  int32 l
  int32 r
int32 gear
int32 mode
geometry_msgs/TwistStamped twist_cmd
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
autoware_msgs/ControlCommand ctrl_cmd
  float64 linear_velocity
  float64 linear_acceleration
  float64 steering_angle
int32 emergency
```

steering_angleに関しては-1から1の値をfloatで入力してください。

その値に比例して~39.4から39.4の範囲で車両の操舵角が変化します。

正の値を入力した場合右にステアがきれます。

linear_accelerationはアクセル・ブレーキの入力値を-1~1の範囲で正規化したものとなります。

正の値を入力した場合加速、負の値を入力した場合減速します。  

# オンライン評価環境にファイルをアップする手順

オンライン評価環境へのアップロードについては2020/10/19に開始します。  
アップロード方法等についても同日にこちらREADMEに掲載します。


# お問い合わせ

## 更新等の通知に関して

githubの更新などがある場合は、以下のURLのissueに新たにコメントします。  
本issueをsubscribeいただければ、更新時に通知されます（通知をオンにしてください）。  
https://github.com/aichallenge2020/aichallenge-bringup-final/issues/1  


## お問い合わせ受付に関して  
「ai-challenge@simulation.tier4.jp」は、送信専用アドレスです。  
このメールに返信されても、返信内容の確認およびご返答ができません。  
お問い合わせについては、github上のissueにてお願いします。  

