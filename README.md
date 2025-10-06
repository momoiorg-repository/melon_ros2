# 🍈 Melon Robot @Factory with ROS2 Humble
![Melon in Isaac Sim](doc/imgs/melon.png)

このリポジトリでは、モバイルマニピュレータ**Melon**を、**@factory**環境で実行するためのプラグインが提供されています。USDファイルのダウンロードスクリプト、ROS2 Humbleアプリケーションとして動作するMelonのデモコードが含まれています。<br>

## 🎯 概要

**Melon**は、モバイルマニピュレータロボットで、@factory環境においてNVIDIA Isaac Simと連携して使用することが前提です。

このプロジェクトでは、@factory環境で動作する際、以下の機能を提供します：

- **at_factoryに対するプラグインの提供（主にUSD定義からなる）**
- **ROS２ロボットとして動作するための基本機能、および、サンプル・アプリケーション**
    - MoveIt!設定
    - Nav2設定
    - Behavior Treeを使用したサンプル・アプリケーション

## 🚀 インストール

### 1. @factory環境へのプラグイン

@factoryへのプラグイン方法は[at_facoryリポジトリ](https://github.com/momoiorg-repository/at_factory.git)を参照してください<br>
参考までに、USDの実体はこちらのリポジトリの[finstall.shスクリプト](https://github.com/momoiorg-repository/at_factory?tab=readme-ov-file#finstallsh-%E3%82%B9%E3%82%AF%E3%83%AA%E3%83%97%E3%83%88)にあり、自動的にダンロードされるようになっています

### 2. ROS2ロボットとして動作させるためのdockerの実行  
以下の手順を実行してください

## ROS２関連設定  
### 1.リポジトリのクローン
```bash
git clone https://github.com/momoiorg-repository/melon_ros2.git
cd melon_ros2
```

### 2. ROS2環境設定
環境変数を設定するために`.env`ファイルを編集：

```bash
# .envファイルの設定例
ROS_DOMAIN_ID=80                    # ROSドメインID
CONTAINER_NAME=melon_ros2_app  # コンテナ名
```

### 3. Dockerコンテナのビルドと起動

以下のスクリプトを実行することで **docker image作成** → **dockerコンテナ作成** → **接続** まで一括実行できます。

```bash
./build.sh
```
コンテナ内に接続されたら、表示に従って初期設定を行ってください。<br>

- 初回のみ上記スクリプトを実行してください
- 2回目以降は以下で直接接続できます（VSCodeを使用している場合は自動的に実行されます）：

```bash
docker start <your container name>
docker exec -it <your container name> bash
```

## 🤖 使用方法

### @factoryへのプラグイン作業が完了していることが前提です

### MoveIt!とNavigation2の同時起動
アーム制御とベース移動を同時に可能にするローンチファイル：
```bash
ros2 launch melon_bringup melon_bringup
```

### MoveIt!のみの起動

アーム制御のためのMoveIt!起動：

```bash
ros2 launch melon_moveit_config melon_moveit.launch.py
```

### Navigation2のみの起動

ベース移動のためのNavigation2起動：

```bash
ros2 launch melon_navigation2 navigation.launch.py
```
1. `2D Pose Estimate`で初期位置を指定
2. `Nav2 Goal`でNavigationを開始

### Behavior Tree と ros_actor を使ったアプリケーションの実行

アプリケーションの使い方は[こちら](./doc/App_README.md)を参照してください。

## 🐛 トラブルシューティング

### よくある問題と解決法

#### ROS2通信の問題
```bash
# DDS設定確認
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds.xml
```

#### GUIアプリが表示されない
- 環境変数DISPLAYがあっているか確認
- 間違っていれば再設定
```bash
echo $DISPLAY

# 例
export DISPLAY=<your IP>:0
```

#### Isaac Sim と通信できないとき
- Isaac Simのシミュレーションを再生してからMoveit!やNavigationを実行
- Isaac Sim 側の ROS Bridge が起動しているか確認<br>
- ROS_DOMAIN_ID が一致しているか確認

## 📄 ライセンス

このプロジェクトのライセンス情報については、[LICENSE](./LICENSE)ファイルを参照してください。

## 📚 参考

- [at_factory](https://github.com/momoiorg-repository/isaacsim-common)
- [franka_description](https://github.com/frankarobotics/franka_description)
- [LimeSimulDemo](https://github.com/momoiorg-repository/LimeSimulDemo/tree/main)
