# 🍈 Melon Robot @Factory with ROS2 Humble
![Melon in Isaac Sim](doc/imgs/melon.png)

このリポジトリには、モバイルマニピュレータ**Melon**をIsaac Sim環境でシミュレーションするためのROS2 HumbleパッケージとUSDファイルのダウンロードスクリプト、Melonのデモコードが含まれています。<br>

## 🎯 概要

**Melon**は、モバイルマニピュレータロボットです。このプロジェクトでは以下の機能を提供します：

- **Isaac Sim とのシームレスな連携**
- **ROS2 Humbleベースの制御システム**
- **MoveIt!によるアーム軌道計画**
- **Navigation2による自律移動**
- **Behavior Treeを使用したタスク実行**

## 🚀 インストール

### 1. Isaac Sim環境の準備

Isaac Simは[@facoryリポジトリ](https://github.com/momoiorg-repository/at_factory.git)を使用してセットアップしてください<br>
USDのダウンロードは、こちらのリポジトリの[finstall.shスクリプト](https://github.com/momoiorg-repository/at_factory?tab=readme-ov-file#finstallsh-%E3%82%B9%E3%82%AF%E3%83%AA%E3%83%97%E3%83%88)を使用してください。

### 2. リポジトリのクローン

```bash
git clone https://github.com/momoiorg-repository/melon_ros2.git
cd melon_ros2
```

### 3. ROS2環境設定
環境変数を設定するために`.env`ファイルを編集：

```bash
# .envファイルの設定例
ROS_DOMAIN_ID=80                    # ROSドメインID
CONTAINER_NAME=melon_ros2_app  # コンテナ名
```

### 4. Dockerコンテナのビルドと起動

以下のスクリプトを実行することで **イメージ作成** → **コンテナ作成** → **接続** まで一括実行できます。

```bash
./build.sh
```
コンテナ内に接続されたら、表示に従って初期設定を行ってください。<br>

- 初回のみ上記スクリプトを実行してください
- 2回目以降は以下で直接接続できます：

```bash
docker start <your container name>
docker exec -it <your container name> bash
```

## 🤖 使用方法

### Isaac Simとの連携

1. Isaac Simでシミュレーション環境を開始
2. ROS2ブリッジで通信開始
3. 各種制御パッケージを起動

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
