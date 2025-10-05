# Melon Moveit Config🍈

## 必要な依存関係

### インストール
```shell
# 不要なものもあるかも
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-moveit-planners-ompl \
  ros-$ROS_DISTRO-pilz-industrial-motion-planner \
  ros-$ROS_DISTRO-moveit-planners-chomp \
  ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-moveit \
  ros-$ROS_DISTRO-moveit-resources-panda-moveit-config \
  ros-$ROS_DISTRO-controller-manager \
  ros-$ROS_DISTRO-topic-based-ros2-control
```

### クローン
melon_descriptionパッケージを参照しているので、必ずcloneしておく
```shell
cd ~/ros2_ws/src
git clone https://github.com/SSatoya/melon_description
git clone https://github.com/SSatoya/melon_moveit_config.git
```

## ビルド

```shell
# ワークスペースに移動
cd ~/ros2_ws

# パッケージをビルド
colcon build 

# 環境設定を読み込み
source install/setup.bash
```

## 使用方法

### 1. Isaac simをスタート（再生ボタンを押す）
先に起動しておかないと、エラーになるときがある

### 2. moveitを起動
```shell
ros2 launch melon_moveit_config melon_mvoeit.launch.py
```

## 更新履歴
- 2025/08/28
    - launchファイルおよび、ro2_control周りの修正
    - first commit
