# Melon Navigation🍈

## 必要な依存関係

### インストール

```shell
# Navigation2関連パッケージのインストール
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-pointcloud-to-laserscan
```

## ビルド

```shell
# ワークスペースに移動
cd ~/ros2_ws

# パッケージをビルド
colcon build --packages-select melon_navigation2

# 環境設定を読み込み
source install/setup.bash
```

## 使用方法

### 1. ナビゲーションの起動

```shell
# Navigation2を起動
ros2 launch melon_navigation2 navigaiton.launch.py
```

### 2. RVizでの操作

1. RVizが起動したら、`2D Pose Estimate`ツールを使用してロボットの初期位置を設定
2. `2D Goal Pose`ツールを使用して目標位置を設定
3. ロボットが自動的に経路を計画し、目標位置に向かって移動

## パッケージ構成
```
melon_navigation2/
├── launch/                 # Launchファイル
│   └── navigaiton.launch.py
├── map/                    # マップファイル
│   ├── draft_lime.png
│   └── draft_lime.yaml
├── params/                 # パラメータファイル
│   └── nav2_paramas.yaml
├── rviz/                   # RViz設定ファイル
│   └── navigation2.rviz
└── melon_navigation2/      # Pythonパッケージ
```

## 設定ファイル

### マップファイル
- `map/draft_lime.yaml`: マップのメタデータ
- `map/draft_lime.png`: マップ画像

### パラメータファイル
- `params/nav2_paramas.yaml`: Navigation2の設定パラメータ

### RViz設定
- `rviz/navigation2.rviz`: ナビゲーション用のRViz設定


## 関連パッケージ

- `melon_description`: ロボットの記述ファイル
- `franka_description`: Frankaアームの記述ファイル

## 更新履歴
- 2025/08/27: first commit