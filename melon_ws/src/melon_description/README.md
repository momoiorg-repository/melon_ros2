# Melon description🍈

## install

- ros2環境で行う場合は、必ず[franka_description](https://github.com/frankarobotics/franka_description)もcloneして使用してください。

## パッケージ構成

```
melon_description/
├── config/                # 設定ファイル
│   ├── display.rviz       # RViz設定
│   ├── gazebo.rviz        # Gazebo用RViz設定
│   └── ros_gz_bridge_gazebo.yaml  # ROS-Gazeboブリッジ設定
├── launch/                # 起動ファイル
│   ├── display.launch.py  # RViz表示用
│   └── gazebo.launch.py   # Gazeboシミュレーション用
├── meshes/                # 3Dメッシュファイル
├── urdf/                         # ロボット記述ファイル
│   ├── old                       # 古いロボット記述ファイル
│   ├── base_v2.gazebo.xacro      # Gazebo設定
│   ├── base_v2.ros2control.xacro # ROS2 Control設定
│   ├── base_v4.xacro.xacro       # ロボットのベース部分記述
│   ├── materials.xacro           # マテリアル定義
│   ├── melon.urdf                # MelonロボットURDF
│   └── melon.urdf.xacro          # MelonロボットXacro
├── usd/                            # Isaac sim用
│   ├── melon_v4                    # Melonロボットのusdファイル             
│   └── Collected_melon_with_graph  # MelonロボットとAction Graphファイル
└── test/                           # テストファイル
```

## RVizでロボットモデルを表示

基本的なロボットモデルの表示：
```bash
ros2 launch melon_description display.launch.py
```

## 利用可能なロボットモデル

- **Melon v2**: <br>
基本的な工場作業ロボット (`melon.urdf.xacro`)

## 更新履歴
- 2025/09/02: realsenseの取り付け
- 2025/08/26:
    - base_footprintの追加
    - lidar sensorの取り付け
    - ロボットの初期姿勢の変更
- 2025/08/22: passive wheelの修正
- 2025/08/21: first commit