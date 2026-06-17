# sllidar_ros2 for ARGOS

このリポジトリは、SLAMTEC の `sllidar_ros2` を ARGOS ロボットで使いやすくするための派生リポジトリです。

ARGOS では RPLiDAR A1M8 を主な対象として使います。ただし、この package は A1M8 専用実装にはせず、SLAMTEC LiDAR 向けの汎用 ROS 2 ドライバとして維持します。ARGOS 固有の既定値は launch と README に集約し、ドライバ本体と SDK は上流追従性を優先します。

## ARGOS での基本方針

- ROS 2 は Jazzy を想定します。
- LiDAR は serial 接続で `/dev/rplidar` として扱います。
- `/scan` の `header.frame_id` は `laser` に統一します。
- `base_link -> laser` の TF は `robot_description` または `robot_bringup` 側で管理します。
- `sllidar_ros2` 側には URDF を置きません。
- 複数ロボット間の分離は通常 `ROS_DOMAIN_ID` で行います。
- `namespace` は必須ではありませんが、将来拡張用に ARGOS launch の引数として用意しています。

## クローン方法

ROS 2 workspace の `src` に clone します。

```bash
cd ~/argos_ws/src
git clone git@github.com:Fujisawa-lab-inside/sllidar_ros2_argos.git sllidar_ros2
```

リモートリポジトリ名は `sllidar_ros2_argos` ですが、ローカルディレクトリ名は `sllidar_ros2` にします。これは ROS 2 package 名 `sllidar_ros2` との整合性を保つためです。

`~/argos_ws` は例です。実際の workspace 名や配置に合わせて読み替えてください。

## ビルド方法

ROS 2 Jazzy の環境を読み込んだうえで build します。

```bash
cd ~/argos_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sllidar_ros2
source install/setup.bash
```

`colcon` が見つからない場合は、ROS 2 Jazzy の colcon 環境が入っているか確認してください。

## 起動方法

ARGOS での通常起動は次のコマンドです。

```bash
ros2 launch sllidar_ros2 argos_sllidar_serial_launch.py
```

この launch は ARGOS 用の serial 接続 SLAMTEC LiDAR launch です。RPLiDAR A1M8 向けの既定値を持ちますが、A1M8 専用ではありません。

既定値は次のとおりです。

| launch argument | default |
| --- | --- |
| `channel_type` | `serial` |
| `serial_port` | `/dev/rplidar` |
| `serial_baudrate` | `115200` |
| `frame_id` | `laser` |
| `inverted` | `false` |
| `angle_compensate` | `true` |
| `scan_mode` | `Sensitivity` |
| `namespace` | 空文字 |

launch 引数を変更する例です。

```bash
ros2 launch sllidar_ros2 argos_sllidar_serial_launch.py \
  serial_port:=/dev/rplidar \
  serial_baudrate:=115200 \
  frame_id:=laser
```

別の SLAMTEC LiDAR を使う場合は、機種に合わせて `serial_baudrate` や `scan_mode` を変更してください。

```bash
ros2 launch sllidar_ros2 argos_sllidar_serial_launch.py \
  serial_baudrate:=256000 \
  scan_mode:=Sensitivity
```

`namespace` を指定しない場合、scan topic は通常 `/scan` です。`namespace:=robot1` のように指定した場合は、node は namespace 配下で起動し、topic は通常 `/robot1/scan` になります。

## 確認方法

デバイスが見えているか確認します。

```bash
ls -l /dev/rplidar
```

topic が出ているか確認します。

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```

TF tree を確認する場合は次を実行します。

```bash
ros2 run tf2_tools view_frames
```

このリポジトリでは実機接続や Raspberry Pi 上での動作確認結果を保証する記述はしません。上記コマンドで各環境ごとに確認してください。

## TF / URDF との関係

`sllidar_ros2` は `/scan` を publish する LiDAR ドライバとして薄く保ちます。

- `sllidar_ros2` 側に URDF は置きません。
- LiDAR の frame は `laser` とします。
- `/scan` の `header.frame_id` は `laser` になります。
- `base_link -> laser` の TF は `robot_description` または `robot_bringup` 側で管理します。
- LiDAR がロボットに固定されている場合、通常は `/tf_static` で十分です。
- `/scan` の `frame_id` と TF の child frame 名が一致していないと、RViz、SLAM、Nav2 で TF error が出ます。

ARGOS 側では、LiDAR の取り付け位置に合わせて `base_link -> laser` の static transform を robot 側の bringup に含めてください。

## udev / scripts の扱い

ARGOS 環境では、udev ルールは Raspberry Pi ホスト側で管理します。通常運用では `scripts/` 以下を実行する必要はありません。

- ホスト側とコンテナ側の両方で `/dev/rplidar` が見えていれば十分です。
- `scripts/` は上流由来の補助スクリプトとして残します。
- `/dev/rplidar` が見えない場合のみ、udev ルールや Docker の device mount を確認してください。

コンテナ利用時は、ホスト側に `/dev/rplidar` が存在していても、コンテナへデバイスを渡していなければ `sllidar_ros2` からは使えません。Docker や compose の設定で `/dev/rplidar` をコンテナへ渡してください。

## ROS_DOMAIN_ID と namespace

ARGOS では、各ロボットで `ROS_DOMAIN_ID` を変更して運用する方針です。そのため、通常は `/scan` topic の衝突を namespace で回避しません。

同一 `ROS_DOMAIN_ID` 内で複数ロボットを扱う場合は、`namespace` や topic remapping の検討が必要です。その場合は、LiDAR だけでなく robot state publisher、TF、Nav2、SLAM なども含めて namespace 方針をそろえてください。

## トラブルシュート

### `/dev/rplidar` が見えない

```bash
ls -l /dev/rplidar
```

ホスト側で見えない場合は、Raspberry Pi 側の udev ルール、USB 接続、LiDAR の電源を確認してください。コンテナ内で見えない場合は、Docker の device mount 設定を確認してください。

### `/scan` が出ない

```bash
ros2 topic list
```

`/scan` が出ない場合は、launch が起動しているか、`serial_port` が正しいか、コンテナ内から `/dev/rplidar` にアクセスできるか確認してください。別 namespace を指定している場合は、`/robot1/scan` のように namespace 配下の topic を確認してください。

### `ros2 topic hz /scan` が不安定

```bash
ros2 topic hz /scan
```

LiDAR の電源、USB ケーブル、CPU 負荷、コンテナ設定、`serial_baudrate`、`scan_mode` を確認してください。LiDAR の機種によって適切な baudrate や scan mode は異なります。

### RViz で TF error が出る

```bash
ros2 topic echo /scan --once
ros2 run tf2_tools view_frames
```

`/scan` の `header.frame_id` が `laser` になっているか確認してください。また、robot 側で `base_link -> laser` の TF が publish されているか確認してください。`/scan` の frame_id と TF の child frame 名が一致していないと、RViz、SLAM、Nav2 で TF error が出ます。

### `scan_mode:=Sensitivity` で起動しない

RPLiDAR A1M8 では `Sensitivity` を既定値にしていますが、機種や firmware によっては対応していない場合があります。その場合は `scan_mode` を空文字、または LiDAR が対応する別のモードに変更して確認してください。

```bash
ros2 launch sllidar_ros2 argos_sllidar_serial_launch.py scan_mode:=
```

または、機種に応じて次のように変更します。

```bash
ros2 launch sllidar_ros2 argos_sllidar_serial_launch.py scan_mode:=Standard
```

## SLAMTEC 汎用情報

この package は SLAMTEC LiDAR 用 ROS 2 ドライバです。ARGOS では A1M8 を既定対象にしますが、上流の SLAMTEC LiDAR 汎用ドライバとしての性質は残します。

SLAMTEC LiDAR 関連リンク:

- SLAMTEC LIDAR roswiki: <http://wiki.ros.org/rplidar>
- SLAMTEC LIDAR HomePage: <http://www.slamtec.com/en/Lidar>
- SLAMTEC LIDAR SDK: <https://github.com/Slamtec/rplidar_sdk>
- SLAMTEC LIDAR Tutorial: <https://github.com/robopeak/rplidar_ros/wiki>

上流 README で示されていた主な対応 LiDAR:

| Lidar Model |
| --- |
| RPLIDAR A1 |
| RPLIDAR A2 |
| RPLIDAR A3 |
| RPLIDAR C1 |
| RPLIDAR S1 |
| RPLIDAR S2 |
| RPLIDAR S3 |
| RPLIDAR S2E |
| RPLIDAR T1 |

既存の `launch/sllidar_*_launch.py` や `launch/view_sllidar_*_launch.py` は上流由来のモデル別 launch として残しています。ARGOS 通常運用では `launch/argos_sllidar_serial_launch.py` を使ってください。
