# jnmouse_ros_examples
Jetson Nano MouseのROSサンプルコード集です。

## Requirements

- Jetson Nano Mouse
  - https://rt-net.jp/products/raspberrypimousev3/
  - Linux OS
    - Ubuntu server 18.04
  - ROS
    - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
  - Raspberry Pi Mouse ROS package
    - https://github.com/ryuichiueda/raspimouse_ros_2
- Remote Computer (Optional)

## Installation

```sh
cd ~/catkin_ws/src
# Clone ROS packages
git clone https://github.com/ryuichiueda/raspimouse_ros_2
git clone https://github.com/rt-net/jnmouse_ros_examples

# Install dependencies
rosdep install -r -y --from-paths . --ignore-src      

# make & install
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

## License

このリポジトリはApache 2.0ライセンスの元、公開されています。 
ライセンスについては[LICENSE](./LICENSE)を参照ください。

## How To Use Examples
### line_following

左カメラでキャプチャした黒線を使ってライントレースを行うコード例です．

#### How to use

次のスクリプトを実行して、カメラの自動調節機能（自動露光，オートホワイトバランス等）を切ります。


```sh
rosrun raspimouse_ros_examples camera.bash
```

次のコマンドでノードを起動します。

```sh
roslaunch jetson_nano_mouse_line_tracing line_tracing.launch
```

