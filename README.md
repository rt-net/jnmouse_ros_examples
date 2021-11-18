# jnmouse_ros_examples

Jetson Nano MouseのROSサンプルコード集です。

<h2 id="toc">目次</h2>

* [動作環境](#requirements)
* [インストール](#installation)
* [サンプルの実行方法](#samples)
  * [ライントレース](#line_following)
  * [ステレオカメラ画像の歪み補正とステレオ平行化](#image_undistortion)
  * [ステレオカメラ画像からの深度推定](#stereo_depth_estimation)

<h2 id="requirements">動作環境</h2>

### ハードウェア
- [Jetson Nano Mouse](https://rt-net.jp/products/jetson-nano-mouse/)
  - with [Jetson Nano Dev. Kit B01](https://ryoyo-gpu.jp/products/jetson/nano2/)
- Remote Computer (オプション)

### ソフトウェア
#### Jetson Nano Mouse
  - Ubuntu 18.04 LTS
    - jetbot_image_v0p4p0.zip (L4T R32.3.1)
      - イメージファイルは[こちら](https://jetbot.org/master/software_setup/sd_card.html#old-releases)からダウンロードできます。
  - ROS
    - [Melodic Morenia](https://wiki.ros.org/melodic/Installation/Ubuntu)
  - Raspberry Pi Mouse ROS package
    - https://github.com/ryuichiueda/raspimouse_ros_2

#### Remote Computer
  - Ubuntu 18.04 LTS
  - ROS
    - [Melodic Morenia](https://wiki.ros.org/melodic/Installation/Ubuntu)

[↑目次に戻る](#toc)

<h2 id="installation">インストール</h2>

Jetson Nano用のOS、L4Tのセットアップは[こちらのブログ記事](https://rt-net.jp/mobility/archives/14941)をご覧ください。  
ROSのセットアップは[こちらのブログ記事](https://rt-net.jp/mobility/archives/15162)をご覧ください。

```sh
cd ~/catkin_ws/src
# Clone ROS packages
git clone https://github.com/rt-net/jnmouse_ros_examples.git
git clone https://github.com/rt-net/jetson_nano_csi_cam_ros.git 
git clone https://github.com/rt-net/gscam.git
git clone https://github.com/ryuichiueda/raspimouse_ros_2.git

# Install dependencies
rosdep install -r -y -i --from-paths .

# make & install
cd ~/catkin_ws && catkin build
source devel/setup.bash
```

本パッケージのスクリプトを実行する際はCPUのパフォーマンスを優先するMAXNモードを推奨します。下記コマンドでMAXNモードに変更できます。

```
sudo nvpmodel -m 0
```

CPUパフォーマンスとエネルギー消費を抑える5Wモードに戻す場合は下記コマンドを実行します。

```
sudo nvpmodel -m 1
```

[↑目次に戻る](#toc)

<h2 id="samples">サンプルの実行方法</h2>

### line_following

左カメラでキャプチャした黒線を使ってライントレースを行うコード例です。

#### 使い方

次のコマンドでノードを起動します。

```sh
roslaunch jnmouse_ros_examples line_following.launch
```

ライントレースのコース上で実行すればJetson Nano Mouseがコースに沿って移動します。

実行中は/line_follower_imgに追跡中のラインを示す画像が配信されます。  
リモートでrqt_image_viewなどを使用すれば、実際にどのラインを検出しているのかが分かるようになっています。

```sh
rqt_image_view
```

![](https://rt-net.github.io/images/jetson-nano-mouse/jnmouse_ros_examples_line_follwing_screenshot.png)

[↑目次に戻る](#toc)

### image_undistortion

ステレオカメラ画像の歪み補正とステレオ平行化を行うコード例です。

#### 使い方

[こちらのNotebook](https://github.com/rt-net/jnm_jupyternotebook/tree/master/notebooks/camera_undistort)でカメラパラメータファイルを作成し`config/camera_param_fisheye.npz`を作成したものと置き換えます。  

次のコマンドでノードを起動します。

```sh
roslaunch jnmouse_ros_examples image_undistortion.launch
```

/camera_l/image_rect_color、/camera_r/image_rect_colorに歪み補正とステレオ平行化された画像が配信されます。  
リモートでrqt_image_viewなどを使用すれば、補正前と補正後の画像を見比べることができます。

```sh
rqt_image_view
```

![](https://rt-net.github.io/images/jetson-nano-mouse/jnmouse_ros_examples_image_undistortion_screenshot.png)

下記コマンドで歪み補正した画像を用いてライントレースを行います。

```sh
roslaunch jnmouse_ros_examples line_following_undistortion.launch
```

[↑目次に戻る](#toc)

### stereo_depth_estimation

ステレオカメラ画像から深度推定を行うコード例です。

#### 使い方
image_undistortionによる補正済み画像を用いるため、image_undistortionの環境構築と動作確認を行ってください。

次のコマンドでノードを起動します。

```sh
roslaunch jnmouse_ros_examples stereo_depth_estimation.launch
```

/depth/image_rectに深度画像が配信されます。  
リモートでrvizなどを使用すれば、深度画像を見ることができます。

```sh
rviz
```

![](https://rt-net.github.io/images/jetson-nano-mouse/jnmouse_stereo_depth.png)

下記コマンドを実行した場合はカメラ中央に写った物体までの推定距離がターミナルに表示されます。また目印として深度画像中央に円が描画されます。

```sh
roslaunch jnmouse_ros_examples stereo_depth_estimation.launch debug:=true
```

[↑目次に戻る](#toc)

<h2 id="license">ライセンス</h2>

(C) 2020 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。
バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。


[↑目次に戻る](#toc)
