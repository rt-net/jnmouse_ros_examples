# jnmouse_ros_examples
Jetson Nano MouseのROSサンプルコード集です。

## Requirements

### Hardware
- Jetson Nano Mouse
  - https://rt-net.jp/products/raspberrypimousev3/
- Remote Computer (Optional)

### Software
  - Linux OS
    - Ubuntu server 18.04
  - ROS
    - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
  - Raspberry Pi Mouse ROS package
    - https://github.com/ryuichiueda/raspimouse_ros_2

## Installation

Jetson Nano用のOS、L4Tのセットアップは[こちらのブログ記事](https://rt-net.jp/mobility/archives/14941)をご覧ください。
ROSのセットアップは[こちらのブログ記事](https://rt-net.jp/mobility/archives/15162)をご覧ください。

```sh
cd ~/catkin_ws/src
# Clone ROS packages
git clone https://github.com/ryuichiueda/raspimouse_ros_2
git clone https://github.com/rt-net/jnmouse_ros_examples

# Install dependencies
rosdep install -r -y -i --from-paths .

# make & install
cd ~/catkin_ws && catkin build
source devel/setup.bash
```

## How To Use Example
### line_following

左カメラでキャプチャした黒線を使ってライントレースを行うコード例です．

#### How to use

次のコマンドでノードを起動します。

```sh
roslaunch jetson_nano_mouse_line_tracing line_tracing.launch
```

ライントレースのコース上で実行すればJetson Nano Mouseがコースに沿って移動します．
実行中は/line_follower_imgに追跡中のラインを示す画像が配信されます．

リモートでrqt_image_viewなどを使用すれば，実際にどのラインを検出しているのかが分かるようになっています．

```
rqt_image_view
```


## How To Use Example
### stereo_line_following

左右カメラからパノラマ画像を作成し，ライントレースを行うコード例です．

#### How to use
##### set_image_points 
最初にパノラマ画像作成のため，左右画像間で共通する特徴点の位置を記録します．

Jetson Nano Mouseの左右カメラから見える位置にチェッカーパターンを印刷した紙を
ライントレースのコース上に置いて下さい．

左カメラ画像上での特徴点を記録するため，次のコマンドでノードを起動します。
左右カメラそれぞれで画像特徴点の記録が完了するまで，機体やチェッカーパターンが動かないように注意して下さい．

```sh
roslaunch jetson_nano_mouse_line_tracing set_image_points.launch camera_position:=left
```

hjklキー，または左クリックでカーソル移動，
pキー，または中クリックで特徴点を設定，
右クリックで最後に記録した特徴点の削除，
qキーで終了します．

チェッカーパターンの格子点にカーソルを合わせたら，pキーまたは中クリックで特徴点を設定します．
特徴点の設定順は左右カメラで同じ順番であれば問題ありませんが，左上から順に行うのが良いと思います．

同様に右カメラ画像上での特徴点を記録するため，次のコマンドでノードを起動します。

```sh
roslaunch jetson_nano_mouse_line_tracing set_image_points.launch camera_position:=left
```


左右カメラで特徴点の記録が完了したら，機体やチェッカーパターンを動かしても大丈夫です．
一連の操作によって左右画像間での特徴点の対応づけが完了し，パノラマ画像作成に必要なホモグラフィ行列の推定が可能となりました．

##### stereo_line_following

次のコマンドでノードを起動します。

```sh
roslaunch jetson_nano_mouse_line_tracing line_tracing.launch
```

ライントレースのコース上で実行すればJetson Nano Mouseがコースに沿って移動します．
実行中は/line_follower_imgに追跡中のラインを示す画像が配信されます．

リモートでrqt_image_viewなどを使用すれば，実際にどのラインを検出しているのかが分かるようになっています．

```
rqt_image_view
```

## ライセンス

(C) 2020 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。  
バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
