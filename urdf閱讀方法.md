# URDF 閱讀方法

## 在 Ubuntu 怎麼看

### 最快的驗證(不用 mesh、幾秒鐘)

```bash
sudo apt install liburdfdom-tools
check_urdf CAR_ASSEMBLE_URDF.urdf      # 印出 link/joint 樹,驗證有沒有錯
urdf_to_graphiz CAR_ASSEMBLE_URDF.urdf # 產生一張樹狀關係 PDF
```

### 看 3D 模型(含 mesh、可以手動轉動輪子)

把匯出的整個資料夾(要含 `meshes/`、`package.xml`)放進 ROS 2 工作區再開 RViz,`package://` 路徑才找得到 STL:

```bash
mkdir -p ~/ros2_ws/src && cp -r car_assemble_description ~/ros2_ws/src/
cd ~/ros2_ws && colcon build && source install/setup.bash
sudo apt install ros-humble-urdf-tutorial   # 換成你的 ROS 版本
ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix car_assemble_description)/share/car_assemble_description/urdf/CAR_ASSEMBLE_URDF.urdf
```

> 或直接用這個 repo 自己轉好的 launch 檔案:`ros2 launch car_assemble_description display.launch.py`(見下方「ROS 2 轉換」章節)。

開起來後會有一個 `joint_state_publisher_gui` 的拉桿視窗,拉動就能看四顆輪子轉、確認軸向對不對。

> Windows 沒有好用的原生工具,線上 viewer 又常卡在 mesh 路徑,建議直接用你的 Ubuntu 電腦。

## 關於「給你資料」

- **檢查邏輯**:不用給,看 `.urdf` 就夠了。
- **想要直接出一張渲染圖**:把 `meshes/` 裡的 6 個 STL 上傳,我可以用程式照 URDF 的位置組起來、輸出一張 PNG 給你先看外觀 —— 這樣不用先把 ROS 環境弄好就能看到長相。但要「互動轉動」還是 RViz 最合適。

## 待辦提醒

之前算的 N20 馬達 `effort 0.5` / `velocity 50` 目前還沒寫進這個檔案(匯出時那兩格是空的)。

- 純 RViz 看不需要。
- 等要接 `ros2_control` 或跑 Gazebo 時,再在每個 `wheel_joint` 裡加一行:

```xml
<limit effort="0.5" velocity="50"/>
```

要我幫你把 `base_footprint`、四顆輪子的 `<limit>`、還有麥輪 `ros2_control` 區塊直接寫成可貼上的內容嗎?或是你先上傳 6 個 STL,我出張渲染圖給你確認外觀?

## URDF 檢查結果(2026-07-13)

已經對 `car_assemble_description/urdf/CAR_ASSEMBLE_URDF.urdf`(當時套件資料夾還叫 `CAR_ASSEMBLE_URDF`,後來改名見下方章節)跑過 `check_urdf`,結果如下:

```
robot name is: CAR_ASSEMBLE_URDF
---------- Successfully Parsed XML ---------------
root Link: base_link has 5 child(ren)
    child(1):  front_left_wheel_link
    child(2):  front_right_wheel_link
    child(3):  lidar_link
    child(4):  rear_left_wheel_link
    child(5):  rear_right_wheel_link
```

**結論:XML 語法正確、樹狀結構正確,沒有孤兒 link 或重複 joint。** 另外確認 6 個 STL(base_link、lidar_link、四顆輪子)都存在於 `meshes/` 裡,`package://` 路徑對得起來。

### 發現的問題

1. **這包原本是 ROS 1(catkin)格式,已於 2026-07-13 轉成 ROS 2(ament_cmake)** —— 詳見下方新增的「ROS 2 轉換(2026-07-13)」章節。

2. **N20 馬達的 `<limit effort="" velocity=""/>` 還沒補**(如上面待辦提醒),四個 `continuous` 輪子 joint 目前都沒有這行。純看模型沒差,但要跑 `ros2_control` / Gazebo 前必須補上。

3. **`rear_left_wheel_joint` 的旋轉角度跟其他三顆不太一樣,建議你確認一下**
   - 四顆輪子的 `<joint><origin rpy="...">`:
     - front_left: `rpy="-3.1416 0 0"`
     - front_right: `rpy="3.1416 0 -3.1416"`
     - rear_right: `rpy="-3.1416 0 3.1416"`
     - rear_left: `rpy="0 -0.4646 3.1416"` ← 多了一個 pitch(繞 Y)方向 **-0.4646 rad(約 -26.6°)**,其他三顆都是 0。
   - 因為輪子的轉軸本來就是 Y 軸(`axis xyz="0 1 0"`),繞 Y 的 pitch 不會改變轉軸方向(數學上會抵消),所以**不影響輪子能不能正常轉動**,check_urdf 也不會報錯。
   - 但這通常代表 SolidWorks 裡 rear_left 這顆輪子的 mesh 座標系跟其他三顆沒對齊,實際渲染出來這顆輪子的視覺角度可能會歪一點點。建議你開 RViz 或渲染圖時特別看一下這顆輪子外觀有沒有跟其他三顆一致,如果是設計上刻意的內傾角(camber)可以忽略,如果不是就要回 SolidWorks 檢查那顆輪子的座標系/mate。

### 建議下一步

- 先確認你是 ROS 1 還是 ROS 2 環境,再決定要不要動 `package.xml` / `CMakeLists.txt`。
- 之後接 `ros2_control` 前記得補上四顆輪子的 `<limit>`。
- 上傳 6 個 STL 讓我出渲染圖,順便肉眼確認 rear_left 輪子角度是否正常。

## Git LFS mesh 檔案問題(2026-07-13 補充,已修復)

第一次檢查時發現 `meshes/` 裡的 6 個 `.STL` 雖然「存在」,但其實只是 **Git LFS 指標檔**(每個約 130 bytes 的純文字,內容是 `version https://git-lfs.github.com/spec/v1` + `oid` + `size`),不是真正的二進位網格資料。原因是 repo 的 `.gitattributes` 有設定 `*.STL filter=lfs`,但這台機器當時沒裝 `git-lfs`。

`check_urdf` 不會發現這個問題,因為它只解析 XML、不會去讀 mesh 內容 —— 但如果直接拿去開 RViz/Gazebo,輪子和車身會顯示不出來。

**已修復**:安裝 `git-lfs`(`sudo apt-get install -y git-lfs`)→ `git lfs install` → `git lfs pull`,把 6 個 STL 的真實內容拉下來。已用 Python 讀 STL 二進位 header(80 bytes header + 4 bytes 三角面數)驗證檔案大小與宣告的三角面數吻合,確認是完整、未損毀的檔案:

| link | 檔案大小 | 三角面數 |
|---|---|---|
| base_link | 34,043,784 bytes | 680,874 |
| lidar_link | 863,084 bytes | 17,260 |
| front/rear ×2 wheel | 2,742,584 bytes(各) | 54,850(各) |

之後如果重新 clone 這個 repo,記得先裝 `git-lfs` 再 clone,否則同樣的問題會再發生一次。

## ROS 2 轉換(2026-07-13)

因為你會用 ROS 2(假設 Humble;Gazebo 假設用 Gazebo Classic / `gazebo_ros`,不是新版 `ros_gz`),把 SolidWorks 匯出的 ROS 1 catkin 包轉成 ament_cmake:

- **`package.xml`**:`format="2"` + `<buildtool_depend>catkin</buildtool_depend>` → `format="3"` + `<buildtool_depend>ament_cmake</buildtool_depend>`,依賴改成 `<exec_depend>`(`robot_state_publisher`、`joint_state_publisher_gui`、`rviz2`、`gazebo_ros`、`xacro`)。
- **`CMakeLists.txt`**:`find_package(catkin REQUIRED)` + `catkin_package()` → `find_package(ament_cmake REQUIRED)` + `ament_package()`,install 目的地從 `${CATKIN_PACKAGE_SHARE_DESTINATION}` 改成 `share/${PROJECT_NAME}`。
- **`launch/display.launch` → `launch/display.launch.py`**:XML launch 改成 Python launch(ROS 2 的主流寫法),邏輯不變(`robot_state_publisher` + `joint_state_publisher_gui` + `rviz2`)。原本 `-d urdf.rviz` 的 RViz config 檔案在 repo 裡其實不存在(SolidWorks 匯出並沒有帶出來),所以新的 launch 檔案先不指定 config,開起來是 RViz 預設畫面,要看到 robot model 記得自己在 RViz 裡手動 `Add > RobotModel` 並把 Fixed Frame 設成 `base_link`。
- **`launch/gazebo.launch` → `launch/gazebo.launch.py`**:改用 `gazebo_ros` 套件的 `gazebo.launch.py`(取代 ROS 1 的 `empty_world.launch`)+ `spawn_entity.py`(取代 `spawn_model`)+ `tf2_ros static_transform_publisher`(取代 ROS 1 的 `tf` 套件,參數改成 `--x/--y/--z/--yaw/--pitch/--roll/--frame-id/--child-frame-id` 的具名參數,拿掉了 ROS 1 版本才有的 latch period 參數)。原本 ROS 1 版本最後那個 `fake_joint_calibration`(用 `rostopic pub /calibrated std_msgs/Bool true`)是 ROS 1 特有的手動校正 topic 慣例,ROS 2 沒有對應機制,轉換時直接拿掉了 —— 如果你的下游程式碼有訂閱 `/calibrated` 才需要另外處理。

**已驗證**:在乾淨的 `~/ros2_ws` 下 `colcon build` 成功,`ros2 launch car_assemble_description display.launch.py --show-args` 可以正常解析。`gazebo.launch.py` 的 Python 語法本身也驗證過沒問題,但這台機器沒裝 `gazebo_ros`,所以還沒實際跑起來過,正式用之前建議你在有裝 Gazebo 的環境再跑一次。

**還沒動、你可能要注意的地方:**

1. `config/joint_names_CAR_ASSEMBLE_URDF.yaml` 是 ROS 1 `ros_control` 的 `controller_joint_names` 格式,ROS 2 的 `ros2_control` 設定檔格式完全不同,等你要接 `ros2_control` 時這個檔案需要重寫,不是單純轉檔。
2. 四顆輪子 joint 缺 `<limit effort="" velocity=""/>` 的問題還沒處理(前面章節已提過)。

## 套件改名(2026-07-13)

原本的套件資料夾/名稱 `CAR_ASSEMBLE_URDF` 不符合 ROS 2 慣例(ament 要求全小寫、只能有小寫字母/數字/底線),已改名成 `car_assemble_description`(ROS 慣例上,純放 URDF/mesh/launch 的描述性套件通常用 `_description` 結尾,例如 `turtlebot3_description`)。改動範圍:

- 資料夾:`CAR_ASSEMBLE_URDF/` → `car_assemble_description/`
- `package.xml` 的 `<name>`
- `CMakeLists.txt` 的 `project()`
- `launch/display.launch.py`、`launch/gazebo.launch.py` 裡的 `get_package_share_directory(...)`
- **`urdf/CAR_ASSEMBLE_URDF.urdf` 和 `urdf/CAR_ASSEMBLE_URDF.csv` 裡全部 12 處 `package://CAR_ASSEMBLE_URDF/meshes/...`**(這個最關鍵,不改的話 RViz/Gazebo 會因為套件名對不上而找不到 mesh)

**故意沒動的地方**(避免非必要的骨牌效應):
- `urdf/` 底下兩個檔案本身的檔名還是 `CAR_ASSEMBLE_URDF.urdf` / `CAR_ASSEMBLE_URDF.csv`,沒有跟著改成小寫 —— 檔名不影響 `package://` 解析,只是單純沒改。
- URDF 內 `<robot name="CAR_ASSEMBLE_URDF">` 這個機器人模型名稱,以及 Gazebo `spawn_entity.py` 的 `-entity CAR_ASSEMBLE_URDF` 沒有改 —— 這是機器人的顯示名稱,跟套件名稱是兩件事,check_urdf 印出來的 `robot name is: CAR_ASSEMBLE_URDF` 還是這個名字,是預期行為。
- `config/joint_names_CAR_ASSEMBLE_URDF.yaml` 的檔名也沒動。

**已驗證**:改名後在乾淨的 ROS 2 workspace 重新 `colcon build`,這次**完全沒有 warning**(套件名稱不符命名慣例的 warning 也消失了);`ros2 launch car_assemble_description display.launch.py --show-args` 解析正常;檢查 install 目錄下的 urdf 檔案,確認 6 個 `package://car_assemble_description/meshes/...` 路徑都正確改過來、沒有遺漏。

`.claude/settings.local.json` 裡原本記錄舊路徑的 `check_urdf` 權限項目也一併更新成新路徑。
