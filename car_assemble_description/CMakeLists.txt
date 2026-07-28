cmake_minimum_required(VERSION 3.8)

project(car_assemble_description)

find_package(ament_cmake REQUIRED)

foreach(dir launch meshes rviz urdf)
	install(DIRECTORY ${dir}/
		DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

# 參數檔集中放在 repo 根目錄的 config/,不在 package 裡面(2026-07-28)。
# 為什麼:所有「會現場調整的東西」希望在同一個地方 —— nav2 參數、slam 參數,
# 和 maps/ 一樣都是操作者會邊測邊改的檔案,散在 package 深處很難找。
#
# 這裡從 ../config/ 安裝進 package share,所以 launch 檔照樣用
# get_package_share_directory('car_assemble_description') + 'config' 找得到,
# 不需要知道 repo 在哪。搭配 colcon --symlink-install 時安裝出來的是 symlink,
# 所以改 config/*.yaml 之後不用重 build(和改 launch/、rviz/ 一樣)。
#
# 代價:這個 package 不能單獨被複製到別的 workspace(少了 ../config 就 configure 失敗)。
# 這個 repo 本身就是部署單位(整包 symlink 進 ~/ros2_ws/src),所以可以接受。
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../config/
	DESTINATION share/${PROJECT_NAME}/config)

ament_package()
