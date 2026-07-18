cmake_minimum_required(VERSION 3.8)

project(car_assemble_description)

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes rviz urdf)
	install(DIRECTORY ${dir}/
		DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_package()
