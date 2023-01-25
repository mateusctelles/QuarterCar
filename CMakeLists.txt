project(QuarterCar)

add_subdirectory(matplotplusplus)

add_executable(${PROJECT_NAME} main.cpp Car.cpp Road.cpp Simulation.cpp Spring.cpp Damper.cpp ModelBuilder.cpp)
#add_executable("testematplotpp1" testematplotpp.cpp)
#add_executable("sciplot" sciplot.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC matplot)
#target_link_libraries("sciplot" PUBLIC sciplot)
#target_link_libraries("testematplotpp1" PUBLIC matplot)