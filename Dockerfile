ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-iris-ap:latest

WORKDIR /ros.env.d


RUN mkdir fenswood
COPY fenswood fenswood

# Build gimbal plugin with ROS2 on path and add plugin to path
COPY simulation/attach_vehicles_plugin /ros_ws/src/attach_vehicles_plugin
RUN cd /ros_ws \
    && . /opt/ros/foxy/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && colcon build --cmake-force-configure --packages-select attach_vehicles_plugin\
    && rm -r build \
    && echo 'export GAZEBO_PLUGIN_PATH="/ros_ws/install/attach_vehicles_plugin/lib:${GAZEBO_PLUGIN_PATH}"' >> /ros.env.d/fenswood/setup.bash


CMD ["ros2", "launch", "/ros.env.d/fenswood/iris.launch.xml"]