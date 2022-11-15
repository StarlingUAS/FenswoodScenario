ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:${VERSION}

COPY . /ros_ws/src/example_controller_python

RUN . /ros_ws/install/setup.sh \
    && colcon build

CMD [ "ros2", "launch", "example_controller_python", "fenswood_example.launch.py" ]
