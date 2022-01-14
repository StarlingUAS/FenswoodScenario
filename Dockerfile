ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-iris-ap:${VERSION}}

WORKDIR /ros.env.d

RUN mkdir fenswood

COPY fenswood fenswood

CMD ["ros2", "launch", "/ros.env.d/fenswood/iris.launch.xml"]