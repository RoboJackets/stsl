FROM ros:foxy

WORKDIR /usr/src/workspace

RUN /bin/bash -c "set -ex; \
    apt-get update; \
    apt-get upgrade -y; \
    apt-get install --no-install-recommends -y python3-bloom fakeroot dpkg-dev debhelper"

COPY . /usr/src/workspace/src/stsl

RUN /bin/bash -c "set -ex; \
    source /opt/ros/foxy/setup.bash; \
    rosdep update --rosdistro=foxy; \
    rosdep install --from-paths . --ignore-src -y --skip-keys=\"libgpiod2 libgpiod-dev\"; \
    cd src/stsl/deployment; \
    ./generate_debs_internal.sh"
