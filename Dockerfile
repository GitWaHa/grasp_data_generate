FROM jdlangs/ros-vtk8.2-pcl1.9
LABEL maintainer="wang hao"

COPY . /home/waha/workspace/grasp_data_generator

RUN apt-get update
RUN apt-get install -y --no-install-recommends vim zip unzip \
    rm -rf /var/lib/apt/lists/* &&\
    cd /home && mkdir -p waha/workspace/data

WORKDIR /home/waha/workspace