FROM jdlangs/ros-vtk8.2-pcl1.9
LABEL maintainer="wang hao"

COPY . /home/waha/workspace/grasp_data_generator

# 删除ros相关
RUN rm /etc/apt/sources.list.d/ros1-latest.list

# 安装一些常用命令
RUN apt-get update
RUN apt-get install -y --no-install-recommends apt-utils vim zip unzip
RUN rm -rf /var/lib/apt/lists/*
RUN cd /home && mkdir -p waha/workspace/data

WORKDIR /home/waha/workspace