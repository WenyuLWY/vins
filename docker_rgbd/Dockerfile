FROM nvidia/cuda:11.3.1-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /

#base
RUN apt-get update && apt-get install -y lsb-release curl wget vim git

#ros
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y ros-noetic-desktop-full && \
    apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    python3-catkin-tools && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc           

# ros workspace
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    bash -c "source /opt/ros/noetic/setup.bash && catkin build" && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# slam env
RUN apt-get update && \
    apt-get install -y libceres-dev 
RUN apt-get update && \
    apt-get install -y ros-noetic-rtabmap* 

# #pytorch
RUN apt-get update && apt-get install -y python3-pip 

RUN pip install torch==1.11.0+cu113 torchvision==0.12.0+cu113 torchaudio==0.11.0 --extra-index-url https://download.pytorch.org/whl/cu113 
 
    
# ADD requirements.txt /root  
# RUN pip install -r /root/requirements.txt 

RUN pip install pandas==0.25.3

RUN pip install cityscapesScripts==1.5.0

# RUN bash -c "source /root/miniconda3/etc/profile.d/conda.sh && \ 
#             conda env create -f /root/rgbd_segmentation.yaml"  


# slam env
# RUN apt-get update && \
#     apt-get install -y libceres-dev ros-noetic-rtabmap*


# start
RUN echo "#!/bin/bash" > /root/start.sh && \
    echo "cd /root/catkin_ws/src/1my_vins/ESANet" >> /root/start.sh && \
    echo "exec \"\$@\"" >> /root/start.sh && \
    chmod +x /root/start.sh
WORKDIR /root
ENTRYPOINT ["/root/start.sh"]

CMD ["/bin/bash"]

