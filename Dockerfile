FROM px4io/px4-dev-ros2-foxy:2022-08-12

RUN mkdir -p $HOME/my_ws

WORKDIR $HOME/my_ws

EXPOSE 14581 

# update the packages and installing deps
RUN sudo apt update -y && sudo apt upgrade -y
RUN pip3 install kconfiglib
RUN sudo apt install ros-foxy-gazebo-ros-pkgs -y

# installing fast-DNS 
RUN git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 /Fast-RTPS-Gen \
  && cd /Fast-RTPS-Gen/gradle/wrapper

RUN distributionUrl=https\://services.gradle.org/distributions/gradle-6.8.3-bin.zip
RUN cd /Fast-RTPS-Gen && ./gradlew assemble && sudo env "PATH=$PATH" ./gradlew install




