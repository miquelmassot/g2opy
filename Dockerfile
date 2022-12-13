FROM dorowu/ubuntu-desktop-lxde-vnc
LABEL maintainer="miquel.massot-campos@soton.ac.uk"

RUN wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
RUN apt-get update 
RUN apt-get install -y cmake git build-essential

RUN git clone https://github.com/libigl/eigen.git
RUN cd eigen && mkdir build && cd build && cmake .. && make -j4 && make install

RUN apt-get update && apt-get install -y libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /code/g2opy
COPY . /code/g2opy/
RUN cd /code/g2opy && mkdir build && cd build && cmake ../ && make -j4 && make install -j4
RUN ldconfig
