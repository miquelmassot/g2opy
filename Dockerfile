FROM dorowu/ubuntu-desktop-lxde-vnc
LABEL maintainer="miquel.massot-campos@soton.ac.uk"

RUN wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y --no-install-recommends cmake git build-essential \
    libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
RUN apt-get install -y --no-install-recommends python3-distutils libeigen3-dev python3-dev python3-pip python3-setuptools \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /code/g2opy
COPY . /code/g2opy/
RUN cd /code/g2opy \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install -j4 \
    && cd .. && python setup.py install
RUN pip install notebook numpy scipy matplotlib
RUN ldconfig
