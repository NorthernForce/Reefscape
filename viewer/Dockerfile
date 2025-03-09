FROM python:3.9

WORKDIR /app
COPY . .
RUN apt-get update && apt-get upgrade
RUN apt-get install -y build-essential cmake git

# build librealsense
RUN git clone -v https://github.com/IntelRealSense/librealsense.git
WORKDIR /app/librealsense
RUN git checkout v2.56.3
RUN mkdir build
WORKDIR /app/librealsense/build
RUN cmake .. \
    -DCMAKE_BUILD_TYPE="Release" \
    -DBUILD_WITH_OPENMP=true \
    -DBUILD_PYTHON_BINDINGS=true \
    -DBUILD_EXAMPLES=false \
    -DBUILD_TOOLS=false
RUN make -j8
RUN make install
RUN export PYTHONPATH=$PYTHONPATH:/usr/local/lib
RUN echo "export PYTHONPATH=$PYTHONPATH:/usr/local/lib" > ~/.bashrc

# cameraserver ports (color + depth for debug)
EXPOSE 1181
EXPOSE 1182

WORKDIR /app
RUN ["python3.9", "./run.py"]