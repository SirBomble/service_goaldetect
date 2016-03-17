#!/bin/bash
# Hey, its not _my_ fault I didnt have time to use CMake...
g++ -g astra-goaldetect.cpp -I/opt/OpenNI-Linux-Arm-2.2/Include -L/opt/OpenNI-Linux-Arm-2.2/Redist -lOpenNI2 -I/usr/include/opencv  /usr/lib/libopencv_calib3d.so /usr/lib/libopencv_contrib.so /usr/lib/libopencv_core.so /usr/lib/libopencv_features2d.so /usr/lib/libopencv_flann.so /usr/lib/libopencv_gpu.so /usr/lib/libopencv_highgui.so /usr/lib/libopencv_imgproc.so /usr/lib/libopencv_legacy.so /usr/lib/libopencv_ml.so /usr/lib/libopencv_objdetect.so /usr/lib/libopencv_photo.so /usr/lib/libopencv_stitching.so /usr/lib/libopencv_superres.so /usr/lib/libopencv_ts.a /usr/lib/libopencv_video.so /usr/lib/libopencv_videostab.so /usr/lib/libopencv_esm_panorama.so /usr/lib/libopencv_facedetect.so /usr/lib/libopencv_imuvstab.so /usr/lib/libopencv_tegra.so /usr/lib/libopencv_vstab.so -lrt -lpthread -lm -ldl -o astra-goaldetect
export LD_LIBRARY_PATH=/opt/OpenNI-Linux-Arm-2.2/Redist/
if [[ $1 == "run" ]]; then
  ./astra-goaldetect
fi
