echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.1
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.1
make -j

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.1
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.1
make -j10
