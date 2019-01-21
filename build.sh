echo "Configuring and building Thirdparty/DBoW2 ..."
BUILDTYPE=Release
cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILDTYPE
make -j4

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILDTYPE
make -j4
cd ..

echo "Converting vocabulary to binary"
./tools/bin_vocabulary
