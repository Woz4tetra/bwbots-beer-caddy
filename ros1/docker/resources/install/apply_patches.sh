#!/usr/bin/env bash
BASE_DIR=$(realpath "$(dirname $1)")

find ${BASE_DIR}/src -type f -name CMakeLists.txt -exec sed -i'' -e 's/Boost REQUIRED python37/Boost REQUIRED python3/g' {} +

# create a patch file: https://stackoverflow.com/questions/6658313/how-can-i-generate-a-git-patch-for-a-specific-commit
# helpful forum post: https://stackoverflow.com/questions/4770177/git-patch-does-not-apply
cd ${BASE_DIR}/src/image_pipeline/
git checkout -f
git apply /root/install/fix-image-pipeline.patch --reject --whitespace=fix 

cd ${BASE_DIR}/src/imu_tools/
git checkout -f
git apply /root/install/fix-imu-tools.patch --reject --whitespace=fix 

cd ${BASE_DIR}/src/geometry2/
git checkout -f
git apply /root/install/fix-geometry2.patch --reject --whitespace=fix 

cd ${BASE_DIR}/src/teb_local_planner
git checkout -f
git checkout 9d91cf2e0e00f09063e955c3652bcfad15d9d016
sed -i -e 's/${G2O_INCREMENTAL_LIB}/#${G2O_INCREMENTAL_LIB}/g' cmake_modules/FindG2O.cmake
