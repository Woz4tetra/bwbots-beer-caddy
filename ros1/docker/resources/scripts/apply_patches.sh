#!/usr/bin/env bash
BASE_DIR=$(realpath "$(dirname $1)")

find ${BASE_DIR}/src -type f -name CMakeLists.txt -exec sed -i'' -e 's/Boost REQUIRED python37/Boost REQUIRED python3/g' {} +

sed -i -e 's/${G2O_INCREMENTAL_LIB}/#${G2O_INCREMENTAL_LIB}/g'  ${BASE_DIR}/src/teb_local_planner/cmake_modules/FindG2O.cmake

# create a patch file: https://stackoverflow.com/questions/6658313/how-can-i-generate-a-git-patch-for-a-specific-commit
# helpful forum post: https://stackoverflow.com/questions/4770177/git-patch-does-not-apply
cd ${BASE_DIR}/src/image_pipeline/
git apply /tmp/scripts/fix-image-pipeline.patch --reject --whitespace=fix 
