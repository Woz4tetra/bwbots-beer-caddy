BASE_DIR=$(realpath "$(dirname $0)")

echo "---"
echo "Installing dependencies via python pip"
echo "---"

cd ${BASE_DIR}/../../ros1/bwbots/bw_tools
pip3 install -r requirements.txt
python3 setup.py install --user
