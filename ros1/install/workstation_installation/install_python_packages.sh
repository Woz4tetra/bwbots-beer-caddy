BASE_DIR=$(realpath "$(dirname $0)")

echo "---"
echo "Installing dependencies via python pip"
echo "---"

cd ${BASE_DIR}/../../bwbots/bw_tools
sudo -H python3 -m pip install -r requirements.txt
python3 setup.py install --user
