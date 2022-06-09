echo "Y" | pip uninstall gym_pybullet_drones
rm -rf dist/
poetry build 
pip install dist/gym_pybullet_drones-1.0.0-py3-none-any.whl
cd tests
python test_build.py
rm -rf results
cd ..