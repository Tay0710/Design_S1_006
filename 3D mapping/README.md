This is the code for the 3D mapping using the position calculated in the pipeline.
To update this for new samples:
    - Run x0_position_pipeline.py to update all position data.
    - Update the ToF link in the main function in tof_map.py

To run:
    - python tof_map.py


Note: to get open3d working must use python 11 or below....