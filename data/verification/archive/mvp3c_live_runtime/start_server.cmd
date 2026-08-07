@echo off
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
E:\Anaconda\Scripts\conda.exe run --no-capture-output -p E:\Anaconda\envs_dirs\lerobot python scripts\mvp_so101_server.py --config config\mvp_hardware.json --read-only
