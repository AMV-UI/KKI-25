source install/local_setup.bash
exec ros2 run core_control movement_controller
exec ros2 run core_control movement_simulator
exec ros2 run core_control channel_simulator

