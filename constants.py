from phoenix6.configs.config_groups import NeutralModeValue

class Constants:
    
    class CanIDs:
        k_drive_left_front = 1
        k_drive_left_rear = 2
        k_drive_right_front = 3
        k_drive_right_rear = 4
    
    class Drivetrain:
        # Physical properties
        k_gear_ratio = 5 # Placeholder value, this means that the rotor needs to spin 5 times to rotate the wheel 360 degrees once
        k_track_width = 0.69 # Distance between the left and right wheels in meters
        k_wheel_size = 0.1 # Wheel size in meters
        k_max_attainable_speed = 3 # Max speed (in m/s) that the wheels can travel at reliably. I normally go for 75%-80% max speed.
        
        k_enable_stator_limit = True
        k_stator_limit = 40 # I believe this is the default, but it's set as a constant here to quickly change.
        
        k_neutral_mode = NeutralModeValue.BRAKE
    
    class Controller:
        k_driver_controller_port = 0
