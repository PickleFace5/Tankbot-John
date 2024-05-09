from commands2 import Subsystem

from pathplannerlib.auto import AutoBuilder, ReplanningConfig

from phoenix6.hardware import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.configs.config_groups import *
from phoenix6.controls import *
from phoenix6.sim import ChassisReference

import math

import navx

from wpilib import Field2d, RobotBase, DriverStation
from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveKinematics, DifferentialDriveOdometry, DifferentialDriveWheelSpeeds # Differential Drive AKA Tank Drive AKA West Coast Drive AKA AndyMark Drive

from constants import Constants


class Drivetrain(Subsystem):
    """
    This is the tank drive drivetrain, comprised of 4 motors,
    2 on each side.
    
    This uses wpimath kinematics (like how our swerve uses it, but for tank drive)
    """
    
    # Initializing motors
    left_front_talon = TalonFX(Constants.CanIDs.k_drive_left_front)
    left_rear_talon = TalonFX(Constants.CanIDs.k_drive_left_rear)
    right_front_talon = TalonFX(Constants.CanIDs.k_drive_right_front)
    right_rear_talon = TalonFX(Constants.CanIDs.k_drive_right_rear)
    
    
    # Setting up normal config (config applied to left motors)
    normal_config = TalonFXConfiguration()
    
    # Stator current limit 
    # (max amps allowed to send to the motor at once, lower values lower acceleration)
    normal_config.current_limits.stator_current_limit_enable = Constants.Drivetrain.k_enable_stator_limit
    normal_config.current_limits.stator_current_limit = Constants.Drivetrain.k_stator_limit
    
    # Gear ratio (this tells the motor "you need to rotate x times to spin the wheel 360 degrees")
    normal_config.feedback.sensor_to_mechanism_ratio = Constants.Drivetrain.k_gear_ratio
    
    # Neutral Mode (coast = spin freely when not doing anything, brake = resist spinning when not doing anything)
    normal_config.motor_output.neutral_mode = Constants.Drivetrain.k_neutral_mode
    
    
    # Applying general config to all talon's on the drivetrain
    left_front_talon.configurator.apply(normal_config)
    left_rear_talon.configurator.apply(normal_config)
    
    # This is a neat python trick 
    # this is renaming the varible by setting it to another varible and deleting the old one
    inverted_config = normal_config
    del normal_config
    
    # Setting up the inverted config 
    # (note how this isn't a constant. This is because you would literally never not invert it)
    inverted_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
    
    # Applying inverted config
    right_front_talon.configurator.apply(inverted_config)
    right_rear_talon.configurator.apply(inverted_config)
    
    # Setting follower motors using the Follower control request
    left_rear_talon.set_control(Follower(Constants.CanIDs.k_drive_left_front, False))
    right_rear_talon.set_control(Follower(Constants.CanIDs.k_drive_right_front, False))
    
    # Create the control requests and set them to the front motors
    left_velocity_control = VelocityVoltage(0)
    right_velocity_control = VelocityVoltage(0)
    left_front_talon.set_control(left_velocity_control)
    right_front_talon.set_control(right_velocity_control)
    
    # Create sim states and configure them so we can drive around in simulation
    left_sim = left_front_talon.sim_state
    right_sim = right_front_talon.sim_state
    
    # Set orientation because it ignores the current invert value for some odd reason
    left_sim.orientation = ChassisReference.CounterClockwise_Positive
    right_sim.orientation = ChassisReference.Clockwise_Positive
    
    
    # Setting up a NavX Gyro (no Pidgeon? *megamind sad face*)
    nav_x = navx.AHRS.create_spi() # Initialize the NavX (yes, it's hideous, no, i don't know what AHRS nor spi is)
    nav_x.reset()
    
    # KINEMATICS TIME
    # (this is where hovering over functions and classes are your friend)
    
    # Luckily, kinematics is WAY easier for tank than swerve; all we need here is the track width
    kinematics = DifferentialDriveKinematics(Constants.Drivetrain.k_track_width)
    
    # ODOMETRY TIME
    
    # we have to reset the positions of the talons to make sure the odometry is correct on bootup
    # NOTE: Since both motors on each side work together, we're only going to read the FRONT talon sensors.
    # (there could be a way to "combine" the sensors, but that's outside my knowledge. You can mess around with it if you want.)
    left_front_talon.set_position(0)
    right_front_talon.set_position(0)
    
    odometry = DifferentialDriveOdometry(nav_x.getRotation2d(), 0, 0)
    
    # If we're in simulation, then we have to set the simulated robot angle to 0
    gyro_sim = 0
    
    
    def __init__(self) -> None:
        # Aaaaaand we're finally to the init! Woooooo!
        
        # Since we've already setup everything, we have to create the widgets for Elastic
        # Now, I'm going to use the ShuffleBoard API, so this is new stuff. We'll work on this more later into the offseason,
        
        # Creating the field widget
        self.field = Field2d()
        
        # Sending Widgets to dashboard
        Shuffleboard.getTab("Main").add("Field", self.field).withWidget(BuiltInWidgets.kField)
        
        # Configuring settings for PathPlanner
        AutoBuilder.configureRamsete(
            self.odometry.getPose, # Get's the position of the robot via odometry
            lambda pose: self.reset_pose(pose), # Function to reset the pose (used by PP at start of auto)
            self.get_robot_speed, # Tells PathPlanner how fast the robot is currently travelling.
            lambda chassis_speed: self.drive(chassis_speed), # Tells PathPlanner "use this function to drive the robot".
            ReplanningConfig(), # Default replanning config (tells PathPlanner what to do if we start too far from the start of a path.)
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed, # Tells PathPlanner to flip the path if we're on red alliance.
            self # Finally, tells PathPlanner that this subsystem is the drivetrain.
        )
        
    def get_yaw(self) -> Rotation2d:
        """Gets the rotation of the robot."""
        # If we're in simulation, then calculate how much the robot *should* be moving.
        # Otherwise, just read the current NavX heading.
        if RobotBase.isReal():
            angle = self.nav_x.getRotation2d()
        else:
            angle = Rotation2d.fromDegrees(self.gyro_sim)
            
        return angle
    
    def get_robot_speed(self) -> ChassisSpeeds:
        """Gets the current speed of the robot. Used by PathPlanner for maing sure we're path following correctly."""
        return self.kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(
            rot_to_meters(self.left_front_talon.get_velocity().value), # Speed of left side of robot (m/s)
            rot_to_meters(self.right_front_talon.get_velocity().value) # Speed of right side of robot (m/s)
        ))
        
    def periodic(self) -> None:
        # Yes, subsystems can have their own periodic function to update every tick.
        # This way we can update the odometry every "tick" to ensure it's mostly up to date (give or take 2ms)
        
        # Update the odometry...
        self.odometry.update(
            self.get_yaw(), 
            rot_to_meters(self.left_front_talon.get_position().value),
            rot_to_meters(self.right_front_talon.get_position().value))
        
        # ...then update the field pose
        self.field.setRobotPose(self.odometry.getPose())
        
    def simulationPeriodic(self) -> None:
        # Position encoders don't update in sim, 
        # so we need to read the current velocity and get an estimate of the distance moved each tick.
        left_front_displacement = self.left_front_talon.get_rotor_velocity().value * 0.02
        right_front_displacement = self.right_front_talon.get_rotor_velocity().value * 0.02
        
        # Update sim states
        self.left_sim.add_rotor_position(left_front_displacement)
        self.right_sim.add_rotor_position(right_front_displacement)
        
    def set_desired_speeds(self, speeds: DifferentialDriveWheelSpeeds) -> None:
        """Reads the given DifferentialDriveWheelSpeeds and sends the required velocity to the motors."""
        
        # Make sure no wheel is travelling faster than the max speed
        speeds.desaturate(Constants.Drivetrain.k_max_attainable_speed)
        
        # Get the speed that we need to pass to the motors
        left_rot_speed = meters_to_rots(speeds.left)
        right_rot_speed = meters_to_rots(speeds.right)
        
        # Update the control requests to the motors
        self.left_velocity_control.velocity = left_rot_speed
        self.right_velocity_control.velocity = right_rot_speed
        
        # Update Sim States
        self.left_sim.set_rotor_velocity(left_rot_speed * Constants.Drivetrain.k_gear_ratio)
        self.right_sim.set_rotor_velocity(right_rot_speed * Constants.Drivetrain.k_gear_ratio)
        
    def drive(self, chassis_speed: ChassisSpeeds) -> None:
        """Drives the robot at the given chassis speed."""
        
        # Remove the vy (because tank drives can't drive sideways)
        chassis_speed.vy = 0
        
        # Discretize the given speed (this accounts for translating and rotating in-between updates, so the motion is more smooth.)
        # This really only applies to swerve drive but it's still good to have.
        chassis_speed = ChassisSpeeds.discretize(chassis_speed, 0.02)
        
        # Convert to individual wheel (left and right) speeds
        wheel_speeds = self.kinematics.toWheelSpeeds(chassis_speed)
        
        # Pass the wheel speeds into the motors
        self.set_desired_speeds(wheel_speeds)
        
        # If we're in simulation, set the navX to what the angle should be
        if not RobotBase.isReal():
            self.gyro_sim += chassis_speed.omega_dps * 0.02
            
    def reset_pose(self, pose: Pose2d) -> None:
        """Reset the robot's position on the field"""
        self.odometry.resetPosition(
            self.get_yaw(),
            self.left_front_talon.get_position().value,
            self.right_front_talon.get_position().value,
            pose)
        
    
def rot_to_meters(rotations: float) -> float:
    """Converts rotations into meters."""
    
    wheel_circumference = math.pi * Constants.Drivetrain.k_wheel_size
    return rotations * wheel_circumference

def meters_to_rots(meters: float) -> float:
    """Converts meters into rotations."""
    
    wheel_circumference = math.pi * Constants.Drivetrain.k_wheel_size
    return meters / wheel_circumference
    