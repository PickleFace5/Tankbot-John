
from commands2 import Command

from pathplannerlib.auto import PathPlannerAuto

from wpilib import XboxController, SendableChooser
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from wpimath.kinematics import ChassisSpeeds

from constants import Constants
from subsystems.drivetrain import Drivetrain


class RobotContainer:
    """
    RobotContainer is where the structure of
    the robot is "declared" to the robot. 
    This includes subsystems, commands, button 
    mappings, etc.
    """
    
    # "Create" controller
    driver_controller = XboxController(Constants.Controller.k_driver_controller_port)
    
    # Initialize the drivetrain
    drivetrain = Drivetrain()
    
    def __init__(self) -> None:
        self._configureButtonBindings()
        
        # Setup snazzy lambda default command (since you're really not supposed to make your own commands via classes, according to the docs)
        self.drivetrain.setDefaultCommand(
            self.drivetrain.runOnce(lambda: self.drivetrain.drive(
               ChassisSpeeds(
                   -self.driver_controller.getLeftY() * Constants.Drivetrain.k_max_attainable_speed, # Speed forward and backward 
                   0, # Speed Left and right (0 because we can't strafe)
                   -self.driver_controller.getRightX() * Constants.Drivetrain.k_max_attainable_speed # Rotation speed
                ) 
            ))
        )
        
        # Create Auto Chooser
        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("Do Nothing", None)
        self.auto_chooser.addOption("Through And Through", PathPlannerAuto("ThroughAndThrough"))
        
        # Show Auto Chooser on the Dashboard
        Shuffleboard.getTab("Main").add("Auto Chooser", self.auto_chooser).withWidget(BuiltInWidgets.kComboBoxChooser)


    # The underscore tells people "don't use 
    # this function outside of this 
    # function, this is for this class to 
    # use only"
    def _configureButtonBindings(self) -> None:
        """
        This function is in charge of 
        configuring all controller bindings for
        teleop for all controllers.
        """
        pass # no buttons :(
            
    def get_selected_auto(self) -> PathPlannerAuto:
        """Returns the currently selected auto from the chooser."""
        return self.auto_chooser.getSelected()
