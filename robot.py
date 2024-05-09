import commands2

from container import RobotContainer


class TankbotJohn(commands2.TimedCommandRobot):
    
    def robotInit(self) -> None:
        self.robot_container = RobotContainer()
        
    def teleopPeriodic(self) -> None:
        pass
    
    def autonomousInit(self) -> None:
        selected_auto = self.robot_container.get_selected_auto()
        
        if selected_auto is None:
            print("Doing nothing :(")
            return
        
        selected_auto.schedule()
        
    def autonomousPeriodic(self) -> None:
        pass
