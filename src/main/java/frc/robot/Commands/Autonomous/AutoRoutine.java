package frc.robot.Commands.Autonomous;

import frc.robot.Robot;

public class AutoRoutine {

    public static void runAutonomous(){
        Robot.ledStrip.stripeRB();
        AutoMethods.runTrajectory().schedule();
    }
}
