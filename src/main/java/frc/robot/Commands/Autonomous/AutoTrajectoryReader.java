package frc.robot.Commands.Autonomous;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory; 
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class AutoTrajectoryReader {
    
    public static Trajectory generateTrajectoryFromFile(String file_path) {
        try {
            Path traj_path = Filesystem.getDeployDirectory().toPath().resolve(file_path);
            return TrajectoryUtil.fromPathweaverJson(traj_path);
        
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + file_path, ex.getStackTrace());
            return null;
        } 
        
    }
    
}
