package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.LimelightTOCom;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

    public double distance;

    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry ty;

    //The following five methods retrieve and make data available from the Limelight Network Table
    public void updateData() {
        // update table, then update from updated table
        table = NetworkTableInstance.getDefault().getTable("limelight");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
    }

    public double getTX() {
        updateData();
        return tx.getDouble(0.0);
    }

    public double getTY() {
        updateData();
        return ty.getDouble(0.0);
    }

    public double getTA() {
        updateData();
        return ta.getDouble(0.0);
    }

    public double getTV() {
        updateData();
        return tv.getDouble(0.0);
    }

    //Detects if the robot is in range left to right to make a shot
    public void getRange(){
        SmartDashboard.putBoolean("IN RANGE", Math.abs(tx.getDouble(0.0))<15 && tx.getDouble(0.0)!= 0.0 ? true : false);
    }

    //Calculates the distance away from the hub across the ground
    public double getDistance(){
        distance = (Constants.goalHeight - Constants.camHeight)/Math.tan((Constants.camAngle + getTY()) * (Math.PI / 180.0))/12;
        SmartDashboard.putNumber("Distance", distance);
        return distance;
    }

    //Switches from thresholding mode to pure camera mode
    public void switchCameraMode(){
        table.getEntry("camMode").setNumber(table.getEntry("camMode").getDouble(0.0) == 0 ? 1 : 0);
        table.getEntry("ledMode").setNumber(table.getEntry("ledMode").getDouble(0.0) == 0 ? 3 : 0);
    }

    //Every scheduler cycle, we pass our XBox controls so we can control the limelight.
    @Override
    public void periodic() {
        setDefaultCommand(new LimelightTOCom());
        // This method will be called once per scheduler run
    }
}