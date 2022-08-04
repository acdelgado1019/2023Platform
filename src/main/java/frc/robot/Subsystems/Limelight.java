package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.LimelightTOCom;
/* LimeLight specific Imports*/
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

    public double distance;


    // Creates a new LimeLight.
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry ty;

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
    public void getRange(){
        SmartDashboard.putBoolean("IN RANGE", Math.abs(tx.getDouble(0.0))<15 && tx.getDouble(0.0)!= 0.0 ? true : false);
    }

    public double getDistance(){
        distance = (Constants.goalHeight - Constants.camHeight)/Math.tan((Constants.camAngle + getTY()) * (Math.PI / 180.0))/12;
        SmartDashboard.putNumber("Distance", distance);
        return distance;
    }

    public void switchCameraMode(){
        table.getEntry("camMode").setNumber(table.getEntry("camMode").getDouble(0.0) == 0 ? 1 : 0);
        table.getEntry("ledMode").setNumber(table.getEntry("ledMode").getDouble(0.0) == 0 ? 3 : 0);
    }

    @Override
    public void periodic() {
        setDefaultCommand(new LimelightTOCom());
        // This method will be called once per scheduler run
    }
}