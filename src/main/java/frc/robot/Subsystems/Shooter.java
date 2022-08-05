package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Commands.ShooterTOCom;

public class Shooter extends SubsystemBase{
    private PWMSparkMax shooter;
    private static Encoder shooterEncoder = new Encoder(17,18);
    private final DCMotorSim shooterSim = new DCMotorSim(DCMotor.getNEO(1),1, 0.000666);
    private EncoderSim shooterEncoder_Sim = new EncoderSim(shooterEncoder);

    private double currDist = 0.0;
    private double prevDist = 0.0;
    private double currTime = 0.0;
    private double prevTime = Timer.getFPGATimestamp();

    public Shooter(int shoot){
        shooter = new PWMSparkMax(shoot);
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our Rotator is doing
        // First, we set our "inputs" (voltages)
        shooterSim.setInput(shooter.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        shooterSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        shooterEncoder_Sim.setDistance(shooterSim.getAngularPositionRad());
    }

    public void setShooterMotor(double speed){
        shooter.setVoltage(-speed);
    }

    public double shooterSpeedAdjust(double distance){
        double outputVoltage = (4-Math.sqrt(16+0.8*(-3.5-distance)))/0.4;
        return outputVoltage;
    }

    public void limelightTrack()
    {
        double degOff = Robot.limelight.getTX();
        if(Math.abs(degOff) > 1 && Robot.limelight.getTV() != 0)
        {
            double speed = .15 * degOff/(Math.abs(degOff));
            Robot.drivetrain.setLeftDrivetrain(-speed);
            Robot.drivetrain.setRightDrivetrain(speed);
            degOff = Robot.limelight.getTX();
        }
    }

    public void updateDashboard(){
        currDist = shooterEncoder.getDistance()/(2 * Math.PI);
        currTime = Timer.getFPGATimestamp();
        if (currTime - prevTime > 0.5){
            double rate = 120*(prevDist - currDist)/(currTime - prevTime);
            SmartDashboard.putNumber("Shooter Speed ", rate);
            prevTime = currTime;
            prevDist = currDist;
        }
    } 

    @Override
    public void periodic(){
        setDefaultCommand(new ShooterTOCom());
    }
}