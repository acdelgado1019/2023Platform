package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.ShooterTOCom;

public class Shooter extends SubsystemBase{
    //Use PWMSparkMax, CANSparkMax not Sim supported well right now
    private PWMSparkMax shooter;
    private VictorSPX trigger;

    public boolean autoShoot = true;

    //Simulator instance of the hardware above
    private final FlywheelSim shooterSim = new FlywheelSim(DCMotor.getNEO(1),1, 0.000666);

    //Shooter constructor - creates a shooter in robot memory
    public Shooter(int shoot, int trig){
        shooter = new PWMSparkMax(shoot);
        trigger = new VictorSPX(trig);
    }

    //While the simulation is running, this runs every clock cycle
    @Override
    public void simulationPeriodic() {
        //Set Simulator
        shooterSim.setInput(shooter.get() * RobotController.getBatteryVoltage());

        //Update Sim -> Advance clock 20 ms
        shooterSim.update(0.020);
    }

    //Runs the motor with a given voltage - We want the motor to spin counter-clockwise, so the voltage needs to be negative
    public void setShooterMotor(double voltage){
        shooter.setVoltage(-voltage);
    }

    //Given a distance calculated from the limelight, this method adjusts the flywheel speed based on a function unique to Bouree
    public double shooterSpeedAdjust(double distance){
        double outputVoltage = (4-Math.sqrt(16+0.8*(-3.5-distance)))/0.4;
        if (Double.isNaN(outputVoltage)){
            outputVoltage = Constants.SHOOTER_IDLE_SPEED;
            Robot.ledStrip.solid(90);
        }
        return outputVoltage;
    }

    public void setTrigger(double speed, boolean firing) {
        trigger.set(ControlMode.PercentOutput, speed);
        if (speed < 0){
            Robot.ledStrip.solid(60);
        } else if (speed > 0){
            Robot.ledStrip.solid(30);
        }
        SmartDashboard.putBoolean("Firing", firing);
    }

    public boolean getAutoShootEnable(){
        return autoShoot;
    }

    public void changeAutoShootState(){
        autoShoot = !autoShoot;
    }

    //Updates the shuffle/smart dashboards with flywheel spin rate in RPM
    public void updateDashboard(){
            SmartDashboard.putNumber("Shooter Speed ", -shooterSim.getAngularVelocityRPM());
            SmartDashboard.putBoolean("Auto Shoot", getAutoShootEnable());
    } 

    //Every scheduler cycle, we pass our XBox controls so we can control the shooter.
    @Override
    public void periodic(){
        setDefaultCommand(new ShooterTOCom());
    }
}