package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.IntakeTOCom;


public class Intake extends SubsystemBase{

    private VictorSPX horizontalIntake;
    private PWMSparkMax intakeLift;
    private Encoder intakeLiftEncoder = new Encoder(8,9);
    private VictorSPX trigger;
    public final PIDController Lift_controller = new PIDController(Constants.kIntakeLiftKp, 0, 0);

    private boolean pulsing = false;

    //Simulated hardware
    private static final DCMotor m_GearBox = DCMotor.getNEO(1);
    private EncoderSim intakeLiftEncoder_Sim = new EncoderSim(intakeLiftEncoder);

    //The intake can be modeled by an arm simulator - think arm on a pivot
    private final SingleJointedArmSim intakeLift_Sim =
      new SingleJointedArmSim(
        m_GearBox,
        Constants.m_IntakeLiftReduction,
        SingleJointedArmSim.estimateMOI(Constants.m_IntakeLiftLength, Constants.m_IntakeLiftMass),
        Constants.m_IntakeLiftLength,
        Units.degreesToRadians(-80),
        Units.degreesToRadians(0),
        Constants.m_IntakeLiftMass,
        false,
        null
    );

    //Graphic display in the simulator
    public final Mechanism2d intake_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d intakePivot = intake_mech2d.getRoot("IntakePivot", 30, 30);
    public final MechanismLigament2d intakeTower = intakePivot.append(new MechanismLigament2d("IntakeTower", 30, 180));
    public final MechanismLigament2d intake =
    intakePivot.append(new MechanismLigament2d(
        "Intake Lift", 
        30, 
        Units.radiansToDegrees(intakeLift_Sim.getAngleRads()), 
        6, 
        new Color8Bit(Color.kDarkRed)));

    //Intake Constructor
    public Intake (int horIntake, int vertIntake, int inLift) {
        horizontalIntake = new VictorSPX(horIntake);
        trigger = new VictorSPX(vertIntake);
        intakeLift = new PWMSparkMax(inLift);

        intakeLiftEncoder.setDistancePerPulse(Constants.kIntakeLiftEncoderDistPerPulse);
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our Rotator is doing
        // First, we set our "inputs" (voltages)
        SmartDashboard.putNumber("Output Lift",intakeLift.get() * RobotController.getBatteryVoltage());
        intakeLift_Sim.setInput(intakeLift.get() * RobotController.getBatteryVoltage());
        
        // Next, we update it. The standard loop time is 20ms.
        intakeLift_Sim.update(0.020);
        // Finally, we set our simulated encoder's readings and simulated battery voltage
        intakeLiftEncoder_Sim.setDistance(intakeLift_Sim.getAngleRads());
        SmartDashboard.putNumber("Lift Angle", intakeLift_Sim.getAngleRads()/Math.PI*180);

        // Update the Mechanism Rotator angle based on the simulated Rotator angle
        intake.setAngle(Units.radiansToDegrees(intakeLift_Sim.getAngleRads()));
    }

    public void setHorizontalIntake(double speed) {
        horizontalIntake.set(ControlMode.PercentOutput, speed);
    }

    public void setTrigger(double speed) {
        trigger.set(ControlMode.PercentOutput, speed);
    }

    public void setIntakeLift(double voltage){
        intakeLift.setVoltage(voltage);
    }

    //Pulses the trigger in half-second increments to allow for flywheel recovery
    public void pulse(){
        if (pulsing == false){
            pulsing = true;
        }
        if (Timer.getFPGATimestamp() % 1 < 0.5){
            setTrigger(Constants.TRIGGER_SPEED);
            Robot.ledStrip.solid(60);
        } else {
            setTrigger(0);
            Robot.ledStrip.solid(90);
        }
        SmartDashboard.putBoolean("Firing", (Timer.getFPGATimestamp() % 1)<0.5);
    }

    public void stopPulse(){
        SmartDashboard.putBoolean("Firing", false);
        setTrigger(0);
        pulsing = false;
    }

    public void resetEncoder(){
        intakeLiftEncoder.reset();
    }

    public double getEncoder(){
        return intakeLiftEncoder.getDistance();
    }

    //Every scheduler cycle, we pass our XBox controls so we can control the intake.
    @Override
    public void periodic(){
        setDefaultCommand(new IntakeTOCom());
    }
}