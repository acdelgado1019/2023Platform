package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
import frc.robot.Commands.ClimbersTOCom;

public class Climbers extends SubsystemBase{
    private static PWMSparkMax leftClimber0;
    private static PWMSparkMax leftClimber1;
    private static PWMSparkMax leftClimberRotate;
    private static PWMSparkMax rightClimber0;
    private static PWMSparkMax rightClimber1;
    private static PWMSparkMax rightClimberRotate;
    private Encoder climberEncoderRight = new Encoder(13, 14);
    private Encoder climberEncoderLeft = new Encoder(15, 16);
    private Encoder rightRotateEncoder = new Encoder(4, 5);
    private Encoder leftRotateEncoder = new Encoder(6, 7);
    public final PIDController L_controller = new PIDController(Constants.kLRotatorKp, 0, 0);
    public final PIDController R_controller = new PIDController(Constants.kRRotatorKp, 0, 0);

    public enum AutoClimbStep{
        MANUAL_MODE,
        MID_BAR_RETRACT,
        MID_BAR_RELEASE,
        HIGH_BAR_EXTEND,
        HIGH_BAR_RETRACT,
        HIGH_BAR_RELEASE,
        TRAVERSAL_BAR_EXTEND,
        TRAVERSAL_BAR_RETRACT,
        CLIMB_COMPLETE
    }

    public AutoClimbStep autoClimbStep;

    private static final DCMotor m_LGearBox = DCMotor.getNEO(1);
    private static final DCMotor m_RGearBox = DCMotor.getNEO(1);
    private final SingleJointedArmSim leftClimberRotate_Sim =
      new SingleJointedArmSim(
        m_LGearBox,
        Constants.m_RotatorReduction,
        SingleJointedArmSim.estimateMOI(Constants.m_RotatorLength, Constants.m_RotatorMass),
        Constants.m_RotatorLength,
        Units.degreesToRadians(-26),
        Units.degreesToRadians(0),
        Constants.m_RotatorMass,
        false,
        null
    );
    private final SingleJointedArmSim rightClimberRotate_Sim =
    new SingleJointedArmSim(
        m_RGearBox,
        Constants.m_RotatorReduction,
        SingleJointedArmSim.estimateMOI(Constants.m_RotatorLength, Constants.m_RotatorMass),
        Constants.m_RotatorLength,
        Units.degreesToRadians(0),
        Units.degreesToRadians(26),
        Constants.m_RotatorMass,
        false,
        null
    );

    private final DCMotorSim rightClimber = new DCMotorSim(DCMotor.getNEO(2),1, 0.000666);
    private final DCMotorSim leftClimber = new DCMotorSim(DCMotor.getNEO(2),1, 0.000666);

    public final Mechanism2d L_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d L_RotatorPivot = L_mech2d.getRoot("LeftRotatorPivot", 30, 30);
    public final MechanismLigament2d L_RotatorTower = L_RotatorPivot.append(new MechanismLigament2d("LeftRotatorTower", 30, 180));
    private final MechanismLigament2d L_Rotator =
        L_RotatorPivot.append(new MechanismLigament2d("Left Rotator", 30, Units.radiansToDegrees(leftClimberRotate_Sim.getAngleRads()), 6, new Color8Bit(Color.kDarkRed)));

    public final Mechanism2d R_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d R_RotatorPivot = R_mech2d.getRoot("RightRotatorPivot", 30, 30);
    public final MechanismLigament2d R_RotatorTower = R_RotatorPivot.append(new MechanismLigament2d("RightRotatorTower", 30, 180));
    private final MechanismLigament2d R_Rotator =
        R_RotatorPivot.append(new MechanismLigament2d("Right Rotator", 30, Units.radiansToDegrees(rightClimberRotate_Sim.getAngleRads()), 6, new Color8Bit(Color.kRed)));
    
    private EncoderSim leftRotateEncoder_Sim = new EncoderSim(leftRotateEncoder);
    private EncoderSim rightRotateEncoder_Sim = new EncoderSim(rightRotateEncoder);
    private EncoderSim climberEncoderRight_Sim = new EncoderSim(climberEncoderRight);
    private EncoderSim climberEncoderLeft_Sim = new EncoderSim(climberEncoderLeft);
    
    private boolean climbMode = false;


    public Climbers(int climberL0, int climberL1, int climberLR, int climberR0, int climberR1, int climberRR) {
        leftClimber0 = new PWMSparkMax(climberL0);
        leftClimber1 = new PWMSparkMax(climberL1);
        leftClimberRotate = new PWMSparkMax(climberLR);
        rightClimber0 = new PWMSparkMax(climberR0);
        rightClimber1 = new PWMSparkMax(climberR1);
        rightClimberRotate = new PWMSparkMax(climberRR);

        climberEncoderRight.setDistancePerPulse(Constants.kClimberEncoderDistPerPulse);
        climberEncoderLeft.setDistancePerPulse(Constants.kClimberEncoderDistPerPulse);
        rightRotateEncoder.setDistancePerPulse(Constants.kRotatorEncoderDistPerPulse);
        leftRotateEncoder.setDistancePerPulse(Constants.kRotatorEncoderDistPerPulse);    
    }


    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our Rotator is doing
        // First, we set our "inputs" (voltages)
        rightClimberRotate_Sim.setInput(rightClimberRotate.get() * RobotController.getBatteryVoltage());
        leftClimberRotate_Sim.setInput(leftClimberRotate.get() * RobotController.getBatteryVoltage());
        rightClimber.setInput(rightClimber0.get() * RobotController.getBatteryVoltage());
        leftClimber.setInput(leftClimber0.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        rightClimberRotate_Sim.update(0.020);
        leftClimberRotate_Sim.update(0.020);
        rightClimber.update(0.020);
        leftClimber.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        rightRotateEncoder_Sim.setDistance(rightClimberRotate_Sim.getAngleRads());
        leftRotateEncoder_Sim.setDistance(leftClimberRotate_Sim.getAngleRads());
        climberEncoderLeft_Sim.setDistance(leftClimber.getAngularPositionRotations());
        climberEncoderRight_Sim.setDistance(rightClimber.getAngularPositionRotations());

        R_Rotator.setAngle(Units.radiansToDegrees(rightClimberRotate_Sim.getAngleRads()));
        L_Rotator.setAngle(Units.radiansToDegrees(leftClimberRotate_Sim.getAngleRads()));
    }

    public void setLeftClimber(double speed){
        leftClimber0.set(speed);
        leftClimber1.set(speed);
    }

    public void setRightClimber(double speed){
        rightClimber0.set(speed);
        rightClimber1.set(speed);
    }

    public void setClimberRotation(double setpoint){
        var lPIDOutput = L_controller.calculate(getLeftRotEncoder(), -setpoint);
        setLeftClimberRotation(lPIDOutput);
        var rPIDOutput = R_controller.calculate(getRightRotEncoder(), setpoint);
        setRightClimberRotation(rPIDOutput);
    }

    public void setLeftClimberRotation(double voltage)
    {
        leftClimberRotate.setVoltage(voltage);
    }

    public void setRightClimberRotation(double voltage)
    {
        rightClimberRotate.setVoltage(voltage);
    }

    public void setClimbMode(){
        climbMode = true;
    }

    public void resetClimbMode(){
        climbMode = false;
    }

    public boolean getClimbMode(){
        return climbMode;
    }

    public void resetEncoders(){
        climberEncoderLeft.reset();
        climberEncoderRight.reset();
        rightRotateEncoder.reset();
        leftRotateEncoder.reset();
    }

    public void updateDashboard()
    {
        SmartDashboard.putNumber("Right Climber Position ", -getRightClimbEncoder());
        SmartDashboard.putNumber("Left Climber Position ", -getLeftClimbEncoder());
        SmartDashboard.putNumber("Right Rotator Position ", Units.radiansToDegrees(getRightRotEncoder()));
        SmartDashboard.putNumber("Left Rotator Position ", -Units.radiansToDegrees(getLeftRotEncoder()));
        SmartDashboard.putBoolean("Climber Mode", climbMode);
    }

    public double getRightClimbEncoder(){
        return climberEncoderRight.getDistance();
    }

    public double getLeftClimbEncoder(){
        return climberEncoderLeft.getDistance();
    }
    
    public double getRightRotEncoder(){
        return rightRotateEncoder.getDistance();
    }

    public double getLeftRotEncoder(){
        return leftRotateEncoder.getDistance();
    }

    @Override
    public void periodic(){
        setDefaultCommand(new ClimbersTOCom());
    }
}