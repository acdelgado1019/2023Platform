package frc.robot.Subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
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
    private static CANSparkMax leftClimber0;
    private static CANSparkMax leftClimber1;
    private static PWMSparkMax leftClimberRotate;
    private static CANSparkMax rightClimber0;
    private static CANSparkMax rightClimber1;
    private static PWMSparkMax rightClimberRotate;
    private Encoder climberEncoderRight = new Encoder(4, 5);
    private Encoder climberEncoderLeft = new Encoder(8, 9);
    private Encoder rightRotateEncoder = new Encoder(10, 11);
    private Encoder leftRotateEncoder = new Encoder(6, 7);
    public final PIDController L_controller = new PIDController(Constants.kLArmKp, 0, 0);
    public final PIDController R_controller = new PIDController(Constants.kRArmKp, 0, 0);

    private static final DCMotor m_LGearBox = DCMotor.getVex775Pro(2);
    private static final DCMotor m_RGearBox = DCMotor.getVex775Pro(2);
    private final SingleJointedArmSim leftClimberRotate_Sim =
      new SingleJointedArmSim(
        m_LGearBox,
        Constants.m_armReduction,
        SingleJointedArmSim.estimateMOI(Constants.m_armLength, Constants.m_armMass),
        Constants.m_armLength,
        Units.degreesToRadians(-26),
        Units.degreesToRadians(0),
        Constants.m_armMass,
        false,
        null
    );
    private final SingleJointedArmSim rightClimberRotate_Sim =
    new SingleJointedArmSim(
        m_RGearBox,
        Constants.m_armReduction,
        SingleJointedArmSim.estimateMOI(Constants.m_armLength, Constants.m_armMass),
        Constants.m_armLength,
        Units.degreesToRadians(0),
        Units.degreesToRadians(26),
        Constants.m_armMass,
        false,
        null
    );

    private EncoderSim leftRotateEncoder_Sim = new EncoderSim(leftRotateEncoder);
    private EncoderSim rightRotateEncoder_Sim = new EncoderSim(rightRotateEncoder);
    private EncoderSim climberEncoderRight_Sim = new EncoderSim(climberEncoderRight);
    private EncoderSim climberEncoderLeft_Sim = new EncoderSim(climberEncoderLeft);

    public final Mechanism2d L_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d L_armPivot = L_mech2d.getRoot("LeftArmPivot", 30, 30);
    public final MechanismLigament2d L_armTower = L_armPivot.append(new MechanismLigament2d("LeftArmTower", 30, 180));
    private final MechanismLigament2d L_arm =
        L_armPivot.append(new MechanismLigament2d("Left Rotator", 30, Units.radiansToDegrees(leftClimberRotate_Sim.getAngleRads()), 6, new Color8Bit(Color.kDarkRed)));

    public final Mechanism2d R_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d R_armPivot = R_mech2d.getRoot("RightArmPivot", 30, 30);
    public final MechanismLigament2d R_armTower = R_armPivot.append(new MechanismLigament2d("RightArmTower", 30, 0));
    private final MechanismLigament2d R_arm =
        R_armPivot.append(new MechanismLigament2d("Right Rotator", 30, Units.radiansToDegrees(rightClimberRotate_Sim.getAngleRads()), 6, new Color8Bit(Color.kRed)));
    
    private boolean climbMode = false;


    public Climbers(int climberL0, int climberL1, int climberLR, int climberR0, int climberR1, int climberRR) {
        leftClimber0 = new CANSparkMax(climberL0, MotorType.kBrushless);
        leftClimber1 = new CANSparkMax(climberL1, MotorType.kBrushless);
        leftClimberRotate = new PWMSparkMax(climberLR);
        rightClimber0 = new CANSparkMax(climberR0, MotorType.kBrushless);
        rightClimber1 = new CANSparkMax(climberR1, MotorType.kBrushless);
        rightClimberRotate = new PWMSparkMax(climberRR);

        // leftClimberRotate.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        // leftClimberRotate.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        // rightClimberRotate.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        // rightClimberRotate.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);

        leftClimber0.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leftClimber1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        rightClimber0.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        rightClimber1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leftClimber0.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        leftClimber1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        rightClimber0.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        rightClimber1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        // leftClimberRotate.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) -4);
        // rightClimberRotate.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) 4);
        // leftClimberRotate.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) 0);
        // rightClimberRotate.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);

        leftClimber0.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 270);
        leftClimber0.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 2);
        rightClimber0.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 270);
        rightClimber0.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 2);
        leftClimber1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 270);
        leftClimber1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 2);
        rightClimber1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 270);
        rightClimber1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 2);

        climberEncoderRight.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
        climberEncoderLeft.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
        rightRotateEncoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
        leftRotateEncoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);    
    }


    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        SmartDashboard.putNumber("Output L",leftClimberRotate.get() * RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Output R",rightClimberRotate.get() * RobotController.getBatteryVoltage());
        rightClimberRotate_Sim.setInput(rightClimberRotate.get() * RobotController.getBatteryVoltage());
        leftClimberRotate_Sim.setInput(leftClimberRotate.get() * RobotController.getBatteryVoltage());
        // Next, we update it. The standard loop time is 20ms.
        rightClimberRotate_Sim.update(0.020);
        leftClimberRotate_Sim.update(0.020);
        // Finally, we set our simulated encoder's readings and simulated battery voltage
        rightRotateEncoder_Sim.setDistance(rightClimberRotate_Sim.getAngleRads());
        leftRotateEncoder_Sim.setDistance(leftClimberRotate_Sim.getAngleRads());
        SmartDashboard.putNumber("LAngle", leftClimberRotate_Sim.getAngleRads()/Math.PI*180);
        SmartDashboard.putNumber("RAngle", rightClimberRotate_Sim.getAngleRads()/Math.PI*180);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(leftClimberRotate_Sim.getCurrentDrawAmps()));
        // Update the Mechanism Arm angle based on the simulated arm angle
        R_arm.setAngle(Units.radiansToDegrees(rightClimberRotate_Sim.getAngleRads()));
        L_arm.setAngle(Units.radiansToDegrees(leftClimberRotate_Sim.getAngleRads()));
    }

    public HashMap<String, Double> getEncoderValues() {
        HashMap<String, Double> encoderMap =  new HashMap<String, Double>();
        encoderMap.put("leftClimberEncoder", climberEncoderLeft.getDistance());
        encoderMap.put("rightClimberEncoder", climberEncoderRight.getDistance());
        encoderMap.put("leftRotationEncoder", -leftRotateEncoder.getDistance());
        encoderMap.put("rightRotationEncoder", rightRotateEncoder.getDistance());
        return encoderMap;
    }

    public void setLeftClimber(double speed){
        leftClimber0.set(-speed);
        leftClimber1.set(-speed);
    }

    public void setRightClimber(double speed){
        rightClimber0.set(-speed);
        rightClimber1.set(-speed);
    }

    public void setLeftClimberRotation(double voltage)
    {
        leftClimberRotate.setVoltage(voltage);
        SmartDashboard.putNumber("LMGET",leftClimberRotate.get());
    }

    public void setRightClimberRotation(double voltage)
    {
        rightClimberRotate.setVoltage(voltage);
        SmartDashboard.putNumber("RMGET",rightClimberRotate.get());
    }

    public void setClimbMode(){
        climbMode = !climbMode;
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
        SmartDashboard.putNumber("Right Climber Position ", climberEncoderRight.getDistance());
        SmartDashboard.putNumber("Left Climber Position ", climberEncoderLeft.getDistance());
        SmartDashboard.putNumber("Left Rotator Position ", -leftRotateEncoder.getDistance());
        SmartDashboard.putNumber("Right Rotator Position ", rightRotateEncoder.getDistance());
    }

    public double getRightEncoder(){
        return rightRotateEncoder.getDistance();
    }

    public double getLeftEncoder(){
        return leftRotateEncoder.getDistance();
    }

    @Override
    public void periodic(){
        setDefaultCommand(new ClimbersTOCom());
    }
}