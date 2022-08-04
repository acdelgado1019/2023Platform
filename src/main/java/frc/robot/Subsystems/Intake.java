package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.IntakeTOCom;


public class Intake extends SubsystemBase{

    private VictorSPX horizontalIntake;
    private CANSparkMax intakeLift;
    private RelativeEncoder intakeLiftEncoder;
    private VictorSPX trigger;

    private boolean pulsing = false;

    public Intake (int horIntake, int vertIntake, int inLift) {
        horizontalIntake = new VictorSPX(horIntake);
        trigger = new VictorSPX(vertIntake);
        intakeLift = new CANSparkMax(inLift, MotorType.kBrushless);
        intakeLiftEncoder = intakeLift.getEncoder();
    }

    public void setHorizontalIntake(double speed) {
        horizontalIntake.set(ControlMode.PercentOutput, speed);
    }

    public void setTrigger(double speed) {
        trigger.set(ControlMode.PercentOutput, speed);
    }

    public void setIntakeLift(double speed){
        intakeLift.set(-speed);
    }

    public void pulse(){
        if (pulsing == false){
            pulsing = true;
        }
        if (Timer.getFPGATimestamp() % 1 > 0.5){
            setTrigger(Constants.TRIGGER_SPEED);
        } else {setTrigger(0);}
        SmartDashboard.putBoolean("Firing", (Timer.getFPGATimestamp() % 1)<0.5);
    }

    public void stopPulse(){
        SmartDashboard.putBoolean("Firing", false);
        setTrigger(0);
        pulsing = false;
    }

    public void resetEncoder(){
        intakeLiftEncoder.setPosition(0.0);
    }

    @Override
    public void periodic(){
        setDefaultCommand(new IntakeTOCom());
    }
}