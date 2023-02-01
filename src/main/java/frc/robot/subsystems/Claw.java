package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.helpers.PneumaticsSystem;
import frc.maps.RobotMap;

public class Claw extends SubsystemBase {
    //Extends and retracts system
    PneumaticsSystem pneumatics = new PneumaticsSystem(PneumaticsModuleType.CTREPCM, RobotMap.CLAW_ONE, RobotMap.CLAW_TWO);
    //Intake and outtake
    CCSparkMax claw = new CCSparkMax("Claw", "claw", RobotMap.CLAW, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_LEFT_REVERSE);
    public void toggleArm() {
        pneumatics.toggle();
    }
    public void moveClaw(double speed) {
        claw.set(speed);
    }
    public Command toggleArmCommand() {
        return new InstantCommand(() -> toggleArm());
    }
    //rolling
    public Command moveClawCommand(double speed) {
        return new InstantCommand(() -> moveClaw(speed));
    }
}