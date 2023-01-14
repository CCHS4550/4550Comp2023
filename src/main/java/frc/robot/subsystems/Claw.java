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
    PneumaticsSystem pneumatics = new PneumaticsSystem(PneumaticsModuleType.CTREPCM, RobotMap.CLAW_ONE, RobotMap.CLAW_TWO);
    CCSparkMax claw = new CCSparkMax("Claw", "claw", RobotMap.CLAW, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_LEFT_REVERSE, true);
    public void toggleClaw() {
        pneumatics.toggle();
    }
    public void moveClaw(double speed) {
        claw.set(speed);
    }
    public Command toggleClawCommand() {
        return new InstantCommand(() -> toggleClaw());
    }
    public Command moveClawCommand(double speed) {
        return new InstantCommand(() -> moveClaw(speed));
    }
}