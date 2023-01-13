package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.helpers.PneumaticsSystem;
import frc.maps.RobotMap;

public class Claw extends SubsystemBase {
    PneumaticsSystem claw = new PneumaticsSystem(PneumaticsModuleType.CTREPCM, RobotMap.CLAW_ONE, RobotMap.CLAW_TWO);

    public void toggleClaw() {
        claw.toggle();
    }
}