package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.parent.RobotMap;

public class Arms extends SubsystemBase {
    private final CCSparkMax arms = new CCSparkMax("Arms", "arms", RobotMap.ARMS, MotorType.kBrushless, IdleMode.kCoast, RobotMap.ARMS_REVERSE, true);

    public void setSpeed(double speed){
        arms.set(speed);
    }

}