package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.parent.RobotMap;

public class Arm extends SubsystemBase {
  // Initializing shooter motors
    final CCSparkMax arm = new CCSparkMax("Shooter", "shoot", RobotMap.ARM, MotorType.kBrushless, IdleMode.kBrake, RobotMap.ARM_REVERSE, true);

    public void move(double speed) {
        arm.set(speed);
    }
}