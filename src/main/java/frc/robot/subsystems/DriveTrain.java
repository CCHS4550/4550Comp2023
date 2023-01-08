

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.parent.RobotMap;

public class DriveTrain extends SubsystemBase {
    // Initializing motors
    private final CCSparkMax frontLeft = new CCSparkMax("Front Left", "fl", RobotMap.FRONT_LEFT, MotorType.kBrushless, IdleMode.kCoast, RobotMap.FRONT_LEFT_REVERSE, true);
    private final CCSparkMax frontRight = new CCSparkMax("Front Right", "fr", RobotMap.FRONT_RIGHT, MotorType.kBrushless, IdleMode.kCoast, RobotMap.FRONT_RIGHT_REVERSE, true);
    private final CCSparkMax backLeft = new CCSparkMax("Back Left", "bl", RobotMap.BACK_LEFT, MotorType.kBrushless, IdleMode.kCoast, RobotMap.BACK_LEFT_REVERSE, true);
    private final CCSparkMax backRight = new CCSparkMax("Back Right", "br", RobotMap.BACK_RIGHT, MotorType.kBrushless, IdleMode.kCoast, RobotMap.BACK_RIGHT_REVERSE, true);

    MotorControllerGroup left = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup right = new MotorControllerGroup(frontRight, backRight);
    
    DifferentialDrive driveTrain = new DifferentialDrive(left, right);

    public void axisDrive(double speed, double turnSpeed) {
        driveTrain.arcadeDrive(speed * speed, turnSpeed * turnSpeed);
    }
}
