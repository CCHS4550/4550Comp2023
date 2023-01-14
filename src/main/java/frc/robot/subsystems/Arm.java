package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;


public class Arm extends SubsystemBase {
  // Initializing shooter motors
    final CCSparkMax arm = new CCSparkMax("Shooter", "shoot", RobotMap.ARM, MotorType.kBrushless, IdleMode.kCoast, RobotMap.ARM_REVERSE, true);

    public void move(double speed) {
        arm.set(speed);
    }
    PIDController controller = new PIDController(.5, 0, 0);
    public Command moveRotations(double rotations) {
      RunCommand res = new RunCommand(() -> {
        move(controller.calculate(arm.getPosition(), rotations));
    }, this){
        @Override
        public boolean isFinished() {
          return Math.abs(rotations - arm.getPosition()) < .01;
        }
    };
    return res;
    }
}