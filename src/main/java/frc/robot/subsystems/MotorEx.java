package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.parent.RobotMap;

public class MotorEx extends SubsystemBase{
    private final CCSparkMax example = new CCSparkMax("Example Motor", "ex", RobotMap.EXAMPLE_MOTOR_PORT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.EXAMPLE_MOTOR_REVERSE, true);

    public MotorEx(){

    }

    public void setSpeed(double speed){
        example.set(speed);
    }

    PIDController controller = new PIDController(0.5, 0, 0);
    public void targetPosition(double position){
        setSpeed(controller.calculate(example.getPosition(), position));
    }

    public SequentialCommandGroup runForTime(double seconds, double power){
        SequentialCommandGroup res = new SequentialCommandGroup(
            new InstantCommand(() -> setSpeed(power)),
            new WaitCommand(seconds)
        );
        return res;
    }

    public Command reachPos(double pos){
        RunCommand res = new RunCommand(() -> targetPosition(pos), this){
            @Override
            public boolean isFinished(){
                return example.getPosition() - pos < 0.05;
            }
        };
        return res;
    }
}
