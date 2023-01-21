

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;

public class DriveTrain extends SubsystemBase {
    // Initializing motors
    private final CCSparkMax frontLeft = new CCSparkMax("Front Left", "fl", RobotMap.FRONT_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FRONT_LEFT_REVERSE, true);
    private final CCSparkMax frontRight = new CCSparkMax("Front Right", "fr", RobotMap.FRONT_RIGHT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FRONT_RIGHT_REVERSE, true);
    private final CCSparkMax backLeft = new CCSparkMax("Back Left", "bl", RobotMap.BACK_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_LEFT_REVERSE, true);
    private final CCSparkMax backRight = new CCSparkMax("Back Right", "br", RobotMap.BACK_RIGHT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_RIGHT_REVERSE, true);

    MotorControllerGroup left = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup right = new MotorControllerGroup(frontRight, backRight);
    
    DifferentialDrive driveTrain = new DifferentialDrive(left, right);

    //Auto Gyro
    PIDController gyroController = new PIDController(0.5, 0, 0);
    AHRS gyro = new AHRS(SPI.Port.kMXP);

    public double defaultAccelTime = .25;

    
    double currentSpeed = 0;
    double deltaTime = .02;
    /**
     * @param targetSpeed The speed that will be accelerated to
     * @param turnSpeed The turning speed; this one will not accelerate
     * @param accelTime The time it will take to accelerate to max speed in seconds.
     */
    public void axisDrive(double targetSpeed, double turnSpeed, double accelTime) {
        if(accelTime != 0){
            if(Math.abs(currentSpeed - targetSpeed) > .05){
                currentSpeed += deltaTime / accelTime * Math.signum(targetSpeed - currentSpeed);
            }
            driveTrain.arcadeDrive(currentSpeed * currentSpeed * Math.signum(currentSpeed), turnSpeed * turnSpeed * Math.signum(turnSpeed) * .5);
            System.out.println(currentSpeed);
        } else {
            driveTrain.arcadeDrive(targetSpeed, turnSpeed);
        }
    }
    
    public void balance(double gyroAngle){
        //test function
        driveTrain.arcadeDrive(gyroController.calculate(gyroAngle), 0);
    }
    
    PIDController positionController = new PIDController(.5, 0, .1);
    public Command moveTo(double position){
        frontLeft.reset();
        frontRight.reset();
        RunCommand res = new RunCommand(() -> {
            left.set(positionController.calculate(frontLeft.getPosition(), position));
            right.set(positionController.calculate(frontRight.getPosition(), position));
        }, this){
            @Override
            public boolean isFinished() {
                // TODO Auto-generated method stub
                return Math.abs(position - frontLeft.getPosition()) < .5 && Math.abs(position - frontRight.getPosition()) < .5;
            }
        };
        return res;
    }
    
    // public Command balanceCommand(){
    //     RunCommand res = new RunCommand(() -> balance(gyro.getAngle()), this){
    //         @Override
    //         public boolean isFinished(){
    //             return false;
    //         }
    //     };
    //     return res;
    // }
    public Command balanceCommand() {
        RunCommand res = new RunCommand(() -> {
            if(gyro.getPitch() < -0.5)
                axisDrive(0.5, 0, defaultAccelTime);
            else if(gyro.getPitch() > 0.5)
                axisDrive(-0.5, 0, defaultAccelTime);
            else
                axisDrive(0, 0, defaultAccelTime);
        }, this) {
            @Override
            public boolean isFinished() {
                return false;
            }
        };
        return res;
    }

    PIDController angController = new PIDController(0.5, 0, 0);
    public Command turnAngle(double angle){
        gyro.reset();
        RunCommand res = new RunCommand(() -> axisDrive(0, angController.calculate(gyro.getYaw(), angle), defaultAccelTime), this){
            @Override
            public boolean isFinished() {
                // TODO Auto-generated method stub
                return Math.abs(gyro.getYaw() - angle) < 2;
            }
        };
        return res;
    }




    private double x = 0;
    private double y = 0;
    private double z = 0;
    private double speed = 1;
    public void test(){
        //frontLeft.set(Math.abs(gyro.getPitch()) > 30 ? Math.signum(gyro.getPitch()) * speed : 0);
        frontLeft.set(speed * gyro.getPitch() / 180);
        System.out.println(gyro.getPitch());
    }

    public double motorbrr(){
        return gyro.getPitch() / 180;
    }

}
