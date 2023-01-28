

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import javax.lang.model.util.ElementScanner6;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.maps.RobotMap;

public class DriveTrain extends SubsystemBase {
    // Initializing motors
    private final CCSparkMax frontLeft = new CCSparkMax("Front Left", "fl", RobotMap.FRONT_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FRONT_LEFT_REVERSE, RobotMap.DRIVE_ENCODER);
    private final CCSparkMax frontRight = new CCSparkMax("Front Right", "fr", RobotMap.FRONT_RIGHT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FRONT_RIGHT_REVERSE, RobotMap.DRIVE_ENCODER);
    private final CCSparkMax backLeft = new CCSparkMax("Back Left", "bl", RobotMap.BACK_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_LEFT_REVERSE, RobotMap.DRIVE_ENCODER);
    private final CCSparkMax backRight = new CCSparkMax("Back Right", "br", RobotMap.BACK_RIGHT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_RIGHT_REVERSE, RobotMap.DRIVE_ENCODER);

    MotorControllerGroup left = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup right = new MotorControllerGroup(frontRight, backRight);
    
    DifferentialDrive driveTrain = new DifferentialDrive(left, right);

    //Auto Gyro
    PIDController gyroController = new PIDController(0.5, 0, 0);
    public AHRS gyro = new AHRS(SPI.Port.kMXP);

    public double defaultAccelTime = .25;
    public double slowModeFactor = 1.0;

    
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
            System.out.println(gyro.getYaw());
            driveTrain.arcadeDrive(currentSpeed * currentSpeed * Math.signum(currentSpeed) * slowModeFactor, turnSpeed * turnSpeed * Math.signum(turnSpeed) * (targetSpeed != 0 ? 1 : .5));
            //System.out.println("FL: " + frontLeft.getPosition() + "   FR: " + frontRight.getPosition());
            // System.out.println(currentSpeed);
        } else {
            driveTrain.arcadeDrive(targetSpeed * slowModeFactor, turnSpeed * (targetSpeed != 0 ? 1 : .5) * slowModeFactor);
        }
    }// D
    
    public void balance(double gyroAngle){
        //test function
        driveTrain.arcadeDrive(gyroController.calculate(gyroAngle), 0);
    }
    
    private double kp = 0.5;
    public Command moveTo(double position){
        frontLeft.reset();
        frontRight.reset();
        double pos = position * -1;
        RunCommand res = new RunCommand(() -> {
            double err = -frontLeft.getPosition() + pos;
            double val = OI.normalize(err * kp, -.3, .3);
            left.set(val);
            right.set(val);

        }, this){
            @Override
            public boolean isFinished() {
                // TODO Auto-generated method stub
                return Math.abs(pos - frontLeft.getPosition()) < 1/12;
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

    double turnTime = 0;
    public Command turnAngle(double angle){
        gyro.reset();
        turnTime = 0;
        // RunCommand res = new RunCommand(() -> axisDrive(0, angController.calculate(gyro.getYaw(), angle), defaultAccelTime), this){
        RunCommand res = new RunCommand(() -> {
            double err = angle - gyro.getYaw();
            double val = err * kp / 45 * .5;
            val = OI.normalize(val, -1, 1);
            left.set(-val);
            right.set(+val);

            if (Math.abs(gyro.getYaw() - angle) < 15){
                turnTime++;
            } else {
                turnTime = 0;
            }
        }, this){
            @Override
            public boolean isFinished() {
                // TODO Auto-generated method stub
                return turnTime > 5;
            }
        };
        return res;
    }

    public void toggleSlowMode(){
        if(slowModeFactor == .5){
            slowModeFactor = 1;
        } else if(slowModeFactor == 1){
            slowModeFactor = .5;
        }
    }

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> {
    //         // Reset odometry for the first path you run during auto
    //         if(isFirstPath){
    //             this.resetOdometry(traj.getInitialPose());
    //         }
    //         }),
    //         new PPRamseteCommand(
    //             traj,
    //             this::getPose, // Pose supplier
    //             new RamseteController(),
    //             new SimpleMotorFeedforward(KS, KV, KA),
    //             this.kinematics, // DifferentialDriveKinematics
    //             this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
    //             new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //             new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
    //             this::outputVolts, // Voltage biconsumer
    //             true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //             this // Requires this drive subsystem
    //         )
    //     );
    // }




    private double x = 0;
    private double y = 0;
    private double z = 0;
    private double speed = 1;
    public void test(){
        System.out.println("FL: " + frontLeft.getPosition() + "   FR: " + frontRight.getPosition());
        if(OI.button(0, ControlMap.A_BUTTON)) {
            frontLeft.reset();
            frontRight.reset();
        }
        axisDrive(OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL), 0, 0);
        // left.set(OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL));
        // right.set(OI.axis(0, ControlMap.R_JOYSTICK_VERTICAL));
    }

    public double motorbrr(){
        return gyro.getPitch() / 180;
    }

}
