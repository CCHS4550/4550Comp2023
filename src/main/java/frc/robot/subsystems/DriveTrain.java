

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import javax.lang.model.util.ElementScanner6;

import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
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
import frc.diagnostics.BooleanSwitch;
import frc.diagnostics.DoubleEntry;
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

    //Auto Gyro
    PIDController gyroController = new PIDController(0.5, 0, 0);
    public AHRS gyro = new AHRS(SPI.Port.kMXP);

    public double defaultAccelTime = .25;
    public double slowModeFactor = 1.0;

    
    double currentSpeed = 0;
    double deltaTime = .02;
    /**a 
     * @param targetSpeed The speed that will be accelerated to
     * @param turnSpeed The turning speed; this one will not accelerate
     * @param accelTime The time it will take to accelerate to max speed in seconds.
     */
    public void axisDrive(double targetSpeed, double turnSpeed, double accelTime) {
        targetSpeed *= .9;
        if(accelTime != 0){
            if(Math.abs(currentSpeed - targetSpeed) > .05){
                currentSpeed += deltaTime / accelTime * Math.signum(targetSpeed - currentSpeed);
            }
            System.out.println(gyro.getYaw());
            arcade(currentSpeed * currentSpeed * Math.signum(currentSpeed) * slowModeFactor *.7, slowModeFactor * turnSpeed * turnSpeed * Math.signum(turnSpeed) * (targetSpeed != 0 ? 1 : .5));
            //System.out.println("FL: " + frontLeft.getPosition() + "   FR: " + frontRight.getPosition());
            // System.out.println(currentSpeed);
        } else {
            arcade(targetSpeed * slowModeFactor, turnSpeed * (targetSpeed != 0 ? 1 : .5) * slowModeFactor);
        }

        // System.out.println(getOutputCurrent());
    }// D

    public void arcade(double yAxis, double xAxis){
        double max = .8;
        left.set(OI.normalize((yAxis - xAxis), -max, max));
        right.set(OI.normalize((yAxis + xAxis), -max, max));
    }
    
    public void balance(double gyroAngle){
        //test function
        arcade(gyroController.calculate(gyroAngle), 0);
    }
    
    
    
    private double kp = 0.5;
    public Command moveTo(double position, boolean autoCorrect){
        InstantCommand s = new InstantCommand(() -> {
            gyro.reset();
            frontLeft.reset();
            frontRight.reset();
        });
        double pos = position * -1;
        RunCommand res = new RunCommand(() -> {
            double err = -frontLeft.getPosition() + pos;
            double val = OI.normalize(err * kp, -.4, .4);
            double turnSpeed = 0.05;
            if(autoCorrect)
            //humza and alex wrote this line of code. Ryder kinda helped
                arcade(val, gyro.getYaw() > 1 ? turnSpeed * -1 * Math.signum(val) * Math.signum(pos): gyro.getYaw() < -1 ? turnSpeed * Math.signum(val) * Math.signum(pos): 0); 
            else
                arcade(val, 0);

            System.out.println(Math.abs(pos - frontLeft.getPosition()));
        }, this){
            @Override
            public boolean isFinished() {
                if(Math.abs(pos - frontLeft.getPosition()) < 1.5/12)
                    gyro.reset();
                return Math.abs(pos - frontLeft.getPosition()) < 1.5/12;
            }
        };
        return new SequentialCommandGroup(s, res);
    }

   
   
    public Command moveToToBalnenceBackwards(double position){
        boolean autoCorrect = true;
        InstantCommand s = new InstantCommand(() -> {
            frontLeft.reset();
            frontRight.reset();
            gyro.reset();
        });
        double pos = position * -1;
        RunCommand res = new RunCommand(() -> {
            double err = -frontLeft.getPosition() + pos; // the difference between the target position and the current position
            double val = OI.normalize(err * kp, -.4, .4); // val passed into motors
            double turnSpeed = 0.05;
            if(autoCorrect)
            //humza and alex wrote this line of code. Ryder kinda helped
                arcade(val, gyro.getYaw() > 2 ? turnSpeed * -1 * Math.signum(val) * Math.signum(pos): gyro.getYaw() < -1 ? turnSpeed * Math.signum(val) * Math.signum(pos): 0); 
                //arcade(val , turn);
            else
                arcade(val, 0);

            System.out.println(Math.abs(pos - frontLeft.getPosition()));
        }, this){
            @Override
            public boolean isFinished() {
                if(Math.abs(gyro.getPitch()) > 20 || Math.abs(pos - frontLeft.getPosition()) < 1.5/122)
                    gyro.reset();
                return Math.abs(gyro.getPitch()) > 20 || Math.abs(pos - frontLeft.getPosition()) < 1.5/12;
            }
        };
        return new SequentialCommandGroup(s, res);
    }

    PIDController turn = new PIDController(0.0015, 0.001, 0, .1);
    public Command turnAngle(double angle){
        InstantCommand s = new InstantCommand(() -> {
            gyro.reset();
            turnTime = 0;
            transition = false;
        });
        WaitCommand w = new WaitCommand(0.1);
        RunCommand res = new RunCommand(() -> {
            double ang = gyro.getYaw() * angle < 0 && Math.abs(gyro.getYaw()) > 150 ? gyro.getYaw() + 360 * Math.signum(angle) : gyro.getYaw();
            double val = turn.calculate(ang, angle);// - turn.getVelocityError() * error.value();
            if(Math.abs(ang - angle) < 15 && !transition){
                transition = true;
                turn.reset();
            }
            val = OI.normalize(val, -0.5, .5);
            left.set(-val);
            right.set(val);
            if (Math.abs(ang - angle) < 3){ //error
                turnTime++;
            } else {
                turnTime = 0;
            }
        }, this){
            @Override
            public boolean isFinished() {
                return turnTime > 5;
            }
        };
        return new SequentialCommandGroup(s, w, res);
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
            if(gyro.getPitch() < -4)
                axisDrive(-0.4, 0, defaultAccelTime);
            else if(gyro.getPitch() > 4)
                axisDrive(0.4, 0, defaultAccelTime);
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

    public void toggleSlowMode(){
        // if(slowModeFactor == .5){
        //     slowModeFactor = 1;
        // } else if(slowModeFactor == 1){
        //     slowModeFactor = .5;
        // }
    slowModeFactor = slowModeFactor == 0.5 ? 1 : 0.5;
    }

    double turnTime = 0;
    // boolean on = false;
    // boolean finished = false;
    // DoubleEntry kpe = new DoubleEntry("kp", 0.01);
    // DoubleEntry kie = new DoubleEntry("ki", 0.01);
    // DoubleEntry kde = new DoubleEntry("kd", 0.01);
    // DoubleEntry error = new DoubleEntry("accpeted", 1);
    // DoubleEntry period = new DoubleEntry("period", 0.1);
    boolean transition = false;
    // double count = 0;
    // double turnFactor;
    // PIDController turn2 = new PIDController(kpe.value(), kie.value(), kde.value(), period.value());
    // public Command turnAngleTest(BooleanSwitch enabled, DoubleEntry angle){
    //     gyro.reset();
    //     turnTime = 0;
    //     // RunCommand res = new RunCommand(() -> axisDrive(0, angController.calculate(gyro.getYaw(), angle), defaultAccelTime), this){
    //     RunCommand res = new RunCommand(() -> {
    //         finished = turnTime > 5;
    //         if(enabled.value()){
    //             count++;
    //             if(!on){
    //                 on = true;
    //                 turnTime = 0;
    //                 count = 0;
    //                 finished = false;
    //                 transition = false;
    //                 turn = new PIDController(kpe.value(), kie.value(), kde.value(), period.value());
    //             }
    //             if(!finished){
    //                 double ang = gyro.getYaw() * angle.value() < 0 ? gyro.getYaw() + 360 * Math.signum(angle.value()) : gyro.getYaw();
    //                 double val = turn.calculate(ang, angle.value());// - turn.getVelocityError() * error.value();
    //                 if(Math.abs(ang - angle.value()) < error.value() && !transition){
    //                     transition = true;
    //                     turn.reset();
    //                 }
    //                 val = OI.normalize(val, -0.5, .5);
    //                 left.set(-val);
    //                 right.set(val);
    //                 System.out.println(ang + " " + angle.value() + " " + Math.abs(ang - angle.value()) + " " + val);
    //                 if (Math.abs(ang - angle.value()) < 1.5){
    //                     turnTime++;
    //                 } else {
    //                     turnTime = 0;
    //                 }
    //             } else {
    //                 left.set(0);
    //                 right.set(0);
    //             }
    //         } else {
    //             if(on) gyro.reset();
    //             on = false;
    //             left.set(0);
    //             right.set(0);
    //         }
    //     }, this){
    //         @Override
    //         public boolean isFinished() {
    //             return false;
    //         }
    //     };
    //     return res;
    // }

    

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
        // left.set(-val);
        // right.set(+val);
        // System.out.println("FL: " + frontLeft.getPosition() + "   FR: " + frontRight.getPosition());
        // if(OI.button(0, ControlMap.A_BUTTON)) {
        //     frontLeft.reset();
        //     frontRight.reset();
        // }
        // axisDrive(OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL), 0, 0);
        // left.set(OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL));
        // right.set(OI.axis(0, ControlMap.R_JOYSTICK_VERTICAL));
        // axisDrive(OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL), 0, 0);
        // System.out.println(frontLeft.getPosition());
        // if(OI.button(0, ControlMap.A_BUTTON)) frontLeft.reset();
        moveTo(5, false);
    }

    public double motorbrr(){
        return gyro.getPitch() / 180;
    }

}
