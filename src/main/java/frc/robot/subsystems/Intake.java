package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.helpers.CCSparkMax;
import frc.helpers.OI;
import frc.helpers.PneumaticsSystem;
import frc.maps.ControlMap;
import frc.maps.RobotMap;
import frc.robot.Robot;


public class Intake extends SubsystemBase {
    //Extends and retracts system
    private final static CCSparkMax intake_top = new CCSparkMax("Intake_Top", "In_P", RobotMap.INTAKE_TOP, MotorType.kBrushless, IdleMode.kCoast, RobotMap.INTAKE_TOP_REVERSE);
    private final static CCSparkMax intake_bottom = new CCSparkMax("Intake_Bottom", "In_B", RobotMap.INTAKE_BOTTOM, MotorType.kBrushless, IdleMode.kCoast, RobotMap.INTAKE_BOTTOM_REVERSE);
    private final static MotorControllerGroup intakey = new MotorControllerGroup(intake_bottom, intake_top);
    
    private final CCSparkMax extender = new CCSparkMax("Arm", "Arm", RobotMap.ARM, MotorType.kBrushless, IdleMode.kBrake, RobotMap.ARM_REVERSE);
    
    private double targetEncoder = 0;

    public Intake(DriveTrain dt){
        intake_bottom.reset();
        intake_top.reset();
        extender.reset();
        targetEncoder = 0;
        //extender.setPositionConversionFactor(RobotMap.INTAKE_POSITION_CONVERSION_FACTOR);
    }

    private static final double inCoder = -3; // change this teehee
    private static final double outCoder = -32; //and dis heetee

    public void printEncoder(){
        // System.out.println("Encoder " + extender.getEncoder().getPosition());
        // System.out.println("targeting " + targeting + " position " + targetEncoder);
        System.out.println("extender encoder:  " + extender.getPosition());
    }

    public void resetEncoders(){
        extender.reset();
    }

    public Command toggle(){
        
        InstantCommand s = new InstantCommand(() -> {
            if(Math.abs(extender.getPosition() - inCoder) < Math.abs(extender.getPosition() - outCoder)){
                targetEncoder = outCoder;
            }else{
                targetEncoder = inCoder;
            }
        });
        
        RunCommand res = new RunCommand(() -> {
            double val = controller.calculate(extender.getPosition(), targetEncoder);
            extender.set(OI.normalize(val, -.3, 0.3));
            System.out.println("extender encoder:  " + extender.getPosition());
        }, this){
            @Override
            public boolean isFinished() {
                if(targetEncoder == outCoder){
                    return extender.getPosition() <= outCoder;
                } else {
                    return extender.getPosition() >= inCoder;
                }
            }
        };


        InstantCommand st = new InstantCommand(() -> {
            extender.set(0);
        });

        return new SequentialCommandGroup(s, res, st);
    }

    public Command spintakeTime(double speed, boolean stopTop, double time){
        InstantCommand s = new InstantCommand(() -> spintake(speed, stopTop));
        InstantCommand a = new InstantCommand(() -> spintake(0, stopTop));
        WaitCommand w = new WaitCommand(time);
        return new SequentialCommandGroup(s, w, a);
    }
   
    PIDController controller = new PIDController(.5, 0, 0);

    public void moveIntake(double speed){
        extender.set(OI.normalize(speed, -.4, .4));
    }
      //Spin intake
    public void spintake(double speed, boolean stopTop) {
        if(!stopTop){
        intakey.set(OI.normalize(speed, -.8, .3));
        }else{
            intake_bottom.set(OI.normalize(speed, -.8, .3));
        }
    }
    public void setSpin(double speed){
        intakey.set(speed);
    }
    double count = 0;
    double st = 0;
    double sb = 0;
    // PIDController controller2 = new PIDController(.5, 0, 0);
    public Command accSpin(double speed, double breakoff, double total){
        double dt = 0.02;
        InstantCommand s = new InstantCommand(() -> {
            count = 0;
            st = 0;
            sb = 0;
        });
        Command c = new RunCommand(() -> {
            count++;
            double time = count * dt;
            if (time >= breakoff) st += speed / ((total - breakoff) / dt);
            sb += speed / (total / dt);
            intake_top.set(st);
            intake_bottom.set(sb);
        }, this){
            @Override
            public boolean isFinished(){
                return count > total / dt;
            }
        };
        return new SequentialCommandGroup(s, c);
    }

    double v;
    public Command autoShoot(String level){
        if(level.equals("High")){
            return new SequentialCommandGroup(
                new InstantCommand(() -> moveIntake(0.2), this),
                new WaitCommand(0.2),
                new InstantCommand(() -> moveIntake(0), this),
                accSpin(-1, 0.1, 0.15),
                new WaitCommand(.3),
                new InstantCommand(() -> spintake(0, false))
            );
        }
        else if(level.equals("Middle")){
            return new SequentialCommandGroup(
                new InstantCommand(() -> moveIntake(0.2), this),
                new WaitCommand(0.2),
                new InstantCommand(() -> moveIntake(0), this),
                accSpin(-.2, .15, 0.15),
                new WaitCommand(.3),
                new InstantCommand(() -> spintake(0, false))
            );
        }else if(level.equals("Horizontal")){
            //Bring intake to position perpendicular to the ground, then shoot
            RunCommand res = new RunCommand(() -> {
                v = controller.calculate(extender.getPosition(), inCoder);
                extender.set(OI.normalize(v, -.3, 0.3));
                System.out.println("extender encoder:  " + extender.getPosition());
            }, this){
                @Override
                public boolean isFinished() {
                    return extender.getPosition() >= inCoder;
                }
            };
            return new SequentialCommandGroup(
                res,
                // new WaitCommand(0.2),
                accSpin(-.2, .15, 0.15),
                new WaitCommand(.3),
                new InstantCommand(() -> spintake(0, false))
            );

        }{
            //low (do later)
            return new SequentialCommandGroup(
                accSpin(-1, 0.1, 0.15),
                new WaitCommand(.2),
                new InstantCommand(() -> spintake(0, false))
            );
        }
    }
    
    /**
     * @param intake_speed: The mechanisms controller left vertical joystick, passed to the intake
     * @param retract_speed: The mechanisms controller right vertical joystick, passed to the arm
     */
    public void manageIntake(double intake_speed, double retract_speed, boolean stopTop){
        moveIntake(retract_speed);
        spintake(intake_speed, stopTop);
        // setSpin(intake_speed);
        printEncoder();
    }

    public void setDefaultCommand(Command... cs){
        ParallelCommandGroup p = new ParallelCommandGroup();
        for(Command c : cs) p.addCommands(c);
        super.setDefaultCommand(p);
    }

    public Command moveToOut(){
        RunCommand res = new RunCommand(() -> {
            double val = controller.calculate(extender.getPosition(), outCoder);
            extender.set(OI.normalize(val, -.3, 0.3));
            System.out.println("extender encoder:  " + extender.getPosition());
        }, this){
            @Override
            public boolean isFinished() {
                return extender.getPosition() <= outCoder;
            }
        };
        return res;
    }
    
    // public Command moveToIn(){
    //     RunCommand res = new RunCommand(() -> {
    //         double val = controller.calculate(extender.getPosition(), targetEncoder);
    //         extender.set(OI.normalize(val, -.3, 0.3));
    //         System.out.println("extender encoder:  " + extender.getPosition());
    //     }, this){
    //         @Override
    //         public boolean isFinished() {
    //             return extender.getPosition() <= outCoder;
    //         }
    //     };
    //     return res;
    // }
}