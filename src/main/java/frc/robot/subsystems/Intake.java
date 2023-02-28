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

    private static final double inCoder = 0; // change this teehee
    private static final double midCoder = 12.1;
    private static final double outCoder = -16.1; //and dis heetee

    public void printEncoder(){
        // System.out.println("Encoder " + extender.getEncoder().getPosition());
        // System.out.println("targeting " + targeting + " position " + targetEncoder);
        System.out.println("extender encoder:  " + extender.getPosition());
    }

    public void resetEncoders(){
        extender.reset();
    }

    public Command toggle(){
        if(Math.abs(extender.getPosition() - midCoder) < Math.abs(extender.getPosition() - outCoder)){
            targetEncoder = outCoder;
        }else{
            targetEncoder = midCoder;
        }
        
        RunCommand res = new RunCommand(() -> {
            double val = controller.calculate(extender.get(), targetEncoder);
            extender.set(OI.normalize(val, -.3, 0.3));
        }, this){
            @Override
            public boolean isFinished() {
                return Math.abs(targetEncoder - extender.get()) < 0.1 || OI.axis(1, ControlMap.R_JOYSTICK_VERTICAL) > 0.1;
            }
        };

        return res;
    }
   
    PIDController controller = new PIDController(.5, 0, 0);

    public void moveIntake(double speed){
        extender.set(OI.normalize(speed, -.4, .4));
    }
      //Spin intake
    public void spintake(double speed, boolean stopTop) {
        if(!stopTop){
        intakey.set(OI.normalize(speed, -.6, .5));
        }else{
            intake_bottom.set(OI.normalize(speed, -.4, .5));
        }
    }
    public void setSpin(double speed){
        intakey.set(speed);
    }
    double count = 0;
    double curr = 0;
    public Command accSpin(double speed, double t){
        double dt = 0.02;
        InstantCommand s = new InstantCommand(() -> {
            count = 0;
            curr = 0;
        });
        Command c = new RunCommand(() -> {
            count++;
            curr += speed / (t / dt);
            setSpin(curr);
        }, this){
            @Override
            public boolean isFinished(){
                return count > t / dt;
            }
        };
        return new SequentialCommandGroup(s, c);
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

}