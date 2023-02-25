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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.helpers.OI;
import frc.helpers.PneumaticsSystem;
import frc.maps.RobotMap;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    //Extends and retracts system
    private final static CCSparkMax intake_top = new CCSparkMax("Intake_Top", "In_P", RobotMap.INTAKE_TOP, MotorType.kBrushless, IdleMode.kCoast, RobotMap.INTAKE_TOP_REVERSE);
    private final static CCSparkMax intake_bottom = new CCSparkMax("Intake_Bottom", "In_B", RobotMap.INTAKE_BOTTOM, MotorType.kBrushless, IdleMode.kCoast, RobotMap.INTAKE_BOTTOM_REVERSE);
    private final static MotorControllerGroup intakey = new MotorControllerGroup(intake_bottom, intake_top);
    private static final CCSparkMax extender = new CCSparkMax("Arm", "Arm", RobotMap.ARM, MotorType.kBrushless, IdleMode.kBrake, RobotMap.ARM_REVERSE);
    private static DriveTrain chassis;
    private double intake_modifier = 1;


    private boolean targeting = false;

    private static boolean isIn = true;
    public Intake(DriveTrain dt){
        intake_bottom.reset();
        intake_top.reset();
        extender.reset();
        extender.setPositionConversionFactor(RobotMap.INTAKE_POSITION_CONVERSION_FACTOR);
        chassis = dt;
    }

    private static final double inCoder = 0; // change this teehee
    private static final double outCoder = 10; //and dis heetee

    public void printEncoder(){
        // System.out.println("Encoder " + extender.getEncoder().getPosition());
        // System.out.println("targeting " + targeting + " position " + targetEncoder);
        System.out.println("penis cock balls " + extender.getPosition());
    }

    public void resetEncoders(){
        extender.reset();
    }

    
    
    double targetEncoder = isIn ? outCoder : inCoder;

    public void toggle() {
        // if close enough to top, move to bottom, otherwise move to top
        if(Math.abs(extender.getPosition() - inCoder) < 0.1 * outCoder){
            targetEncoder = outCoder;
        }else{
            targetEncoder = inCoder;
        }
        targeting = true;
    }

   
    PIDController controller = new PIDController(.5, 0, 0);
    public void target(){
        if(targeting){
            double val = controller.calculate(extender.get(), targetEncoder);
            extender.set(OI.normalize(val, -.3, 0.3));
        }
    }

    public void moveIntake(double speed){
        extender.set(OI.normalize(speed, -.25, .25));
        //possibly change the 0 to incoder; right now it stops once it is in shoot position. Oooooor change it to a value barely above/below zero so it doesn't kill itself
        // if (speed != 0 && Math.abs(extender.getPosition()) < outCoder && Math.abs(extender.getPosition()) > 0) {
            
        //     targeting = false;
        // }else if(!targeting){
        //     extender.set(0);
        // }
    }
      //Spin intake
    public void spintake(double speed) {
        intakey.set(OI.normalize(speed, -.6, .2) * intake_modifier);
    }

    public void fastMode(boolean fast){
        intake_modifier = fast ? 1.5 : 1;
    }
    
    /**
     * @param intake_speed: The mechanisms controller left vertical joystick, passed to the intake
     * @param retract_speed: The mechanisms controller right vertical joystick, passed to the arm
     */
    public void manageIntake(double intake_speed, double retract_speed, boolean fast){
        spintake(intake_speed);
        moveIntake(retract_speed);
        fastMode(fast);
        printEncoder();
    }

    public void setDefaultCommand(Command... cs){
        ParallelCommandGroup p = new ParallelCommandGroup();
        for(Command c : cs) p.addCommands(c);
        super.setDefaultCommand(p);
    }

}