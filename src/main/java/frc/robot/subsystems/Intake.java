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

public class Intake extends SubsystemBase {
    //Extends and retracts system
    private final static CCSparkMax intake_top = new CCSparkMax("Intake_Top", "In_P", RobotMap.INTAKE_TOP, MotorType.kBrushless, IdleMode.kBrake, RobotMap.INTAKE_TOP_REVERSE);
    private final static CCSparkMax intake_bottom = new CCSparkMax("Intake_Bottom", "In_B", RobotMap.INTAKE_BOTTOM, MotorType.kBrushless, IdleMode.kBrake, RobotMap.INTAKE_BOTTOM_REVERSE);
    private final static MotorControllerGroup intakey = new MotorControllerGroup(intake_bottom, intake_top);
    private static final CCSparkMax extender = new CCSparkMax("Arm", "Arm", RobotMap.ARM, MotorType.kBrushless, IdleMode.kBrake, RobotMap.ARM_REVERSE);
    private static DriveTrain chassis;
    private boolean targeting = false;

    private static boolean isIn = true;
    public Intake(DriveTrain dt){
        chassis = dt;
    }

    private static final double inCoder = 123124325; // change
    private static final double outCoder = 696966996; //changes

    public void printEncoder(){
        System.out.println(extender.get());
    }

    
    
    double targetEncoder = isIn ? outCoder : inCoder;
    public void toggle() {
        // if close enough to top, move to bottom, otherwise move to top
        if(Math.abs(extender.get() - inCoder) < 10){
            targetEncoder = outCoder;
        }else{
            targetEncoder = inCoder;
        }
        targeting = true;
    }

    public void moveIntake(double speed){
        if (speed != 0) {
            extender.set(OI.normalize(speed, -.3, .3));
            targeting = false;
        }
    }

    PIDController controller = new PIDController(.5, 0, 0);
    public void target(){
        if(targeting){
            double val = controller.calculate(extender.get(), targetEncoder);
            extender.set(OI.normalize(val, -.3, 0.3));
        }
    }
    
      //Spin intake
    public void spintake(double speed) {
        intakey.set(speed);
    }
    
    /**
     * @param intake_speed: The mechanisms controller left vertical joystick, passed to the intake
     * @param retract_speed: The mechanisms controller right vertical joystick, passed to the arm
     */
    public void manageIntake(double intake_speed, double retract_speed){
        spintake(intake_speed);
        moveIntake(retract_speed);
    }

    public void setDefaultCommand(Command... cs){
        ParallelCommandGroup p = new ParallelCommandGroup();
        for(Command c : cs) p.addCommands(c);
        super.setDefaultCommand(p);
    }

}