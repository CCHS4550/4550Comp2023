package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.helpers.PneumaticsSystem;
import frc.maps.RobotMap;

public class Intake extends SubsystemBase {
    //Extends and retracts system
    private final static CCSparkMax intake_top = new CCSparkMax("Intake_Top", "In_P", RobotMap.INTAKE_TOP, MotorType.kBrushless, IdleMode.kBrake, RobotMap.INTAKE_TOP_REVERSE);
    private final static CCSparkMax intake_bottom = new CCSparkMax("Intake_Bottom", "In_B", RobotMap.INTAKE_BOTTOM, MotorType.kBrushless, IdleMode.kBrake, RobotMap.INTAKE_BOTTOM_REVERSE);
    private final static MotorControllerGroup intakey = new MotorControllerGroup(intake_bottom, intake_top);
    private static final CCSparkMax extender = new CCSparkMax("Arm", "Arm", RobotMap.ARM, MotorType.kBrushless, IdleMode.kBrake, RobotMap.ARM_REVERSE);
    private static DriveTrain chassis;
    private static boolean isIn = true;
    public Intake(DriveTrain dt){
        chassis = dt;
    }

    public void toggle() {
        
      }
    
    
      //Spin intake
    public void spintake(double speed) {
        intakey.set(speed);
    }


}