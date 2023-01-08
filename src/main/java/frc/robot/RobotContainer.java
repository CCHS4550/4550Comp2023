package frc.robot;

import javax.print.attribute.standard.JobHoldUntil;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.helpers.OI;
import frc.parent.ControlMap;
import frc.robot.subsystems.MotorEx;
import frc.robot.subsystems.*;
public class RobotContainer {
    //must instantiate an object of each subsystem you use
    private MotorEx example = new MotorEx();
    private DriveTrain chassis = new DriveTrain();
    private Arms arms = new Arms();
    private Shooter shooter = new Shooter();

    Joystick[] controllers = OI.joystickArray;

    public RobotContainer(){
        configureButtons();
        chassis.setDefaultCommand(new RunCommand(() -> chassis.axisDrive(OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL), OI.axis(0, ControlMap.R_JOYSTICK_HORIZONTAL)), chassis));
        shooter.setDefaultCommand(new RunCommand(() -> shooter.shoot(OI.axis(1, ControlMap.RT)), shooter));
        arms.setDefaultCommand(new RunCommand(() -> arms.setSpeed(OI.dPadAng(1) > -1 ? Math.cos(Math.toRadians(OI.dPadAng(1))) : 0), arms));
        
        //arms.setDefualtCommand(new RunCommand(() -> arms.setSpeed(OI.dPad(1, )), arms);
    } 
        //This is Tyler's contribution to this code, you're welcome!!!
    private void configureButtons() {
        //this is how you do teleop stuff
        //when you create a trigger or trigger subclass, it will run whatever you want run when you specify
        
        //https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Button.html 
        //list of modifiers to control what happens for trigger objects (joystickbuttons extend trigger)

        //https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Trigger.html 
        //triggers are like buttons, but you can control when they go off
        //any logic amongst triggers must be done with .and, .negate, and others
        //see link for full list of logic operators


        new JoystickButton(controllers[0], ControlMap.A_BUTTON)
         .whenPressed(() -> example.setSpeed(0.5))
         .whenReleased(() -> example.setSpeed(0));

        new JoystickButton(controllers[1], ControlMap.A_BUTTON)
         .whenPressed(() -> shooter.load(0.5))
         .whenReleased(() -> shooter.load(0));
        
    }
        


        

    void test(){

    }

    public Command getAutoCommand(){
        //see Autonomous class for more details
        return new Autonomous(example);
    }
}
