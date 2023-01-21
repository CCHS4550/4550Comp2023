package frc.robot;

import javax.print.attribute.standard.JobHoldUntil;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.diagnostics.CommandSelector;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.robot.autonomous.Autonomous;
import frc.robot.autonomous.BallinAutonomous;
import frc.robot.autonomous.DriveAutonomous;
// import frc.robot.subsystems.MotorEx;
import frc.robot.subsystems.*;
public class RobotContainer {
    //must instantiate an object of each subsystem you use
    private DriveTrain chassis = new DriveTrain();
    private Claw claw = new Claw();
    private Arm arm = new Arm();

    Joystick[] controllers = OI.joystickArray;

    public RobotContainer(){
        configureButtons();
        chassis.setDefaultCommand(new RunCommand(() -> chassis.axisDrive(OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL), OI.axis(0, ControlMap.R_JOYSTICK_HORIZONTAL), chassis.defaultAccelTime), chassis));
        arm.setDefaultCommand(new RunCommand(() -> arm.move(OI.axis(1, ControlMap.L_JOYSTICK_VERTICAL)), arm)); 
        claw.setDefaultCommand(new RunCommand(() -> claw.moveClaw(Math.cos(OI.dPadAng(1) > 0 ? OI.dPadAng(1) : 0))));
        //Humza wrote the line above
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



        // new JoystickButton(controllers[0], ControlMap.A_BUTTON)
        //  .whenPressed(() -> example.setSpeed(0.5))
        //  .whenReleased(() -> example.setSpeed(0));

        new JoystickButton(controllers[1], ControlMap.A_BUTTON)
         .whenPressed(() -> claw.toggleArm());
        
    }
        


        

    void test(){
        // chassis.test();
        arm.move(chassis.motorbrr());
        // System.out.println(chassis.motorbrr());
    }
    CommandSelector selector = new CommandSelector(
        "Autonomous",
        new Autonomous(chassis, arm, claw, true, false),
        new Autonomous(chassis, arm, claw, true, true),
        new Autonomous(chassis, arm, claw, false, false),
        new Autonomous(chassis, arm, claw, false, true),
        new BallinAutonomous(chassis, arm, claw),
        new DriveAutonomous(chassis, arm, claw));
        
    public Command getAutoCommand(){
        //see Autonomous class for more details
        
        return selector.value();
    }
}
