package frc.robot;

import javax.management.InstanceAlreadyExistsException;
import javax.print.attribute.standard.JobHoldUntil;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.diagnostics.BooleanSwitch;
import frc.diagnostics.CommandSelector;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.robot.autonomous.Autonomous;
import frc.robot.autonomous.BallinAutonomous;
import frc.robot.autonomous.DriveAutonomous;
import frc.diagnostics.*;
// import frc.robot.subsystems.MotorEx;
import frc.robot.subsystems.*;
public class RobotContainer {
    //must instantiate an object of each subsystem you use
    private DriveTrain chassis = new DriveTrain();
    private Claw claw = new Claw();
    private Arm arm = new Arm();

    Joystick[] controllers = OI.joystickArray;

    public RobotContainer(){
        //chassis.defaultAccelTime
        chassis.setDefaultCommand(new RunCommand(() -> chassis.axisDrive(
         OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL) * (OI.axis(0, ControlMap.RT) > 0.5 ? 0.5 : 1),
         OI.axis(0, ControlMap.R_JOYSTICK_HORIZONTAL) * (OI.axis(0, ControlMap.RT) > 0.5 ? 0.75 : 1),
         chassis.defaultAccelTime), chassis));
        arm.setDefaultCommand(new RunCommand(() -> arm.move(OI.axis(1, ControlMap.L_JOYSTICK_VERTICAL) * 0.85), arm)); 
        claw.setDefaultCommand(new RunCommand(() -> claw.moveClaw(OI.axis(1, ControlMap.R_JOYSTICK_VERTICAL)), claw));
        //Math.cos(OI.dPadAng(1) >= 0 ? Math.toRadians(OI.dPadAng(1)) : Math.PI/2)
        configureButtons();
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
         .onTrue(new InstantCommand(() -> chassis.gyro.reset(), chassis));

         new JoystickButton(controllers[0], ControlMap.B_BUTTON)
         .onTrue(new SequentialCommandGroup(new InstantCommand(() -> chassis.gyro.reset(), chassis), chassis.turnAngle(180)));

         new JoystickButton(controllers[0], ControlMap.A_BUTTON)
         .onTrue(new InstantCommand(() -> chassis.toggleSlowMode(), chassis));
    }
        


        
    DoubleEntry turnval = new DoubleEntry("bollocks", 0);
    void test(){
        chassis.test(turnval.value());
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

    public BooleanSwitch enabled = new BooleanSwitch("Enable", false);
    public DoubleEntry angle = new DoubleEntry("Angle", 0);
    public Command getAutoCommand(){
        // see Autonomous class for more details
        SequentialCommandGroup s = new SequentialCommandGroup(
            chassis.moveTo(5),
            chassis.turnAngle(90),
            chassis.moveTo(5),
            chassis.turnAngle(90),
            chassis.moveTo(5),
            chassis.turnAngle(90),
            chassis.moveTo(5),
            chassis.turnAngle(90),
            chassis.moveTo(5),
            chassis.turnAngle(90),
            chassis.moveTo(5),
            chassis.turnAngle(90),
            chassis.moveTo(5),
            chassis.turnAngle(90)
            );
        return s;
        // return selector.value();
        
        //return chassis.turnAngle(-90);

        // if(enabled.value()){
        // return chassis.turnAngle(angle.value());
        // }
    }
}
