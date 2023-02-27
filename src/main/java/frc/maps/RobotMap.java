package frc.maps;

// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import frc.robot.CCSparkMax;

/*
    RobotMap holds all the ports involved in the robot.
    This ranges from talon ports, all the way to the ports
    on the controllers. This also contains the polarity for the wheels
    and the various ports assoiated with sensors
    If you wish to create your own port, here is the syntax:
        public static final returnType name = value;
    Notes on creating ports:
        1. Ports must be integers or booleans
        2. they MUST be public static final;
        3. If the port is not plugged in, make int values -1, and boolean values false
*/
public interface RobotMap {
     //DriveTrain Constants
     public static final int FRONT_RIGHT = 3;
     public static final boolean FRONT_RIGHT_REVERSE = true;
     public static final int FRONT_LEFT = 1;
     public static final boolean FRONT_LEFT_REVERSE = false;
     public static final int BACK_RIGHT = 4;
     public static final boolean BACK_RIGHT_REVERSE = true;
     public static final int BACK_LEFT = 2;
     public static final boolean BACK_LEFT_REVERSE = false;

     public static final double DRIVE_ENCODER = 0.1673;

     // Intake Arm Retractor Constants
     public static final int ARM = 5;
     public static final boolean ARM_REVERSE = false;

     //Intake Spinner Constants (Motors)
     public static final int INTAKE_TOP = 7;
     public static final boolean INTAKE_TOP_REVERSE = true; //change

     
     public static final int INTAKE_BOTTOM = 6;
     public static final boolean INTAKE_BOTTOM_REVERSE = true;

     public static final double INTAKE_POSITION_CONVERSION_FACTOR = 0.51095075;

}
