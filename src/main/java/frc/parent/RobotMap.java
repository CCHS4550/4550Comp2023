package frc.parent;

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
     public static final int FRONT_RIGHT = 0;
     public static final boolean FRONT_RIGHT_REVERSE = false;
     public static final int FRONT_LEFT = 1;
     public static final boolean FRONT_LEFT_REVERSE = false;
     public static final int BACK_RIGHT = 2;
     public static final boolean BACK_RIGHT_REVERSE = false;
     public static final int BACK_LEFT = 3;
     public static final boolean BACK_LEFT_REVERSE = false;
 
     //Shooter Constants
     public static final int SHOOTER = 4;
     public static final boolean SHOOTER_REVERSE = false;
     public static final int LOADER = 5;
     public static final boolean LOADER_REVERSE = false;
     
 
     //Bar Lifter Constants
     public static final int LIFTER = 6;
     public static final boolean LIFTER_REVERSE = false;

     //Arms Constants
     public static final int ARMS = 7;
     public static final boolean ARMS_REVERSE = false;
}
