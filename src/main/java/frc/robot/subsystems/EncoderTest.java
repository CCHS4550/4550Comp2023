package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.helpers.CCSparkMax;
// import frc.maps.RobotMap;


public class EncoderTest extends SubsystemBase {
  // Initializing arm motor
    AnalogInput encoder;
    public EncoderTest(int port){
      encoder = new AnalogInput(port);
    }
    
    public double get(){
        return encoder.getValue();
    }
  }
