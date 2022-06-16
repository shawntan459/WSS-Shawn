package frc.robot.subsystems;


import com.studica.frc.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Good to create a subsystem for robot arm
public class Arm extends SubsystemBase {

    private final Servo servo0;
    private final Servo servo1;
    private double servoValue0;
    private double servoValue1;
    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    private final NetworkTableEntry D_armValue0 = tab.add("armValue", 0).getEntry();
    private final NetworkTableEntry D_armValue1 = tab.add("armValue", 0).getEntry();

    public Arm() {
        servo0 = new Servo(0);
        servo1 = new Servo(1);
    }
    /**
     * Sets the servo angle
     * <p>
     * 
     * @param degrees degree to set the servo to, range 0° - 300°
     */
    public void setServoAngle0(final double degrees){
        servoValue0 = degrees;
        servo0.setAngle(degrees);
    }
    public void setServoAngle1(final double degrees){
        servoValue1 = degrees;
        servo1.setAngle(degrees);
    }


    @Override
    // Runs every 20ms
    public void periodic()
    {
        D_armValue0.setDouble(servoValue0);
        D_armValue1.setDouble(servoValue1);
    }
    
}
