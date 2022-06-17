package frc.robot.subsystems;

//Java imports

//Vendor imports

import com.studica.frc.Cobra;
import com.studica.frc.Servo;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sensor extends SubsystemBase
{
    //Creates all necessary hardware interface here for sensors
    //For servo testing also????

    // Sensors
    private final DigitalInput input10;
    private AnalogInput sharp;
    
    
    private int i;
    
    // Good for debugging
    // Shuffleboard
    private final ShuffleboardTab tab = Shuffleboard.getTab("Sensors");
    private final NetworkTableEntry D_inputDisp = tab.add("inputDisp", false).getEntry();
    private final NetworkTableEntry D_cntDisp = tab.add("cntDisp", 0).getEntry();
    private final NetworkTableEntry D_IRSensor = tab.add("IR Value (cm)", 0).getEntry();
    private final NetworkTableEntry D_Servo = tab.add("Servo", 0).getEntry();
    //Subsystem for sensors
    //This is just an example.
    public Sensor() {
        //Constuct a new instance
        
        sharp = new AnalogInput(0);
        input10 = new DigitalInput(Constants.INPUT0);
        
    }

    /**
     * Sets the servo angle
     * <p>
     * 
     * @param degrees degree to set the servo to, range 0° - 300°
     */
    
    public Boolean getSwitch() {
        return input10.get();
    }


    /**
     * Call for the raw ADC value
     * <p>
     * 
     * @param channel range 0 - 3 (matches what is on the adc)
     * @return value between 0 and 2047 (11-bit)
     */
    public int getCobraRawValue(final int channel) {
        return 0;
    }


    /**
     * Call for the distance measured by the Sharp IR Sensor
     * <p>
     * 
     * @return value between 0 - 100 (valid data range is 10cm - 80cm)
     */
    public double getIRDistance() {
        return (Math.pow(sharp.getAverageVoltage(), -1.2045)) * 27.726;
    }
    
    /**
     * Code that runs once every robot loop
     */
    @Override
    // Runs every 20ms
    public void periodic()
    {
        i++;
        //Display on shuffleboard
        //These display is good for debugging but may slow system down.
        //Good to remove unnecessary display during competition
        D_inputDisp.setBoolean(getSwitch());
        D_cntDisp.setNumber(i);
        D_IRSensor.setNumber(getIRDistance());
        
    }
}