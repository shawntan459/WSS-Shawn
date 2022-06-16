package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.gamepad.OI;
import frc.robot.subsystems.OmniDrive;
import frc.robot.subsystems.Sensor;
import java.lang.Math;
public class TeleCmd extends CommandBase
{
    /**
     * Bring in Subsystem and Gamepad code
     */
    private final OmniDrive m_omnidrive;
    private final Sensor m_sensor;
    private final OI m_oi;
    

    /**
     * Constructor
     */
    public TeleCmd(OmniDrive omnidrive, OI oi)
    {
        m_omnidrive = RobotContainer.m_omnidrive;
        m_sensor = RobotContainer.m_sensor;
        m_oi = RobotContainer.m_oi;
        addRequirements(m_omnidrive); //add the drive subsystem as a requirement 
		//addRequirements(m_menu); 
    }

    /**
     * Code here will run once when the command is called for the first time
     */
    @Override
    public void initialize()
    {

    }

    /**
     * Code here will run continously every robot loop until the command is stopped
     */
    @Override
    public void execute()
    {
        /**
         * Get Joystick data
         */
        //Right stick for X-Y control
        //Left stick for W (rotational) control
        double x = m_oi.getRightDriveX();
        double y = -m_oi.getRightDriveY();//Down is positive. Need to negate
        double w = -m_oi.getLeftDriveX(); //X-positive is CW. Need to negate

        //Get other buttons?

        //Add code here to control servo motor etc.
        /* 
        double a[][] = {{ -0.866025, -0.5, 1, 0.1 },
                        { 1, , 1, 0.1 },
                        { 2, 1, 1, 7 }};*/
        double s0, s1, s2;
        /*
         * s0 = -Math.sqrt(3)/2*x - 0.5*y + 1*w;
        s1 = x + 1*w;
        s2 = Math.sqrt(3)/2*y - 0.5*x + 1*w;
         */
        s0 = -0.5*x - Math.sqrt(3)/2*y + 1*w;
        s1 = x + 1*w;
        s2 = Math.sqrt(3)/2*y - 0.5*x + 1*w;
         
        //m_omnidrive.setMotorOut012(s0,s1,s2);
        double input_start = -1;    // The lowest number of the range input.
        double input_end = 1;    // The largest number of the range input.
        double output_start = 0; // The lowest number of the range output.
        double output_end = 300;  // The largest number of the range output.
        double output = output_start + ((output_end - output_start) / (input_end - input_start)) * (w - input_start);
        //m_sensor.setServoAngle(output);
    
        
        m_omnidrive.setRobotSpeedXYW(x*0.6, y*0.6, w*Math.PI);

    }

    /**
     * When the comamnd is stopped or interrupted this code is run
     * <p>
     * Good place to stop motors in case of an error
     */
    @Override
    public void end(boolean interrupted)
    {
        m_omnidrive.setMotorOut012(0, 0, 0);
    }

    /**
     * Check to see if command is finished
     */
    @Override
    public boolean isFinished()
    {
        return false;
    }
}