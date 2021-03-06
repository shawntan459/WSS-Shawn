package frc.robot.commands.auto;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
// import the commands
import frc.robot.commands.auto.MoveRobotSense;
import frc.robot.commands.auto.MoveRobot;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class MoveDirection extends SequentialCommandGroup {
    private enum CommandSelector {
        ONE, TWO, THREE, FOUR
    }

    static public CommandSelector selectCmd123() {

        if (RobotContainer.m_sensor.getIRDistance() < 10)
            return CommandSelector.ONE;
        else if (RobotContainer.m_sensor.getIRDistance() > 50)
            return CommandSelector.TWO;

        else
            return CommandSelector.FOUR;
    }

    /*
     * static public Command selectCmd123_B() {
     * if (RobotContainer.m_sensor.getIRDistance()<20)
     * return new MoveLeft();
     * else if (RobotContainer.m_sensor.getIRDistance()<40)
     * return new MoveBack();
     * else
     * return new MoveRight();
     * }
     */

    // Use limit switch to select
    /*
     * static public boolean selectCmd12_SW() {
     * return RobotContainer.m_sensor.getSwitch();
     * }
     * // use IR to select
     * static public boolean selectCmd12_IR() {
     * return RobotContainer.m_sensor.getIRDistance()>30?true:false;
     * }
     */
    public MoveDirection() {

        super(
                // new MoveRobotSense(1, 0.5, 0, 0.0, 0.5,
                // ()->RobotContainer.m_sensor.getIRDistance()<60),

                // selectCmd123_B() // Didn't work

                // Select one of two commands
                // new ConditionalCommand( new MoveLeft(), new MoveRight(), MoveTest::selectCmdA
                // )

                // Select one of many commands
                // Selection command in selectCmd123
                new SelectCommand(
                        Map.ofEntries(
                                Map.entry(CommandSelector.ONE, new MoveSqLeft()),
                                Map.entry(CommandSelector.TWO, new MoveSq())),
                        MoveTest::selectCmd123));
    }
}
// Map.entry(CommandSelector.THREE, new MoveRobot(1, 1, 0, 0, 0.4))