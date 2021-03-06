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
public class MoveTest extends SequentialCommandGroup {

    /*
     * static public CommandSelector selectCmd123() {
     * if (RobotContainer.m_sensor.getIRDistance()<20)
     * return CommandSelector.ONE;
     * else if (RobotContainer.m_sensor.getIRDistance()<40)
     * return CommandSelector.TWO;
     * else
     * return CommandSelector.THREE;
     * }
     */
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
    static public boolean selectCmd12_SW() {
        return RobotContainer.m_sensor.getSwitch();
    }

    // use IR to select
    static public boolean selectCmd12_IR() {
        return RobotContainer.m_sensor.getIRDistance() > 30 ? true : false;
    }

    static public int selectCmd123() {
        if (RobotContainer.m_sensor.getIRDistance() < 50)
            return 1;
        // else if (RobotContainer.m_sensor.getIRDistance() > 50)
        // return CommandSelector.TWO;
        else
            return 2;
    }

    static public int selectCmd12() {
        if (RobotContainer.m_sensor.getIRDistance() < 10)
            return 1;
        else
            return 2;

    }

    public MoveTest() {

        super(
                // new MoveRobotSense(1, 0.5, 0, 0.0, 0.5,
                // ()->RobotContainer.m_sensor.getIRDistance()<60),

                // selectCmd123_B() // Didn't work

                // Select one of two commands
                // new ConditionalCommand( new MoveLeft(), new MoveRight(), MoveTest::selectCmdA
                // )

                // Select one of many commands
                // Selection command in selectCmd123
                /*
                 * new SelectCommand(
                 * Map.ofEntries(
                 * Map.entry(CommandSelector.ONE, new MoveSq()),
                 * Map.entry(CommandSelector.TWO, new MoveSq()),
                 * Map.entry(CommandSelector.THREE, new MoveSq()) ),
                 * MoveTest::selectCmd123
                 * )
                 */

                /*
                 * new SelectCommand(
                 * Map.ofEntries(
                 * Map.entry(CommandSelector.ONE, new MoveRight()),
                 * Map.entry(CommandSelector.TWO, new MoveLeft())
                 * //Map.entry(CommandSelector.THREE, new MoveRobot(1, 0.5, 0, 0.0, 0.5))
                 * ),
                 * MoveTest::selectCmd12
                 * )
                 */
                new SelectCommand(
                        Map.ofEntries(
                                Map.entry(1, new MoveSqLeft()),

                                Map.entry(2, new MoveSq())

                        ),
                        MoveTest::selectCmd12)

        );

    }
}
