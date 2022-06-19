package frc.robot.commands.auto;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
// import the commands
import frc.robot.commands.auto.MoveRobot;
import frc.robot.commands.auto.MoveRobotSense;
import frc.robot.commands.auto.MoveServo;
import frc.robot.commands.auto.RotateTest;
import frc.robot.commands.auto.MoveRobotSense.end_func;
import frc.robot.subsystems.Sensor;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class AutoMainCmd extends SequentialCommandGroup {

    public AutoMainCmd() {

        super(

                // new MoveRobot(1, 1, 0, 0,0.4),
                // new MoveRobot(2, Math.PI, 0, 0,Math.PI),
                // new MoveRobot(1, 1, 0, 0,Math.PI),
                // new MoveRobot(2, -Math.PI, 0, 0,Math.PI)
                // new MoveRobotSense(1, 1, 0, 0, 0.4, )
                /*
                 * new MoveRobot(0, 0.1, 0, 0,0.4),
                 * new MoveRobot(1, -0.1, 0, 0,0.4),
                 * new MoveRobot(0, -0.1, 0, 0,0.4),
                 * new MoveRobot(2, Math.PI*2, 0, 0,Math.PI)
                 */

                // new MoveRobotSense(1, 3, 0, 0.0, 0.5, () ->
                // RobotContainer.m_sensor.getIRDistance() < 20));
                // new MoveDirection()
                // new MoveRobotSense(1, 3, 0, 0.0, 0.5, () ->
                // RobotContainer.m_sensor.getIRDistance() < 20));
                new MoveServo(1, 90, 0, 0, 0.4));
    }

}
