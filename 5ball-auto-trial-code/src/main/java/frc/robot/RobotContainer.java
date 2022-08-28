
package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import frc.robot.Constants.DriveTrainConstants;

import frc.robot.TrajectoriesContainer;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.BlueTrajectoryFilesAndPaths.BlueTrajectoryFilesAndPathsWithOutTurret;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.BlueTrajectoryFilesAndPaths.BlueTrajectoryFilesAndPathsWithTurret;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.RedTrajectoryFilesAndPaths.RedTrajectoryFilesAndPathsWithOutTurret;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.RedTrajectoryFilesAndPaths.RedTrajectoryFilesAndPathsWithTurret;
import frc.robot.commands.DoNothing;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.FiveBallCommand;

public class RobotContainer {

        final frc.robot.subsystems.DriveSubsystem m_robotDrive = new frc.robot.subsystems.DriveSubsystem();
        XboxController m_driverController = new XboxController(1);

        SendableChooser<Command> autoChooser;

        public RobotContainer() throws IOException {

                configureButtonBindings();

                autoChooser = new SendableChooser<>();
                autoChooser.setDefaultOption("Do Nothing", new DoNothing(m_robotDrive));

                autoChooser.addOption("Blue-Left-Turret+", new FiveBallCommand(new FollowPathCommand(
                                BlueTrajectoryFilesAndPathsWithTurret.trajectoryPathBlue5ballsLeftWithTurret1,
                                m_robotDrive),
                                new FollowPathCommand(
                                                BlueTrajectoryFilesAndPathsWithTurret.trajectoryPathBlue5ballsLeftWithTurret2,
                                                m_robotDrive),
                                new FollowPathCommand(
                                                BlueTrajectoryFilesAndPathsWithTurret.trajectoryPathBlue5ballsLeftWithTurret3,
                                                m_robotDrive),
                                new FollowPathCommand(
                                                BlueTrajectoryFilesAndPathsWithTurret.trajectoryPathBlue5ballsLeftWithTurret4,
                                                m_robotDrive)));

                autoChooser.addOption("Red-Left-Turret+", new FiveBallCommand(new FollowPathCommand(
                                RedTrajectoryFilesAndPathsWithTurret.trajectoryPathRed5ballsLeftWithTurret1,
                                m_robotDrive),
                                new FollowPathCommand(
                                                RedTrajectoryFilesAndPathsWithTurret.trajectoryPathRed5ballsLeftWithTurret2,
                                                m_robotDrive),
                                new FollowPathCommand(
                                                RedTrajectoryFilesAndPathsWithTurret.trajectoryPathRed5ballsLeftWithTurret3,
                                                m_robotDrive),
                                new FollowPathCommand(
                                                RedTrajectoryFilesAndPathsWithTurret.trajectoryPathRed5ballsLeftWithTurret4,
                                                m_robotDrive)));

                SmartDashboard.putData(autoChooser);

        }

        private void configureButtonBindings() {

                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
                                .whenReleased(() -> m_robotDrive.setMaxOutput(1));
        }

        public Command getAutonomousCommand() {

                return null;
        }
}
