package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.BlueTrajectoryFilesAndPaths.BlueTrajectoryFilesAndPathsWithOutTurret;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.BlueTrajectoryFilesAndPaths.BlueTrajectoryFilesAndPathsWithTurret;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.RedTrajectoryFilesAndPaths.RedTrajectoryFilesAndPathsWithOutTurret;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.RedTrajectoryFilesAndPaths.RedTrajectoryFilesAndPathsWithTurret;

public final class TrajectoriesContainer {

        public static final class TrajectoryFilesAndPaths {

                public static final class BlueTrajectoryFilesAndPaths {

                        public static final class BlueTrajectoryFilesAndPathsWithTurret {

                                public static final String trajectoryBlue5ballsLeftWithTurret1File = "paths/blue-5balls-left-withTurret1.wpilib.json";

                                public static final String trajectoryBlue5ballsLeftWithTurret2File = "paths/blue-5balls-left-withTurret2.wpilib.json";

                                public static final String trajectoryBlue5ballsLeftWithTurret3File = "paths/blue-5balls-left-withTurret3.wpilib.json";

                                public static final String trajectoryBlue5ballsLeftWithTurret4File = "paths/blue-5balls-left-withTurret4.wpilib.json";

                                public static final Path trajectoryPathBlue5ballsLeftWithTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue5ballsLeftWithTurret1File);

                                public static final Path trajectoryPathBlue5ballsLeftWithTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue5ballsLeftWithTurret2File);

                                public static final Path trajectoryPathBlue5ballsLeftWithTurret3 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue5ballsLeftWithTurret3File);

                                public static final Path trajectoryPathBlue5ballsLeftWithTurret4 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue5ballsLeftWithTurret4File);

                        }

                        public static final class BlueTrajectoryFilesAndPathsWithOutTurret {

                        }
                }

                public static final class RedTrajectoryFilesAndPaths {

                        public static final class RedTrajectoryFilesAndPathsWithTurret {

                                public static final String trajectoryRed5ballsLeftWithTurret1File = "paths/red-5balls-left-withTurret1.wpilib.json";

                                public static final String trajectoryRed5ballsLeftWithTurret2File = "paths/red-5balls-left-withTurret2.wpilib.json";

                                public static final String trajectoryRed5ballsLeftWithTurret3File = "paths/red-5balls-left-withTurret3.wpilib.json";

                                public static final String trajectoryRed5ballsLeftWithTurret4File = "paths/red-5balls-left-withTurret4.wpilib.json";

                                public static final Path trajectoryPathRed5ballsLeftWithTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed5ballsLeftWithTurret1File);

                                public static final Path trajectoryPathRed5ballsLeftWithTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed5ballsLeftWithTurret2File);

                                public static final Path trajectoryPathRed5ballsLeftWithTurret3 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed5ballsLeftWithTurret3File);

                                public static final Path trajectoryPathRed5ballsLeftWithTurret4 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed5ballsLeftWithTurret4File);

                        }

                        public static final class RedTrajectoryFilesAndPathsWithOutTurret {

                        }

                }

        }

}
