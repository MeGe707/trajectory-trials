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

                                public static final String trajectoryBlue3ballsLeftWithTurret1File = "paths/blue-3balls-left-withTurret1.wpilib.json";

                                public static final String trajectoryBlue3ballsLeftWithTurret2File = "paths/blue-3balls-left-withTurret2.wpilib.json";

                                public static final Path trajectoryPathBlue3ballsLeftWithTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue3ballsLeftWithTurret1File);

                                public static final Path trajectoryPathBlue3ballsLeftWithTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue3ballsLeftWithTurret2File);

                        }

                        public static final class BlueTrajectoryFilesAndPathsWithOutTurret {

                                public static final String trajectoryBlue3ballsLeftWithOutTurret1File = "paths/blue-3balls-left-withOutTurret1.wpilib.json";

                                public static final String trajectoryBlue3ballsLeftWithOutTurret2File = "paths/blue-3balls-left-withOutTurret2.wpilib.json";

                                public static final Path trajectoryPathBlue3ballsLeftWithOutTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue3ballsLeftWithOutTurret1File);

                                public static final Path trajectoryPathBlue3ballsLeftWithOutTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue3ballsLeftWithOutTurret2File);

                        }
                }

                public static final class RedTrajectoryFilesAndPaths {

                        public static final class RedTrajectoryFilesAndPathsWithTurret {

                                public static final String trajectoryRed3ballsLeftWithTurret1File = "paths/red-3balls-left-withTurret1.wpilib.json";

                                public static final String trajectoryRed3ballsLeftWithTurret2File = "paths/red-3balls-left-withTurret2.wpilib.json";

                                public static final Path trajectoryPathRed3ballsLeftWithTurret1 = Filesystem
                                                .getDeployDirectory().toPath()
                                                .resolve(trajectoryRed3ballsLeftWithTurret1File);

                                public static final Path trajectoryPathRed3ballsLeftWithTurret2 = Filesystem
                                                .getDeployDirectory().toPath()
                                                .resolve(trajectoryRed3ballsLeftWithTurret2File);

                        }

                        public static final class RedTrajectoryFilesAndPathsWithOutTurret {

                                public static final String trajectoryRed3ballsLeftWithOutTurret1File = "paths/red-3balls-left-withOutTurret1.wpilib.json";

                                public static final String trajectoryRed3ballsLeftWithOutTurret2File = "paths/red-3balls-left-withOutTurret2.wpilib.json";

                                public static final Path trajectoryPathRed3ballsLeftWithOutTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed3ballsLeftWithOutTurret1File);

                                public static final Path trajectoryPathRed3ballsLeftWithOutTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed3ballsLeftWithOutTurret2File);

                        }

                }

        }

}
