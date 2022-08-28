package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FiveBallCommand extends SequentialCommandGroup {

    public FiveBallCommand(FollowPathCommand followFirstPath, FollowPathCommand followSecondPath,
            FollowPathCommand followThirdPath, FollowPathCommand followFourthPath) {

        addCommands(
                followFirstPath,
                followSecondPath,
                followThirdPath,
                followFourthPath);
    }

}
