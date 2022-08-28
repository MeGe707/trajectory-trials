package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeballCommand extends SequentialCommandGroup {

    public ThreeballCommand(FollowPathCommand followFirstPath, FollowPathCommand followSecondPath) {

        addCommands(
                followFirstPath,
                followSecondPath);
    }

}
