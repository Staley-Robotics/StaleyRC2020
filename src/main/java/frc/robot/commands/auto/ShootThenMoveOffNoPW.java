package frc.robot.commands.auto;

import static frc.robot.Constants.ShooterConstants.shooterClosedLoopThreshold;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.shooter.ShootBallsClosedLoop;
import java.util.ArrayList;

public class ShootThenMoveOffNoPW extends LowGearAuto {

  public ShootThenMoveOffNoPW() {

    ArrayList<Pose2d> list = new ArrayList<>();

    list.add(new Pose2d(0, 0, new Rotation2d(0)));
    list.add(new Pose2d(-1.0, 0, new Rotation2d(0)));

    TrajectoryConfig trajectoryConfig = driveTrain.createTrajectoryConfig(true);

    var trajectory = TrajectoryGenerator.generateTrajectory(list, trajectoryConfig);

    //
    //    addCommands(
    //        new ShootBallsClosedLoop(0, shooterClosedLoopThreshold).withTimeout(4),
    //        new RunCommand(
    //            () -> driveTrain
    //                .runDriveTrain(0.6)).withTimeout(1)
    //    );

    addCommands(
        new InstantCommand(driveTrain::zeroEncoder),
        new InstantCommand(driveTrain::resetOdometry),
        new ShootBallsClosedLoop(0, shooterClosedLoopThreshold).withTimeout(4),
        driveTrain.getAutonomousCommandFromTrajectory(trajectory)
    );
  }
}
