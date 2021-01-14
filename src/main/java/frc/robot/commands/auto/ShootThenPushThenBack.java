package frc.robot.commands.auto;

import static frc.robot.Constants.ShooterConstants.shooterClosedLoopThreshold;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.shooter.ShootBallsClosedLoop;
import frc.robot.commands.shooter.ShootBallsCommandGroupWithSpeed;
import frc.robot.commands.shooter.TestingShootBallsCommandGroup;
import java.util.ArrayList;

public class ShootThenPushThenBack extends LowGearAuto {

  public ShootThenPushThenBack() {

    ArrayList<Pose2d> a_b_PushRobotForward = new ArrayList<>();
    ArrayList<Pose2d> b_c_MoveBack = new ArrayList<>();

    // Push the robot forward
    a_b_PushRobotForward.add(new Pose2d(0, 0, new Rotation2d(0)));
    a_b_PushRobotForward.add(new Pose2d(0.5, 0, new Rotation2d(0)));

    // Move backwards towards the switches
    b_c_MoveBack.add(new Pose2d(0.5, 0, new Rotation2d(0)));
    b_c_MoveBack.add(new Pose2d(-1.25, 0, new Rotation2d(0)));

    // This config is false so we move forwards
    TrajectoryConfig a_bConfig = driveTrain.createTrajectoryConfig(false);

    // This config is true so we move reversed
    TrajectoryConfig b_cConfig = driveTrain.createTrajectoryConfig(true);

    var pushRobotForward = TrajectoryGenerator.generateTrajectory(a_b_PushRobotForward, a_bConfig);

    var moveBack = TrajectoryGenerator.generateTrajectory(b_c_MoveBack, b_cConfig);


    addCommands(
        new InstantCommand(driveTrain::zeroEncoder),
        new InstantCommand(driveTrain::resetOdometry),
        new ShootBallsCommandGroupWithSpeed(28).withTimeout(6),
        driveTrain.getAutonomousCommandFromTrajectory(pushRobotForward),
        driveTrain.getAutonomousCommandFromTrajectory(moveBack)
    );
  }
}
