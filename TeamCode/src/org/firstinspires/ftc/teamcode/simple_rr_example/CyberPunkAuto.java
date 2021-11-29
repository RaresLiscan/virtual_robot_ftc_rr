package org.firstinspires.ftc.teamcode.simple_rr_example;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.rr_quickstart_examples.drive.SampleMecanumDrive;

@Autonomous(name = "TestAutonomous")
public class CyberPunkAuto extends LinearOpMode {

    SampleMecanumDrive drive;
    ElapsedTime runtime = new ElapsedTime();


    void roadRunnerSplines() {
        drive.updatePoseEstimate();
        Pose2d currentPose;drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();
        Trajectory traj1 = new TrajectoryBuilder(currentPose, drive.constraints)
                .splineTo(new Vector2d(3.3578631250134747, -47.20390951776473), 6.282717808272803)
                .build();

        drive.followTrajectory(traj1);
        drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();
        Trajectory traj2 = new TrajectoryBuilder(currentPose, drive.constraints)
                .splineTo(new Vector2d(30.749608710148784, -63.018850963136224), 6.282717808272803)
                .build();

        drive.followTrajectory(traj2);
        drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();
        Trajectory traj3 = new TrajectoryBuilder(currentPose, drive.constraints)
                .splineTo(new Vector2d(8.973810883073257, -7.103362693065838), 0.7423882639733037)
                .build();

        drive.followTrajectory(traj3);
        drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();
        Trajectory traj4 = new TrajectoryBuilder(currentPose, drive.constraints)
                .splineTo(new Vector2d(-19.354943597606407, -5.296527096253905), 0.4745113903859597)
                .build();

        drive.followTrajectory(traj4);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        roadRunnerSplines();

    }
}
