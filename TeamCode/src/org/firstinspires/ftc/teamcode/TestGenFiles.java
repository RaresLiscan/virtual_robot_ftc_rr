package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.rr_quickstart_examples.drive.SampleMecanumDrive;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "TestGenFiles")
public class TestGenFiles extends LinearOpMode {

    SampleMecanumDrive drive;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException, IOException {
        drive = new SampleMecanumDrive(hardwareMap);
        FileWriter myWriter;

        int noOfTrajectories = 0;
        waitForStart();
        myWriter = new FileWriter("Test.java");
        myWriter.write("void roadRunnerSplines() {\ndrive.updatePoseEstimate();\n" +
                "        Pose2d currentPose;");
        runtime.reset();
        while (opModeIsActive()) {

            drive.updatePoseEstimate();
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double sF = forward + strafe + rotate;
            double sS = forward - strafe + rotate;
            double dF = forward - strafe - rotate;
            double dS = forward + strafe - rotate;

            drive.setMotorPowers(sF, sS, dS, dF);

            if (gamepad1.x && runtime.seconds() > 1) {
                runtime.reset();
                Pose2d pose = drive.getPoseEstimate();
                noOfTrajectories ++;
                myWriter.write("drive.updatePoseEstimate();\n" +
                        "        currentPose = drive.getPoseEstimate();\n" +
                        "       Trajectory traj" + noOfTrajectories + " = new TrajectoryBuilder(currentPose, drive.constraints)\n" +
                        "                .splineTo(new Vector2d(" + pose.getX() + ", " + pose.getY() + "), " + pose.getHeading() + ")\n" +
                        "                .build();\n" +
                        "\n" +
                        "        drive.followTrajectory(traj" + noOfTrajectories + ");\n");
            }

            telemetry.addData("Pose X: ", drive.getPoseEstimate().getX());
            telemetry.addData("Pose Y: ", drive.getPoseEstimate().getY());
            telemetry.addData("Pose Heading: ", drive.getPoseEstimate().getHeading());

        }
        myWriter.write("\n}");
        myWriter.close();
    }
}
