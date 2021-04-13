package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class odomBlueLeftAuto0 extends LinearOpMode {

    hardwareMap robot = new hardwareMap();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(this);


        Pose2d startPose = new Pose2d(-64, -48, Math.toRadians(-90));

        Pose2d nextPose = new Pose2d(0, -48, Math.toRadians(0));

        Pose2d thirdPose = new Pose2d(-41, -20, Math.toRadians(0));

        Pose2d fourthPose = new Pose2d(0, -20, Math.toRadians(0));

        Pose2d fifthPose = new Pose2d(17.5, -20, Math.toRadians(0));

        drive.setPoseEstimate(startPose);



        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(62)
                //.strafeTo(new Vector2d(0, -48))
                //.lineToSplineHeading(new Pose2d(-1, 13.5, Math.toRadians(-135)))
                //.splineToLinearHeading(new Pose2d(0, 13.5, Math.toRadians(45)), Math.toRadians(0))
                //.strafeRight(60)
                //.lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(-45)))
                .build();


        drive.setPoseEstimate(nextPose);


        Trajectory traj2 = drive.trajectoryBuilder(nextPose)
                .strafeLeft(36)
                .build();

        drive.setPoseEstimate(thirdPose);

        Trajectory traj3 = drive.trajectoryBuilder(thirdPose)
                .back(45)
                .build();

        drive.setPoseEstimate(fourthPose);

        Trajectory traj4 = drive.trajectoryBuilder(fourthPose)
                .back(22.5)
                .build();

        drive.setPoseEstimate(fifthPose);

        Trajectory traj5 = drive.trajectoryBuilder(fifthPose)
                .strafeLeft(15)
                .build();
       /* Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-30, -20, Math.toRadians(-270)))
                        .build();*/

        /*Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(9, -25, Math.toRadians(-180)))
                .build();*/


       /* Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(-270)))
                .build();*/

        waitForStart();



        if (isStopRequested()) return;


        robot.wobble.setPosition(0.45);
        sleep(100);
        robot.wobble.setPosition(0);
        sleep(1000);

        drive.followTrajectory(traj1);
        //sleep(10000);


        robot.wobble.setPosition(0.52);
        sleep(500);
        //robot.strafeLeftGyro(2, 0.5, 3);
        robot.wobble.setPosition(0.8);

        drive.turn(Math.toRadians(145));

        robot.wobble.setPosition(0.52);
        //sleep(500);

        //robot.wobble.setPosition(0.52);

        drive.followTrajectory(traj2);

        sleep(500);

        robot.wobble.setPosition(0);

        sleep(1000);

        drive.turn(Math.toRadians(123));

        runtime.reset();

        drive.followTrajectory(traj3);

        robot.pusherOut();


        runtime.reset();

        //robot.pivotHigh(500, 0.3);
        //robot.newPivotHigh(0.3);

        robot.shootReady();
        robot.pusherIn();

        robot.autoShoot(-900);
        robot.pusherIn();
        robot.resetShooter();
        robot.autoShoot(-900);
        robot.pusherIn();
        robot.resetShooter();
        robot.autoShoot(-900);
        robot.pusherIn();
       // robot.resetShooter();
        //robot.pusherOutShort();

        //robot.stopShoot();

        drive.followTrajectory(traj4);

        drive.followTrajectory(traj5);



        sleep(2000);

       /* drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        ); */
    }
}
