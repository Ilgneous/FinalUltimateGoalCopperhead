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
public class testPoseOdom extends LinearOpMode {

    hardwareMap robot = new hardwareMap();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(this);


        Pose2d startPose = new Pose2d(-64, -48, Math.toRadians(-90));


        drive.setPoseEstimate(startPose);



   /*     Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(62)
                //.strafeTo(new Vector2d(0, -48))
                //.lineToSplineHeading(new Pose2d(-1, 13.5, Math.toRadians(-135)))
                //.splineToLinearHeading(new Pose2d(0, 13.5, Math.toRadians(45)), Math.toRadians(0))
                //.strafeRight(60)
                //.lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(-45)))
                .build();*/



        waitForStart();



        if (isStopRequested()) return;


        //sleep(10000);






        sleep(2000);

       /* drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        ); */
    }
}
