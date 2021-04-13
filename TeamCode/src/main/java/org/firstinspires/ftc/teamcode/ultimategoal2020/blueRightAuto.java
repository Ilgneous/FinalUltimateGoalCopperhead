package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "blueRightAuto", group = "Autonomous")
public class blueRightAuto extends LinearOpMode {

    hardwareMap robot = new hardwareMap();
    //WebCamVision vision = null;
    //ElapsedTime time = new ElapsedTime();
    //WebCamVision vision = new WebCamVision(this);

    // Testing
    @Override
    public void runOpMode() throws InterruptedException {

        //stuff that happens after init is pressed
        WebCamVision vision = new WebCamVision(this);
        robot.init(this);
        robot.wobble.setPosition(0.5);
        waitForStart();

        while (opModeIsActive()) {

            //call functions

            //robot.strafeLeft(20, 0.5);
            int height = vision.findStackHeight();
            telemetry.addData("stack height: ", height);
            telemetry.update();
            if (height == 0)
            {
                robot.strafeRightGyro(79, 0.4);
                robot.goStraightGyro(30, 0.4, 3);
                robot.wobble.setPosition(0);


                robot.goStraightGyro(12, 0.4, 3);
                robot.strafeRightGyro(70, 0.4);
                robot.wobble.setPosition(0);
                robot.strafeRightGyro(5, -0.4);
            }
            else if(height == 1)
            {
                robot.goStraightGyro(12, 0.4, 3);
                robot.strafeRightGyro(95, 0.4);
                robot.wobble.setPosition(0);
                robot.strafeRightGyro(23, -0.4);
            }
            else if(height == 4)
            {
                robot.goStraightGyro(12, 0.4, 3);
                robot.strafeRightGyro(118, 0.4);
                robot.wobble.setPosition(0);
                robot.strafeRightGyro(46, -0.4);
            }
            sleep(3000000);

            // strafe left
            // turn right
            //dump.setPosition(1);
            // go backwards
            // strafe left


        }

        telemetry.addLine("done");
        telemetry.update();
    }
}



