package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "goForward", group = "Autonomous")
public class goForward extends LinearOpMode {

    hardwareMap robot = new hardwareMap();
    public ElapsedTime runtime = new ElapsedTime();
    int height;
    //WebCamVision vision = null;
    //ElapsedTime time = new ElapsedTime();
    //WebCamVision vision = new WebCamVision(this);

    // Testing
    @Override
    public void runOpMode() throws InterruptedException {

        //stuff that happens after init is pressed
        WebCamVision vision = new WebCamVision(this);
        robot.init(this);





        waitForStart();

        while (opModeIsActive()) {

            //call functions

            //robot.strafeLeft(20, 0.5);
            /*int height = vision.findStackHeight();
            telemetry.addData("stack height: ", height);
            telemetry.update();*/

            robot.goStraightGyro(20, 0.7, 100);
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



