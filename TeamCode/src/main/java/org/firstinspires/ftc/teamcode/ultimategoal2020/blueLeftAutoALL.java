package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "blueLeftAutoALL", group = "Autonomous")
public class blueLeftAutoALL extends LinearOpMode {

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






        //height = vision.findStackHeight();
        //telemetry.addData("stack height: ", height);
        //telemetry.update();

        while(!isStarted())
        {
            String ring1 = vision.rbgVals(1185, 519);
            String ring2 = vision.rbgVals(1185, 449);

            height = vision.findStackHeight();
            telemetry.addData("stack height: ", height);
            telemetry.addData("ring 1: ", ring1);
            telemetry.addData("ring 2: ", ring2);
            telemetry.update();
        }


        waitForStart();

        while (opModeIsActive()) {



            //call functions

            //robot.strafeLeft(20, 0.5);
            /*int height = vision.findStackHeight();
            telemetry.addData("stack height: ", height);
            telemetry.update();*/

            if (height == 0)
            {
                robot.wobble.setPosition(0.4);
                sleep(100);
                robot.wobble.setPosition(0.8);
                sleep(1000);
                robot.goStraightGyro(4, 0.7, 3);
                robot.strafeRightGyro(56, 0.7);
                robot.wobble.setPosition(0.52);
                sleep(500);
                //robot.strafeLeftGyro(2, 0.5, 3);
                robot.wobble.setPosition(0.12);
                sleep(500);
                robot.turnPID2(-90, 0.81 /90, 0, 0, 2);
                robot.strafeLeftGyro(21, 0.7, 5);
                robot.turnRight(8, 0.5);
                robot.goStraightGyro(2, -0.7, 1.5); //comment out maybe




                telemetry.addLine("strafe done");
                robot.pusherOut();


                runtime.reset();

                //robot.pivotHigh(500, 0.3);
                robot.newPivotHigh(0.3);

                robot.shootReady();
                robot.pusherIn();

                robot.autoShoot(-900);
                robot.pusherIn();
                robot.resetShooter();
                robot.autoShoot(-900);
                robot.pusherIn();
                robot.resetShooter();
                robot.pusherOutShort();
                robot.autoShoot(-900);

                robot.stopShoot();

                telemetry.addLine("here 2");
                telemetry.update();

                robot.goStraightGyro(10, -0.7, 4);
            }
            else if (height == 1)
            {
                robot.wobble.setPosition(0.4);
                sleep(200);
                robot.wobble.setPosition(0.8);
                sleep(1000);
                robot.goStraightGyro(5, 0.7, 1.5);
                sleep(500);
                robot.strafeRightGyro(76, 0.7);
                sleep(500);
                robot.goStraightGyro(15, -0.5, 1.7);
                sleep(500);
                robot.wobble.setPosition(0.52);
                sleep(500);
                robot.wobble.setPosition(0.12);

                sleep(500);
                robot.strafeLeftGyro(20, 0.5, 1.5);
                sleep(500);
                robot.turnPID2(-90, 0.81 /90, 0, 0, 2);

                sleep(500);

                robot.strafeLeftGyro(7, 0.7, 1);
                //robot.goStraightGyro(2, -0.7, 1.5);


                robot.pusherOut();
                robot.newPivotHigh(0.3);
                robot.shootReady();
                robot.pusherIn();

                robot.autoShoot(-900);
                robot.pusherIn();
                robot.resetShooter();
                robot.autoShoot(-900);
                robot.pusherIn();
                robot.resetShooter();
                robot.pusherOutShort();
                robot.autoShoot(-900);

                robot.stopShoot();
                robot.goStraightGyro(5, -0.7, 1.2);
            }
            else if(height == 4)
            {
                robot.wobble.setPosition(0.4);
                sleep(100);
                robot.wobble.setPosition(0.8);
                sleep(1000);
                robot.goStraightGyro(7, 0.7, 1.5);
                robot.strafeRightGyro(98, 0.7);
                robot.wobble.setPosition(0.52);
                sleep(500);
                robot.wobble.setPosition(0.12);
                sleep(500);
                robot.strafeLeftGyro(35, 0.7, 2.5);
                robot.turnPID2(-90, 0.81 /90, 0, 0, 1.5);
                robot.strafeLeftGyro(25, 0.4, 3);

                robot.pusherOut();
                robot.newPivotHigh(0.3);
                robot.shootReady();
                robot.pusherIn();

                robot.autoShoot(-900);
                robot.pusherIn();
                robot.resetShooter();
                robot.autoShoot(-900);
                robot.pusherIn();
                robot.resetShooter();
                robot.pusherOutShort();
                robot.autoShoot(-900);

                robot.stopShoot();
                robot.goStraightGyro(5, -0.7, 4);
            }
            else
            {
                telemetry.addLine("yikes");
                telemetry.update();
            }







            //robot.autoShoot();
            /*if (height == 0)
            {
                robot.goStraightGyro(12, 0.4, 3);
                robot.strafeRightGyro(70, 0.4);
                robot.wobble.setPosition(0);
                robot.strafeRightGyro(5, -0.4);
                robot.turnPID(-90, 0.79/90, 0, 0);
                robot.pivotHigh(500, 0.3);
                robot.shoot(1000, 0.8);
                robot.turnPID(180, 0.79/90, 0, 0);
                robot.intake.setPower(1);
                robot.goStraightGyro(15, 0.25, 6);
                robot.turnAndPrep(180, 0.79/90);
                robot.shoot(1000, 0.8);
                robot.goStraightGyro(10, 0.4, 3);
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
            }*/
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



