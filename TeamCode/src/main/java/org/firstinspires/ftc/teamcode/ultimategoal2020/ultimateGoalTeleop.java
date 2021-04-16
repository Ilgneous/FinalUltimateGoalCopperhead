package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "ultimateGoalTeleop", group = "TeleOp")
public class ultimateGoalTeleop extends LinearOpMode {
    hardwareMap robot = new hardwareMap();
    public ElapsedTime runtime = new ElapsedTime();
    public boolean teleop = true;
    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;
    public double speed = 1;
    public int max = 570;

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(this, teleop);
        max = 570;

        waitForStart();
        while (opModeIsActive())
        {
            //robot.resetPulley();
            trigMecanum();

            //tank();

            if (gamepad1.dpad_right) {
                speed = 1;
            }
            if (gamepad1.dpad_left) {
                speed = .5;
            }

            if(gamepad2.right_trigger > 0.1){
                robot.shooter.setPower(-gamepad2.right_trigger * 0.8);
                robot.shooter2.setPower(-gamepad2.right_trigger * 0.8);
                //telemetry.addData("shooter", robot.shooter.getPower());
                //telemetry.update();
            }
            else{
                robot.shooter.setPower(0);
                robot.shooter2.setPower(0);
                //telemetry.addData("shooter", robot.shooter.getPower());
               // telemetry.update();
            }
            if(gamepad2.dpad_down){
                robot.elevator.setPower(1);
                //telemetry.addData("touch: ", robot.touch.getValue());
                //telemetry.addData("elevator", robot.elevator.getPower());
                telemetry.update();
            }
            else if(gamepad2.dpad_up && robot.touch.getState() == true){
                runtime.reset();
                robot.elevator.setPower(-1);
                telemetry.addData("time:", runtime.seconds());
                telemetry.update();


               // robot.elevator.setPower(-1);
               // telemetry.addData("touch: ", robot.touch.getValue());
                //telemetry.addData("elevator", robot.elevator.getPower());
                telemetry.update();
            }
            else
            {
                robot.elevator.setPower(0);
            }

            if (!robot.touch.getState() && !gamepad2.dpad_down)
            {
                robot.elevator.setPower(-0.1);
            }
            if(gamepad1.a){
                robot.wobble.setPosition(0.5);
                //telemetry.addData("Wobble Grabber", robot.wobble.getPosition());
                //telemetry.update();
            }
            else if(gamepad1.b){
                robot.wobble.setPosition(0.35);
                //telemetry.addData("Wobble Grabber", robot.wobble.getPosition());
                //telemetry.update();
            }
            else if(gamepad1.x)
            {
                robot.wobble.setPosition(0.8);
            }
            else if (gamepad1.y)
            {
                robot.wobble.setPosition(0);
            }

            if(gamepad1.right_trigger > 0.1){
                robot.intake.setPower(-0.825);
                //telemetry.addData("intake", robot.intake.getCurrentPosition());
                //telemetry.update();
            }
            else if(gamepad1.left_trigger > 0.1)
            {
                robot.intake.setPower(0.825);
            }
            else{
                robot.intake.setPower(0);
                //telemetry.addData("intake", robot.intake.getCurrentPosition());
                //telemetry.update();
            }

            if (gamepad1.right_bumper)
            {
                robot.brush.setPower(-1);
            }
            else if(gamepad1.left_bumper)
            {
                robot.brush.setPower(1);
            }
            else
            {
                robot.brush.setPower(0);
                //telemetry.addData("second intake: ", robot.brush.getPower());
                //telemetry.update();
            }





            /*if(gamepad2.x){
            elevator.setPosition(1);
            telemetry.addData("Elevator", elevator.getPosition());
            telemetry.update();
            }
            else if(gamepad2.y){
            elevator.setPosition(0);
            telemetry.addData("Elevator", elevator.getPosition());
            telemetry.update();
            } */

            if (robot.pulley1.getCurrentPosition() == 0)
            {
                robot.resetPivot();
            }

            /*
            if (gamepad2.right_bumper && robot.powerTouch.getState() == true)
            {
                robot.pulley1.setPower(0.4);
            }
            else if (gamepad2.left_bumper && robot.highTouch.getState() == true)
            {
                robot.pulley1.setPower(0.4);
            }*/

            if(gamepad2.left_bumper && robot.highTouch.getState())
            {
                robot.pulley1.setPower(.6);
            }
            else if(gamepad2.right_bumper && robot.powerTouch.getState())
            {
                robot.pulley1.setPower(.6);
            }
            else if(gamepad2.left_trigger > 0.1)
            {
                robot.pulley1.setPower(-0.1);
            }
            else
            {
                robot.pulley1.setPower(0);
                robot.pulley1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            /*if (gamepad1.y)
            {
                max += 50;
            }

            if (gamepad1.dpad_up && robot.highTouch.getState() == true)
            {
                robot.pulley1.setPower(0.4);
            }
            else if (gamepad1.dpad_down)
            {
                robot.pulley1.setPower(-0.1);
            }
            else
            {
                robot.pulley1.setPower(0);
                robot.pulley1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (robot.highTouch.getState() == false && !gamepad1.dpad_down)
            {
                robot.pulley1.setPower(0.1);
            }*/

          /*  if(gamepad1.dpad_up && robot.pulley1.getCurrentPosition() < max){
                //robot.resetPulley();
                robot.pulley1.setPower(0.4);
                telemetry.addData("pivot: ", robot.pulley1.getCurrentPosition());
                telemetry.update();
            }
            else if (gamepad1.dpad_down ){
                robot.pulley1.setPower(-0.1);
                telemetry.addData("pivot: ", robot.pulley1.getCurrentPosition());
                telemetry.update();
            }
            else {
                robot.pulley1.setPower(0);
                robot.pulley1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }*/

            if(gamepad2.a){
                robot.pusher.setPower(1);
            }
            else if(gamepad2.b){
                robot.pusher.setPower(-1);
            }

            else if(gamepad2.x) {
                for (int i = 0; i < 3; i++) {
                    robot.pusher.setPower(1);
                    sleep(600);
                    robot.pusher.setPower(-1);
                    sleep(500);
                }
            }

            else
            {
                robot.pusher.setPower(0);
            }
        }
    }

    public void trigMecanum() { //turning and moving method
        rightstickx = Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x; //-gamepad1.right_stick_x
        leftstickx = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);

        leftstickyfront = Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y;
        leftstickyback = Math.abs(gamepad1.left_stick_y) * -gamepad1.left_stick_y;

        double rFront = Math.hypot(rightstickx, leftstickyfront);
        double rBack = Math.hypot(rightstickx, leftstickyback);

        double robotAngleFront = Math.atan2(leftstickyfront, rightstickx) - Math.PI / 4;
        double robotAngleBack = Math.atan2(leftstickyback, rightstickx) - Math.PI / 4;

        double rightX = leftstickx;

        final double v1 = rFront * Math.cos(robotAngleFront) + rightX; //flip all signs
        final double v2 = rFront * Math.sin(robotAngleFront) - rightX;
        final double v3 = rBack * Math.sin(robotAngleBack) + rightX;
        final double v4 = rBack * Math.cos(robotAngleBack) - rightX;

        /*telemetry.addData("fl", v1);
        telemetry.addData("fR", v2);
        telemetry.addData("bL", v3);
        telemetry.addData("bR", v4);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData("Right X", rightX);
        telemetry.update();*/

        robot.fL.setPower(v1 * speed);
        robot.fR.setPower(v2 * speed);
        robot.bL.setPower(-v3 * speed);
        robot.bR.setPower(-v4 * speed);

        /*if(Math.abs(gamepad1.right_stick_y) > 0.1){ //strafing method
            fL.setPower(gamepad1.right_stick_y);
            bL.setPower(-gamepad1.right_stick_y);
            fR.setPower(gamepad1.right_stick_y);
            bR.setPower(-gamepad1.right_stick_y);
        }
        /*if(Math.abs(gamepad1.left_stick_x) > 0.1|| Math.abs(gamepad1.left_stick_y) > 0.1){
            robot.fL.setPower(v1);
            robot.fR.setPower(-v2);
            robot.bL.setPower(-v3);// * .79);
            robot.bR.setPower(v4);// * .79);
        }
        else if(Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2){
            robot.fL.setPower(-v1);
            robot.fR.setPower(v2);
            robot.bL.setPower(v3);// * .79);
            robot.bR.setPower(-v4);// * .79);
        }
        else{
            robot.fL.setPower(0);
            robot.fR.setPower(0);
            robot.bL.setPower(0);// * .79);
            robot.bR.setPower(0);// * .79);
        }*/
    }

    public void tank()
    {
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;

        if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_y) > 0.1)
        {
            robot.fL.setPower(left * speed);
            robot.fR.setPower(right * speed);
            robot.bL.setPower(left * speed);
            robot.bR.setPower(right * speed);
        }
        else
        {
            robot.fL.setPower(-gamepad1.left_stick_x * speed);
            robot.fR.setPower(gamepad1.right_stick_x * speed);
            robot.bL.setPower(gamepad1.right_stick_x * speed);
            robot.bR.setPower(-gamepad1.left_stick_x * speed);
        }
    }


    public void trigTankMecanum() { //turning and moving method
        rightstickx = Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x; //-gamepad1.right_stick_x
        leftstickx = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);

        leftstickyfront = Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y;
        leftstickyback = Math.abs(gamepad1.left_stick_y) * -gamepad1.left_stick_y;

        double rFront = Math.hypot(rightstickx, leftstickyfront);
        double rBack = Math.hypot(rightstickx, leftstickyback);

        double robotAngleFront = Math.atan2(leftstickyfront, rightstickx) - Math.PI / 4;
        double robotAngleBack = Math.atan2(leftstickyback, rightstickx) - Math.PI / 4;

        double rightX = leftstickx;

        final double v1 = rFront * Math.cos(robotAngleFront) + rightX; //flip all signs
        final double v2 = rFront * Math.sin(robotAngleFront) - rightX;
        final double v3 = rBack * Math.sin(robotAngleBack) + rightX;
        final double v4 = rBack * Math.cos(robotAngleBack) - rightX;

        /*telemetry.addData("fl", v1);
        telemetry.addData("fR", v2);
        telemetry.addData("bL", v3);
        telemetry.addData("bR", v4);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData("Right X", rightX);
        telemetry.update();*/

        robot.fL.setPower(v1 * speed);
        robot.fR.setPower(v2 * speed);
        robot.bL.setPower(-v3 * speed);
        robot.bR.setPower(-v4 * speed);

        /*if(Math.abs(gamepad1.right_stick_y) > 0.1){ //strafing method
            fL.setPower(gamepad1.right_stick_y);
            bL.setPower(-gamepad1.right_stick_y);
            fR.setPower(gamepad1.right_stick_y);
            bR.setPower(-gamepad1.right_stick_y);
        }
        /*if(Math.abs(gamepad1.left_stick_x) > 0.1|| Math.abs(gamepad1.left_stick_y) > 0.1){
            robot.fL.setPower(v1);
            robot.fR.setPower(-v2);
            robot.bL.setPower(-v3);// * .79);
            robot.bR.setPower(v4);// * .79);
        }
        else if(Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2){
            robot.fL.setPower(-v1);
            robot.fR.setPower(v2);
            robot.bL.setPower(v3);// * .79);
            robot.bR.setPower(-v4);// * .79);
        }
        else{
            robot.fL.setPower(0);
            robot.fR.setPower(0);
            robot.bL.setPower(0);// * .79);
            robot.bR.setPower(0);// * .79);
        }*/
    }
}
