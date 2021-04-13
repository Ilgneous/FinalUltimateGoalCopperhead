package org.firstinspires.ftc.teamcode.ultimategoal2020.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ultimategoal2020.hardwareMap;
import org.firstinspires.ftc.teamcode.ultimategoal2020.odometry.OdometryGlobalCoordinatePosition;

/**
 * Created by on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    hardwareMap robot = new hardwareMap();
    //Drive motors

    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String fRName = "fR", bRName = "bR", fLName = "fL", bLName = "bL";
    String verticalLeftEncoderName = fLName, verticalRightEncoderName = fRName, horizontalEncoderName = bLName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(fLName, fRName, bLName, bRName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap(String fRName, String bRName, String fLName, String bLName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        robot.fR = hardwareMap.dcMotor.get(fRName);
        robot.bR = hardwareMap.dcMotor.get(bRName);
        robot.fL = hardwareMap.dcMotor.get(fLName);
        robot.bL = hardwareMap.dcMotor.get(bLName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        robot.fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bL.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }


    public void goToPosition(double targetX, double targetY, double power, double orient, double allowError)
    {
        double xDist = targetX - globalPositionUpdate.returnXCoordinate();
        double yDist = targetY - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(xDist, yDist);
        while (opModeIsActive() && distance > allowError)
        {
            xDist = targetX - globalPositionUpdate.returnXCoordinate();
            yDist = targetY - globalPositionUpdate.returnYCoordinate();
            double angle = Math.toDegrees(Math.atan2(xDist, yDist));

            double xMove = calculateX(angle, power);
            double yMove = calculateY(angle, power);
            double correct = orient - globalPositionUpdate.returnOrientation();

            double v1 = yMove * correct;
            double v2 = xMove * correct;
            double v3 = yMove * correct;
            double v4 = xMove * correct;

            robot.fL.setPower(v1);
            robot.fR.setPower(v2);
            robot.bL.setPower(-v3);
            robot.bR.setPower(-v4);
        }

        robot.fL.setPower(0);
        robot.fR.setPower(0);
        robot.bL.setPower(0);
        robot.bR.setPower(0);
    }
    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
