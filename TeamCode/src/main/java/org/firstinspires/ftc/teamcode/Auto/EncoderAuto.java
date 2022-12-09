package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="EncoderAuto")
//"tag" that is displayed on driver hub
public class EncoderAuto extends LinearOpMode {
    //creating robot object
    //the project name will be different, make sure to change this line below to proper name and also in the imports
    public ProjectUdon robot = new ProjectUdon();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);
        robot.frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //wait for start button to be pressed
        waitForStart();

        //write autonomous code here

        //record number of encoder counts for certain distances
        //"blank" encoder counts = 1 tile
        //"blank" encoder counts = 90 degree turn

        //speed, leftCounts, rightCounts
        encoderDrive (0.5, 925, 925, 925, 925);
        stop(1000);
        encoderDrive (0.5, -412, 412, -412, 412);


    }


    //encoder method
    public void encoderDrive(double speed,
                             double frontLeftCounts, double frontRightCounts, double backLeftCounts, double backRightCounts) {
        int newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontleft.getCurrentPosition() + (int) (frontLeftCounts);
            newFrontRightTarget = robot.frontright.getCurrentPosition() + (int) (frontRightCounts);
            newBackLeftTarget = robot.backleft.getCurrentPosition() + (int) (backLeftCounts);
            newBackRightTarget = robot.backright.getCurrentPosition() + (int) (backRightCounts);
            robot.frontleft.setTargetPosition(newFrontLeftTarget);
            robot.frontright.setTargetPosition(newFrontRightTarget);
            robot.backleft.setTargetPosition(newBackLeftTarget);
            robot.backright.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontleft.setPower(Math.abs(speed));
            robot.frontright.setPower(Math.abs(speed));
            robot.backleft.setPower(Math.abs(speed));
            robot.backright.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.frontleft.isBusy() && robot.frontright.isBusy() && robot.backleft.isBusy() && robot.backright.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.frontleft.getCurrentPosition(),
                        robot.frontright.getCurrentPosition(),
                        robot.backleft.getCurrentPosition(),
                        robot.backright.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontleft.setPower(0);
            robot.frontright.setPower(0);
            robot.backleft.setPower(0);
            robot.backright.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void stop(int time) {
        robot.frontleft.setPower(0);
        robot.frontright.setPower(0);
        robot.backleft.setPower(0);
        robot.backright.setPower(0);
        sleep(time);
    }
}