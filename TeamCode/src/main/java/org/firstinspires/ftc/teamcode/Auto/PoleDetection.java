package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.PoleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;

@Autonomous
public class PoleDetection extends LinearOpMode{
    public ProjectUdon robot = new ProjectUdon();
    OpenCvCamera webcam;
    PoleDetectionPipeline poleDetectionPipeline = new PoleDetectionPipeline(telemetry);
    boolean poleInRange = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(poleDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();


        //NEEDS TO BE FIXED
        // DRIVE TO AND LINE UP WITH POLE
        while(poleInRange == false){
            PoleDetectionPipeline.PoleLocation elementLocation = poleDetectionPipeline.getPoleLocation();

//            if(elementLocation == PoleDetectionPipeline.PoleLocation.FAR){
//                encoderDrive(0.25, 25, 25, 25, 25);
//                stop(1000);
//            }
            if(elementLocation == PoleDetectionPipeline.PoleLocation.LEFT) {
                encoderDrive(0.25, -25, 25, -25, 25);
                stop(1000);
            }
            else if (elementLocation == PoleDetectionPipeline.PoleLocation.RIGHT) {
                encoderDrive(0.25, 25, -25, 25, -25);
                stop(1000);
            } else if (elementLocation == PoleDetectionPipeline.PoleLocation.MIDDLE) {
//                stop(2000);
                encoderDrive(0.25, 25, 25, 25, 25);
                stop(1000);
//                poleInRange = true;
            }
            else if (elementLocation == PoleDetectionPipeline.PoleLocation.CLOSE) {
                stop(2000);
                poleInRange = true;
            }
            else{
                encoderDrive(0.25, -25, -25, -25, -25);
                stop(1000);
            }
        }

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
