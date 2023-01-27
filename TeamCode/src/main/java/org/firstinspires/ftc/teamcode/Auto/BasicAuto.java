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
public class BasicAuto extends LinearOpMode
{
    public ProjectUdon robot = new ProjectUdon();
    OpenCvCamera webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    PoleDetectionPipeline detector = new PoleDetectionPipeline(telemetry);

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
//    double tagsize = 0.166;
      double tagsize = 0.003;
    int LEFT = 1; // Tag ID 1, 2, 3 from the 36h11 family
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        robot.frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


//         * The INIT-loop:
//         * This REPLACES waitForStart!

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.

         Update the telemetry*/

        PoleDetectionPipeline.ShippingElementLocation elementLocation = detector.getShippingElementLocation();

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // Actually do something useful
        if (tagOfInterest == null || tagOfInterest.id == LEFT)
        {
            //trajectory for LEFT/DEFAULT
            // 750 counts for 1 tile
            // 412 counts to turn 90 degrees

            encoderDrive (0.3, 800,800,800,800);
            stop(500);
            encoderDrive (0.3, -412, 412, -412, 412);
            stop(500);
            encoderDrive (0.3, 750,750,750,750);
            stop(500);
        }
        else if (tagOfInterest.id == MIDDLE) {
            //trajectory for MIDDLE
            encoderDrive (0.3, 800, 800,800,800);
            stop(1000);
        }
        else {
            //trajectory for RIGHT
            encoderDrive (0.3, 800,800,800,800);
            stop(1000);
            encoderDrive (0.3, 412, -412, 412, -412);
            stop(1000);
            encoderDrive (0.3, 750,750,750,750);
            stop(1000);
        }

        //You wouldn't have this in your autonomous, this is just to prevent the sample from ending
         // DELETE FOR PROD
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
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
                telemetry.addData("Pole Location", detector.getShippingElementLocation());
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

