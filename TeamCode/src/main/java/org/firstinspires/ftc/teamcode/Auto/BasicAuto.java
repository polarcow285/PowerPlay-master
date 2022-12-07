package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);
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

//            moveForwards(1000, 0.5);
//            stop(200);
//            moveLeft(1000, 0.5);
//            stop(500);
            // 1.25 sec at 0.5 speed to travel 1 tile
            // 2.5 sec at 0.25 speed to travel 1 tile
            moveForwards(2000, 0.5);
            stop(200);
            turnLeft(600, 0.5);
            stop(200);
            moveForwards(1000,0.5);
            stop(200);

        }
        else if (tagOfInterest.id == MIDDLE) {
            //trajectory for MIDDLE
            moveForwards(1800, 0.5);
            stop(500);
        }
        else {
            //trajectory for RIGHT
//            moveForwards(1000, 0.5);
//            stop(200);
//            moveRight(1000, 0.5);
//            stop(500);
            moveForwards(2000,0.5);
            stop(200);
            turnRight(600,0.5);
            stop(200);
            moveForwards(1000,0.5);
            stop(200);
        }

        //You wouldn't have this in your autonomous, this is just to prevent the sample from ending
         // DELETE FOR PROD
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


    // Beautiful methods
    public void moveForwards(int time, double power){
        robot.frontleft.setPower(power);
        robot.frontright.setPower(power);
        robot.backleft.setPower(power);
        robot.backright.setPower(power);
        sleep(time);
    }

    public void moveRight(int time, double power){
        robot.frontleft.setPower(-power);
        robot.frontright.setPower(power);
        robot.backleft.setPower(power);
        robot.backright.setPower(-power);
        sleep(time);
    }

    public void moveLeft(int time, double power) {
        robot.frontleft.setPower(power);
        robot.frontright.setPower(-power);
        robot.backleft.setPower(-power);
        robot.backright.setPower(power);
        sleep(time);
    }

    public void moveBackwards(int time, double power){
        robot.frontleft.setPower(-power);
        robot.frontright.setPower(-power);
        robot.backleft.setPower(-power);
        robot.backright.setPower(-power);
        sleep(time);
    }

    public void turnRight (int time, double power){
        robot.frontleft.setPower(power);
        robot.frontright.setPower(-power);
        robot.backleft.setPower(power);
        robot.backright.setPower(-power);
        sleep(time);
    }

    public void turnLeft(int time, double power){
        robot.frontleft.setPower(-power);
        robot.frontright.setPower(power);
        robot.backleft.setPower(-power);
        robot.backright.setPower(power);
        sleep(time);
    }

    public void stop(int time) {
        robot.frontleft.setPower(0);
        robot.frontright.setPower(0);
        robot.backleft.setPower(0);
        robot.backright.setPower(0);
        sleep(time);
    }

}

