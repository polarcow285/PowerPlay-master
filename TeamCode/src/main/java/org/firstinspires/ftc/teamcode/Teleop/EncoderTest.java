//package org.firstinspires.ftc.teamcode.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.exception.RobotCoreException;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;
//
//@TeleOp(name="EncoderTest", group="Mecanum")
//public class EncoderTest extends LinearOpMode {
//    private ProjectUdon robot = new ProjectUdon();
//    Gamepad currentGamepad2 = new Gamepad();
//    Gamepad previousGamepad2 = new Gamepad();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot.init(hardwareMap);
//
//        telemetry.setAutoClear(true);
//        telemetry.addData("Front Right Motor Position", robot.frontright.getCurrentPosition());
//        telemetry.update();
//
//        robot.lift.setTargetPosition(0);
//        robot.frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            try {
//                previousGamepad2.copy(currentGamepad2);
//                currentGamepad2.copy(gamepad2);
//                throw new RobotCoreException("I am Exception Alpha!");
//            }
//            catch (RobotCoreException e) {
//
//            }
//
//            double y = 0; //back and forth
//            double x = gamepad1.right_stick_x * 1.1; //strafing
//            double rx = gamepad1.left_stick_x; //turning
//
//            if (gamepad1.right_trigger > 0) {
//                robot.frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                y = gamepad1.right_trigger;
//                x = 0;
//            } else if (gamepad1.left_trigger > 0) {
//                robot.frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                y = -gamepad1.left_trigger;
//                x = 0;
//            }
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio, but only when
//            // at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//
//            // Turn Right at lower power
//            if(frontLeftPower > 0 && backLeftPower > 0 && frontRightPower < 0 && backRightPower < 0){
//                robot.frontleft.setPower(frontLeftPower*0.35);
//                robot.backleft.setPower(backLeftPower*0.35);
//                robot.frontright.setPower(frontRightPower*0.35);
//                robot.backright.setPower(backRightPower*0.35);
//            }
//            // Turn Left at lower power
//            else if(frontLeftPower < 0 && backLeftPower < 0 && frontRightPower > 0 && backRightPower > 0){
//                robot.frontleft.setPower(frontLeftPower*0.35);
//                robot.backleft.setPower(backLeftPower*0.35);
//                robot.frontright.setPower(frontRightPower*0.35);
//                robot.backright.setPower(backRightPower*0.35);
//            }
//            // Strafe Right at lower power
//            else if(frontLeftPower > 0 && backLeftPower < 0 && frontRightPower < 0 && backRightPower > 0){
//                robot.frontleft.setPower(frontLeftPower*0.25);
//                robot.backleft.setPower(backLeftPower*0.25);
//                robot.frontright.setPower(frontRightPower*0.25);
//                robot.backright.setPower(backRightPower*0.25);
//            }
//            // Strafe Left at lower power
//            else if(frontLeftPower < 0 && backLeftPower > 0 && frontRightPower > 0 && backRightPower < 0){
//                robot.frontleft.setPower(frontLeftPower*0.25);
//                robot.backleft.setPower(backLeftPower*0.25);
//                robot.frontright.setPower(frontRightPower*0.25);
//                robot.backright.setPower(backRightPower*0.25);
//            }
//
//            //normal speed
//            else{
//                robot.frontleft.setPower(frontLeftPower*0.4);
//                robot.backleft.setPower(backLeftPower*0.4);
//                robot.frontright.setPower(frontRightPower*0.4);
//                robot.backright.setPower(backRightPower*0.4);
//            }
//
//            if (robot.frontright.getCurrentPosition()< 500 && gamepad1.x) { // Arm UP
//
//                encoderDrive(0.5, 500);
//
//            }else if (robot.frontright.getCurrentPosition()>500 && gamepad1.x) { // Arm DOWN
//
//                encoderDrive(-0.5, 500);
//
//            }else if (robot.frontright.getCurrentPosition()< 1000 && gamepad1.b) { // Arm UP
//
//                encoderDrive(0.5, 1000);
//
//            }else if (robot.frontright.getCurrentPosition()> 1000 && gamepad1.b) { // Arm DOWN
//
//                encoderDrive(-0.5, 1000);
//
//            }
//
//            telemetry.addData("Front Right Motor Position", robot.frontright.getCurrentPosition());
//            telemetry.update();
//
//            idle();
//        }
//
//    }
//
//    public void encoderDrive(double speed, int counts) {
//        robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontleft.setPower(speed);
//        robot.backleft.setPower(speed);
//        robot.frontright.setPower(speed);
//        robot.backright.setPower(speed);
//        robot.frontright.setTargetPosition(counts);
//        robot.frontleft.setTargetPosition(counts);
//        robot.backright.setTargetPosition(counts);
//        robot.backleft.setTargetPosition(counts);
//        robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//}
