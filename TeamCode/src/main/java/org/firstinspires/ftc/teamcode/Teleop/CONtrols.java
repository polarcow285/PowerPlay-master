//package org.firstinspires.ftc.teamcode.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.exception.RobotCoreException;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;
//
//@TeleOp(name="CONtrols", group="Mecanum")
//public class CONtrols extends LinearOpMode {
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
//        telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
//
//
//        int liftTarget = 0;
//        double liftSpeed = 0;
//        String liftCurrentDirection = "up";
//        boolean rollerForwardToggle = false;
//        boolean rollerBackwardToggle = false;
//
//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
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
//            //Driving controls
//            int high = 3000; //TEMP, 4100 is for medium poles
//            int medium = 2000;
//            int low = 1000;
//
//            double y = 0; //back and forth
//            double x = gamepad1.right_stick_x * 1.1; //strafing
//            double rx = gamepad1.left_stick_x; //turning
//
//            //back and forth movement using triggers
//            if (gamepad1.right_trigger > 0) {
//                y = gamepad1.right_trigger;
//                x = 0;
//            } else if (gamepad1.left_trigger > 0) {
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
//            // roller code
//            //intake
//            if (gamepad2.right_trigger > 0 && !robot.rollerSwitch.getState()) {
//                robot.roller.setPower(0.5);
//            }
//            //outtake
//            else if (gamepad2.left_trigger > 0) {
//                robot.roller.setPower(-0.5);
//            }
//            else {
//                robot.roller.setPower(0);
//            }
//
//            // lift code
//            if (gamepad2.right_bumper && robot.lift.getCurrentPosition()<3800) {
//                //liftSpeed = 1;
//                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.lift.setPower(0.75);
//            }
//            else if (gamepad2.left_bumper && robot.lift.getCurrentPosition()>0){
//                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.lift.setPower(-0.50);
//            }
//            else {
//                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.lift.setPower(0);
//            }
//
//
//
//
//
//            telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
//            telemetry.addData("Intake roller switch", robot.rollerSwitch.getState());
//            telemetry.addData("Distance sensor", robot.distance.getDistance(DistanceUnit.INCH));
//
//            // lift code
//
//            if (robot.lift.getCurrentPosition()<low && gamepad2.x) { // Arm UP
//                liftTarget = low;
//                liftSpeed = 0.5;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }else if (robot.lift.getCurrentPosition()>low && gamepad2.x) { // Arm DOWN
//                liftTarget = low;
//                liftSpeed = -0.5;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }else if (robot.lift.getCurrentPosition()<medium && gamepad2.b) { // Arm UP
//                liftTarget = medium;
//                liftSpeed = 0.5;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }else if (robot.lift.getCurrentPosition()>medium && gamepad2.b) { // Arm DOWN
//                liftTarget = medium;
//                liftSpeed = -0.5;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            else if (robot.lift.getCurrentPosition()<high && gamepad2.y) { // Arm UP
//                liftTarget = high;
//                liftSpeed = 0.5;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }else if (robot.lift.getCurrentPosition()>high && gamepad2.y) { // Arm DOWN
//                liftTarget = high;
//                liftSpeed = -0.5;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            else if (robot.lift.getCurrentPosition() > 0 && gamepad2.a) { // Arm DOWN
//                liftTarget = 0;
//                liftSpeed = -0.5;  // From my research, negative is ignore, so I don't understand why this *seemed* to work
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            /** END ARM LIFT **/
//
//            //Outtake sequence
////            if(gamepad2.dpad_down && robot.lift.getCurrentPosition()>50){
////                liftTarget = robot.lift.getCurrentPosition()-50;
////                liftSpeed = -0.98;
////                liftCurrentDirection = "down";
////
////                robot.lift.setPower(liftSpeed);
////                robot.lift.setTargetPosition(liftTarget);
////
////                robot.roller.setPower(-0.5);
////                sleep(500);
////                robot.roller.setPower(0);
////            }
//
//
//            idle();
//
//
//
//
//            /*telemetry.addData("frontLeftPower", frontLeftPower);
//            telemetry.addData("frontRightPower", frontRightPower);
//            telemetry.addData("backRightPower", backRightPower);
//            telemetry.addData("backLeftPower", backLeftPower);*/
//            telemetry.update();
//        }
//
//    }
//
//    }
