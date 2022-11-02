//package org.firstinspires.ftc.teamcode.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.exception.RobotCoreException;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;
//
//@TeleOp(name="NonDriving", group="Mecanum")
//public class NonDriving extends LinearOpMode {
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
//        int liftTarget = 0;
//        double liftSpeed = 0;
//        String liftCurrentDirection = "up";
//        boolean rollerForwardToggle = false;
//        boolean rollerBackwardToggle = false;
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            try {
//                previousGamepad2.copy(currentGamepad2);
//                currentGamepad2.copy(gamepad2);
//            }
//            catch (RobotCoreException e) {
//
//            }
//
//            // roller code
//            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
//                rollerForwardToggle = !rollerForwardToggle;
//            }
//            if (rollerForwardToggle) {
//                robot.roller.setPower(1);
//            }
//            else {
//                robot.roller.setPower(0);
//            }
//
//            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
//                rollerBackwardToggle = !rollerBackwardToggle;
//            }
//            if (rollerBackwardToggle) {
//                robot.roller.setPower(-1);
//            }
//            else {
//                robot.roller.setPower(0);
//            }
//
//
//            // lift code
//
//            if (robot.lift.getCurrentPosition()>-2000 && gamepad2.x) { // Arm UP
//                liftTarget = -2000;
//                liftSpeed = 0.98;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()<-2000 && gamepad2.x) { // Arm DOWN
//                liftTarget = -2000;
//                liftSpeed = -0.98;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()>-3500 && gamepad2.b) { // Arm UP
//                liftTarget = -3500;
//                liftSpeed = 0.98;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()<-3500 && gamepad2.b) { // Arm DOWN
//                liftTarget = -3500;
//                liftSpeed = -0.98;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()>-4200 && gamepad2.y) { // Arm UP
//                liftTarget = -4200;
//                liftSpeed = 0.98;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()<-4200 && gamepad2.y) { // Arm DOWN
//                liftTarget = -4200;
//                liftSpeed = -0.98;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            } else if (gamepad2.a){ // Arm DOWN
//                liftTarget = 0;
//                liftSpeed = -0.98;  // From my research, negative is ignore, so I don't understand why this *seemed* to work
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }
//
//            // Remove Power from the Arm Motor if motor is close to 0 position, arm should drop
////            if ( liftCurrentDirection == "down" && ( lift.getTargetPosition() < 5 ) ){
////                liftSpeed = 0;
////                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            }
//
//            /** END ARM LIFT **/
//
//
//            idle();
//
//            // Arm Lift Telemetry
//            if(robot.lift.isBusy() ){
//                telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
//                telemetry.update();
//            }
//
//        }
//
//    }
//
//}
