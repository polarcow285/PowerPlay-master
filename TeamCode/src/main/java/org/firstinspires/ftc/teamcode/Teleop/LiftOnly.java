//package org.firstinspires.ftc.teamcode.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.exception.RobotCoreException;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;
//
//@TeleOp(name="LiftOnly", group="Mecanum")
//public class LiftOnly extends LinearOpMode {
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
//
//        robot.lift.setTargetPosition(0);
//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
//            // lift code
//
//
//            int high = 3000; //TEMP, 4100 is for medium poles
//            int medium = 2000;
//            int low = 1000;
//
//
//            telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
//            telemetry.update();
//
//            if (gamepad2.right_bumper && robot.lift.getCurrentPosition()<3800) {
//                robot.lift.setPower(0.5);
//                robot.lift.setTargetPosition(robot.lift.getCurrentPosition() + 400);
//            }
//            else if (gamepad2.left_bumper && robot.lift.getCurrentPosition()>0){
//                robot.lift.setPower(-0.5);
//                robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - 400);
//            }
//
//            if (robot.lift.getCurrentPosition()<low && gamepad2.x) { // Arm UP
//                liftTarget = low;
//                liftSpeed = 0.5;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()>low && gamepad2.x) { // Arm DOWN
//                liftTarget = low;
//                liftSpeed = -0.5;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()<medium && gamepad2.b) { // Arm UP
//                liftTarget = medium;
//                liftSpeed = 0.5;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()>medium && gamepad2.b) { // Arm DOWN
//                liftTarget = medium;
//                liftSpeed = -0.5;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }
//            else if (robot.lift.getCurrentPosition()<high && gamepad2.y) { // Arm UP
//                liftTarget = high;
//                liftSpeed = 0.5;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()>high && gamepad2.y) { // Arm DOWN
//                liftTarget = high;
//                liftSpeed = -0.5;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }
//            else if (robot.lift.getCurrentPosition() > 0 && gamepad2.a) { // Arm DOWN
//                liftTarget = 0;
//                liftSpeed = -0.5;  // From my research, negative is ignore, so I don't understand why this *seemed* to work
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }
//
//
//
//            idle();
//
//            // Arm Lift Telemetry
//            if(robot.lift.isBusy() ){
//                telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//
//    }
//
//    }
