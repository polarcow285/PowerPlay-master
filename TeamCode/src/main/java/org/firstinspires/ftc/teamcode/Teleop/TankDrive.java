package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;

@TeleOp(name="Tank Drive")
public class TankDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ProjectUdon robot = new ProjectUdon();
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            //driving controls
            robot.backright.setPower(-gamepad1.right_stick_y);
            robot.backleft.setPower(-gamepad1.left_stick_y);
            //robot.frontright.setPower(-gamepad1.right_stick_y);
            //robot.frontleft.setPower(-gamepad1.left_stick_y);



        }
    }
}
