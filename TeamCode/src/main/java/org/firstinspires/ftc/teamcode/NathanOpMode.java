package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//This is my personal code. If you want to use this code, please rename it.
@TeleOp(name = "Nathan's OpMode", group = "LinearOpMode")
public class NathanOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FRONT_L = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor FRONT_R = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor BACK_L = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor BACK_R = hardwareMap.get(DcMotor.class, "backright");
        //DcMotor flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        //DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");
        //DcMotor intake1 = hardwareMap.get(DcMotor.class, "intake1");

        FRONT_L.setDirection(DcMotorSimple.Direction.FORWARD);
        FRONT_R.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_L.setDirection(DcMotorSimple.Direction.FORWARD);
        BACK_R.setDirection(DcMotorSimple.Direction.REVERSE);
        //flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        //intake1.setDirection(DcMotorSimple.Direction.FORWARD);

        FRONT_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRONT_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRONT_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRONT_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        double heading = 0;
        double x;
        double y;

        boolean lastA = false;
        boolean lastB = false;
        boolean yawReset = false;
        boolean fieldToggle = false;
        //double ballCannon_p = 0.0;
        double pulley_p = 0.0;
        //double suction_p = 0.0;
        double dx = 100;

        double forward_p;
        double right_p;
        double spin_p;

        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {
            telemetry.addLine("dpad up toggles field centric drive");
            telemetry.addLine("dpad down resets yaw value");
            telemetry.addLine("dpad right toggles rotation lock");
            telemetry.addLine("dpad left resets strafe distance");

            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            spin_p = gamepad1.right_stick_x;

            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if(gamepad1.dpad_up && !lastA) {
                fieldToggle = !fieldToggle;
            }
            lastA = gamepad1.dpad_up;

            if(fieldToggle) {
                forward_p = y*cos(-heading) - x * sin(-heading);
                right_p = y *sin(-heading) + x*cos(-heading);
            }
            else {
                forward_p = y;
                right_p = x;
            }
            telemetry.addData("Field Centric Toggled:", fieldToggle);
            telemetry.update();

            if(gamepad1.dpad_down && !lastB) {
                yawReset = !yawReset;
            }
            lastB = gamepad1.dpad_down;

            if(yawReset) {
                imu.resetYaw();
            }

            if(gamepad1.a) {
                pulley_p = 1.0;
            }
            else {
                pulley_p = 0.0;
            }

            //if(gamepad1.b) {
            //suction_p = 0.5;
            //pulley_p = 0.2;
            //}
            //else {
            //suction_p = 0.0;
            //pulley_p = 0.0;
            //}

            double frontRightPower = forward_p + spin_p + right_p;
            double frontLeftPower = forward_p - spin_p - right_p;
            double backRightPower = forward_p + spin_p - right_p;
            double backLeftPower = forward_p - spin_p + right_p;

            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            FRONT_R.setPower(frontRightPower);
            FRONT_L.setPower(frontLeftPower);
            BACK_R.setPower(backRightPower);
            BACK_L.setPower(backLeftPower);
            //flywheel.setPower(pulley_p);
            //launcher.setPower(ballCannon_p);
            //intake1.setPower(suction_p);

            telemetry.addData("Front Wheel Power Right/Left", "%4.2f, %4.2f", frontRightPower, frontLeftPower);
            telemetry.addData("Back Wheel Power Right/Left", "%4.2f, %4.2f", backRightPower, backLeftPower);
            telemetry.addData("Current Rotation (Degrees)", "%.2f", heading);
            telemetry.update();
        }
    }
}

