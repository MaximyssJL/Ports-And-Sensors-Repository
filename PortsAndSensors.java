package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "OpModeVideo", group = "Honors Robotics")

public class DriverOpMode extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor armMotor;

    Servo armServo;

    RevColorSensorV3 colorSensor;

    NormalizedColorSensor colorSensor1;

    ColorSensor colorSensor2;

    TouchSensor touchSensor;

    boolean forwardCondition = false;
    boolean backwardCondition = false;
    boolean buttonState = false;

    public void init() {
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        armMotor = hardwareMap.dcMotor.get("arm_motor");

        armServo = hardwareMap.servo.get("arm_servo");

        colorSensor = (RevColorSensorV3) hardwareMap.get("color_sensor");

        colorSensor1 = (NormalizedColorSensor) hardwareMap.get("color_sensor");

        colorSensor2 = (ColorSensor) hardwareMap.get("color_sensor");

        touchSensor = hardwareMap.touchSensor.get("touch_sensor");

        frontRight.setDirection(DcMotor.Direction.REVERSE);

    }

    public void loop() {

        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);

        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("Right Stick x", gamepad1.right_stick_x);

        telemetry.addData("X", gamepad1.x);
        telemetry.addData("Y", gamepad1.y);
        telemetry.addData("A", gamepad1.a);
        telemetry.addData("B", gamepad1.b);

        telemetry.addData("Button State", buttonState);


        telemetry.addData("R", colorSensor2.red());
        telemetry.addData("G", colorSensor2.green());
        telemetry.addData("B", colorSensor2.blue());
        telemetry.addData("Hue", colorSensor2.argb());

        telemetry.addData("Normalized Color", colorSensor1.getNormalizedColors());

        telemetry.addData("R (REV)", colorSensor.red());
        telemetry.addData("G (REV)", colorSensor.green());
        telemetry.addData("B (REV)", colorSensor.blue());
        telemetry.addData("Hue (REV)", colorSensor.argb());


        float leftTrigger = gamepad1.left_trigger;
        float rightTrigger = gamepad1.right_trigger;
        float rightStickX = gamepad1.right_stick_x;
        if (forwardCondition == false && backwardCondition == false) {
            frontLeft.setPower(leftTrigger + rightStickX);
            frontRight.setPower(leftTrigger - rightStickX);
            frontLeft.setPower(-(rightTrigger) + rightStickX);
            frontRight.setPower(-(rightTrigger) - rightStickX);
        }

        float leftStickY = gamepad1.left_stick_y;
        if (leftStickY < 0) {
            armMotor.setPower(0.3);
        }
        if (leftStickY > 0) {
            armMotor.setPower(-0.3);
        }
        if (leftStickY == 0) {
            armMotor.setPower(0);
        }

        if (gamepad1.y == true) {
            forward();
        }

        if (gamepad1.a == true) {
            backward();
        }

        if (gamepad1.b) {
            stop();
        }

        if (touchSensor.isPressed()) {
            buttonState = !buttonState;
        }


    }

    public void forward() {
        forwardCondition = true;
        frontLeft.setPower(-1);
        frontRight.setPower(-1);
    }
    public void backward() {
        backwardCondition = true;
        frontLeft.setPower(1);
        frontRight.setPower(1);
    }
    public void stop() {
        forwardCondition = false;
        backwardCondition = false;
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

}
