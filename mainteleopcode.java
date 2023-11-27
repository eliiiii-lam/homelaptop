
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "GGTele")
public class mainteleopcode extends OpMode {

    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;

    DcMotor elbow;

    Servo clawL;
    Servo clawR;

    CRServo wrist;

    @Override
    public void init() {
        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");

        elbow = hardwareMap.dcMotor.get("elbow");

        wrist = hardwareMap.crservo.get("wrist");

        clawL = hardwareMap.servo.get("clawL");

        clawR = hardwareMap.servo.get("clawR");

    }

    @Override
    public void loop() {
        if (Math.abs(-gamepad1.left_stick_y) > .1) {
            fL.setPower(-gamepad1.left_stick_y * 1);
            bL.setPower(-gamepad1.left_stick_y * 1);
        } else {
            fL.setPower(0);
            bL.setPower(0);
        }
        if (Math.abs(-gamepad1.right_stick_y) > .1) {
            fR.setPower(-gamepad1.right_stick_y * -1);
            bR.setPower(-gamepad1.right_stick_y * -1);
        } else {
            fR.setPower(0);
            bR.setPower(0);
        }



        if (Math.abs(-gamepad2.left_stick_y) > .1) {
            elbow.setPower(-gamepad2.left_stick_y * 0.7);
        } else {
            elbow.setPower(0);
        }





        if (gamepad2.left_bumper) {
            clawL.setPosition(0.4);
        }

        if (Math.abs(gamepad2.left_trigger)  > .1) {
            clawL.setPosition(0);
        }

        if (gamepad2.right_bumper) {
            clawR.setPosition(-0.4);
        }

        if (Math.abs(gamepad2.right_trigger) > .1) {
            clawR.setPosition(0);
        }






        if(Math.abs(gamepad2.right_trigger) > .1){
            wrist.setPower(0.7);
        } else {
            wrist.setPower(0);
        }


        if (gamepad1.left_bumper) {
            fL.setPower(-1);
            bL.setPower(1);
            fR.setPower(-1);
            bR.setPower(1);
        } else {
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            bR.setPower(0);

        }
        if (gamepad1.right_bumper) {
            fL.setPower(1);
            bL.setPower(-1);
            fR.setPower(1);
            bR.setPower(-1);
        } else {
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            bR.setPower(0);
        }


    }
}





//fart elijah
