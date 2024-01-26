package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Mechanisms {
    public DcMotor viperSlide;
    public DcMotor axle1;
    public DcMotor axle2;


    public Servo leftHanging;
    public Servo rightHanging;
    public Servo pivot;
    public double pivotGrab = 0.37;
    public double pivotScore = 0.2;


    public Servo rightClaw;
    public double rightClose = 0.83;
    public double rightOpen = 0.37;

    public Servo leftClaw;
    public double leftClose = 0.17;
    public double leftOpen = 0.63;

    double slideSpeed = 0.6;
    int maxPosition = 3600;
    LinearOpMode opMode;

    public Mechanisms(LinearOpMode op){
        opMode = op;
    }

    public void storePosition() {
        axle1.setTargetPosition(2400);
        axle2.setTargetPosition(2400);
        axle1.setPower(0.8);
        axle2.setPower(0.8);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setTargetPosition(150);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void scorePosition() {
        viperSlide.setTargetPosition(500);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPivotScore();
    }
    public void rightClawOpen() {
        rightClaw.setPosition(rightOpen);
    }
    public void leftClawOpen() {
        leftClaw.setPosition(leftOpen);
    }
    public void openClaws() {
        leftClaw.setPosition(leftOpen);
        rightClaw.setPosition(rightOpen);
    }

    public void closeClaws() {
        leftClaw.setPosition(leftClose);
        rightClaw.setPosition(rightClose);
    }

    public void setPivotGrab() {

        pivot.setPosition(pivotGrab);
    }

    public void setPivotScore() {

        pivot.setPosition(pivotScore);
    }
    public void zeroPosition() {
        axle1.setTargetPosition(0);
        axle2.setTargetPosition(0);
        axle1.setPower(0.8);
        axle2.setPower(0.8);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setTargetPosition(100);
        viperSlide.setPower(0.4);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void releaseHooks() {
        leftHanging.setPosition(0.2);
        rightHanging.setPosition(0.9);
        wait(500);
    }
    public void hangingPosition() {
        axle1.setTargetPosition(-4000);
        axle2.setTargetPosition(-4000);
        axle1.setPower(0.8);
        axle2.setPower(0.8);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void grabbingPosition() {
        axle1.setTargetPosition(3800);
        axle2.setTargetPosition(3800);
        axle1.setPower(0.8);
        axle2.setPower(0.8);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPivotGrab();
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void autoscoringPosition() {
        axle1.setTargetPosition(2100);
        axle2.setTargetPosition(2100);
        axle1.setPower(1);
        axle2.setPower(1);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(500 );
        viperSlide.setTargetPosition(1250);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(500);
        setPivotScore();
        wait(500);
        openClaws();
        wait(500);
        resetSlide();
        wait(500);

    }
    public void purplePosition() {
        viperSlide.setTargetPosition(300);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle1.setTargetPosition(4000);
        axle2.setTargetPosition(4000);
        axle1.setPower(1);
        axle2.setPower(1);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(2000);
        setPivotGrab();
        wait(500);
        lO();
        wait(500);

    }
    public void purplePosition3() {
        viperSlide.setTargetPosition(800);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle1.setTargetPosition(4000);
        axle2.setTargetPosition(4000);
        axle1.setPower(1);
        axle2.setPower(1);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(2000);
        setPivotGrab();
        wait(500);
        lO();
        wait(500);

    }
    public void resetSlide() {
        viperSlide.setTargetPosition(150);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void pickupwhitePixel() {
        axle1.setTargetPosition(3675);
        axle2.setTargetPosition(3675);
        axle1.setPower(1);
        axle2.setPower(1);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(3000);
        viperSlide.setTargetPosition(1150);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(500);
        setPivotGrab();
        wait(500);
        lC();
        wait(1000);
        resetSlide();

    }
    public void pickupwhitePixelB() {
        viperSlide.setTargetPosition(1150);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle1.setTargetPosition(3550);
        axle2.setTargetPosition(3550);
        axle1.setPower(1);
        axle2.setPower(1);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(3000);
        wait(500);
        setPivotGrab();
        wait(500);
        rC();
        wait(1000);
        resetSlide();

    }
    public void purplePositionB() {
        viperSlide.setTargetPosition(300);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle1.setTargetPosition(4000);
        axle2.setTargetPosition(4000);
        axle1.setPower(1);
        axle2.setPower(1);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(2000);
        setPivotGrab();
        wait(500);
        rO();
        wait(500);

    }
    public void purplePositionB1() {
        viperSlide.setTargetPosition(800);
        viperSlide.setPower(0.6);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle1.setTargetPosition(4000);
        axle2.setTargetPosition(4000);
        axle1.setPower(1);
        axle2.setPower(1);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(2000);
        setPivotGrab();
        wait(500);
        rO();
        wait(500);

    }
    public void autoServoPosition() {
        double pos = 0;
        double m = -0.00001;
        double b = 0.20915;
        pos = (m * ((axle1.getCurrentPosition() + axle2.getCurrentPosition()) / 2)) + b;
        pivot.setPosition(pos);
    }
    public void scoringPosition() {
        axle1.setTargetPosition(1600);
        axle2.setTargetPosition(1600);
        axle1.setPower(0.8);
        axle2.setPower(0.8);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void slideVipers(int position, double power, double runtimeMilli) {
        ElapsedTime runtimeTimer = new ElapsedTime();
        runtimeTimer.startTime();

        viperSlide.setTargetPosition(position);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(power);

        while (viperSlide.isBusy() && runtimeTimer.milliseconds() < runtimeMilli) {
            opMode.telemetry.addData("viperSlideRight current", viperSlide.getCurrentPosition());
            opMode.telemetry.addData("viperSlideRight target", viperSlide.getTargetPosition());
            opMode.telemetry.update();
        }

        viperSlide.setPower(0);

    }
    public void rotateAxle(String direction) {
        int pos1 = axle1.getCurrentPosition();
        int pos2 = axle2.getCurrentPosition();

        if(direction.equals("up")) {
            pos1 += 200;
            pos2 += 200;
        }
        else if(direction.equals("down")) {
            pos1 -= 200;
            pos2 -= 200;
        }

        axle1.setTargetPosition(pos1);
        axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle1.setPower(slideSpeed);

        axle2.setTargetPosition(pos2);
        axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axle2.setPower(slideSpeed);
    }

    public void extendSlide(String direction) {
        int pos1 = viperSlide.getCurrentPosition();

        if(direction.equals("up") && pos1 <= maxPosition) {
            pos1 += 100;
        }
        else if(direction.equals("down") && pos1 <= maxPosition) {
            pos1 -= 100;

        }

        viperSlide.setTargetPosition(pos1);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(slideSpeed);
    }

    /*
    public void autoScore(int position, double power, double rightPos, double midPos) {
        viperSlideLeft.setTargetPosition(position);
        viperSlideRight.setTargetPosition(position);

        viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperSlideLeft.setPower(power);
        viperSlideRight.setPower(power);

        wait(2000);
        right.setPosition(rightPos);
        wait(950);
        middle.setPosition(midPos);
        wait(800);
        right.setPosition(0.5);


        while (viperSlideLeft.isBusy()) {
            opMode.telemetry.addData("viperSlideLeft current", viperSlideLeft.getCurrentPosition());
            opMode.telemetry.addData("viperSlideLeft target", viperSlideLeft.getTargetPosition());

            opMode.telemetry.addData("viperSlideRight current", viperSlideRight.getCurrentPosition());
            opMode.telemetry.addData("viperSlideRight target", viperSlideRight.getTargetPosition());
            opMode.telemetry.update();
            break;
        }

    }
*/

    public void initMechanisms(HardwareMap hwMap){
        axle1 = hwMap.get(DcMotor.class,"axle1");
        axle2 = hwMap.get(DcMotor.class,"axle2");
        viperSlide = hwMap.get(DcMotor.class,"viperSlide");

        leftHanging = hwMap.get(Servo.class, "leftHanging");
        rightHanging = hwMap.get(Servo.class, "rightHanging");
        pivot = hwMap.get(Servo.class, "pivot");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        leftClaw = hwMap.get(Servo.class, "leftClaw");

        viperSlide.setDirection(DcMotor.Direction.FORWARD);
        axle1.setDirection(DcMotor.Direction.REVERSE);
        axle2.setDirection(DcMotor.Direction.REVERSE);

        axle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        axle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        axle1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        axle2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viperSlide.setPower(0);
        axle1.setPower(0);
        axle2.setPower(0);
    }
    public void lC() {
        leftClaw.setPosition(leftClose);
    }
    public void rC() {
        rightClaw.setPosition(rightClose);
    }
    public void rO() {rightClaw.setPosition(rightOpen);}
    public void lO() {leftClaw.setPosition(leftOpen);}
    public void initEncoders(){
        axle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        axle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        axle1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        axle2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void wait(int ms) {
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.milliseconds() < ms && opMode.opModeIsActive()) {
            opMode.telemetry.update();
        }
    }
}
