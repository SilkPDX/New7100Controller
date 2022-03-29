package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="JustParkBlue")
public class JustParkBlue extends LinearOpMode{
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor SpinnerL;
    DcMotor SpinnerR;
    DcMotor[] motors = new DcMotor[4];

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loadDelay = new ElapsedTime();
    boolean infiniteLoop = false;

    static final double TICKS_PER_CM = 1120 / 31.9186; //cm.
    @Override
    public void runOpMode() throws InterruptedException {
        runSetup();
        waitForStart();

        driveCm(1, 0, -122);
        driveCm(1, -60,0);


    }

    void runSetup() {
        motorLF = hardwareMap.get(DcMotor.class, "lf_drive");
        motorLB = hardwareMap.get(DcMotor.class, "lb_drive");
        motorRF = hardwareMap.get(DcMotor.class, "rf_drive");
        motorRB = hardwareMap.get(DcMotor.class, "rb_drive");
        motors[0] = motorLF;
        motors[1] = motorLB;
        motors[2] = motorRF;
        motors[3] = motorRB;
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        SpinnerL = hardwareMap.get(DcMotor.class, "spinner_l");
        SpinnerR = hardwareMap.get(DcMotor.class, "spinner_r");
        SpinnerR.setDirection(DcMotor.Direction.REVERSE);
        SpinnerL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SpinnerR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();

    }
    public void moveForward(double power, long seconds){

        motorLB.setPower(power);
        motorLF.setPower(power);
        motorRB.setPower(power);
        motorRF.setPower(power);
        sleep(seconds*1000);
        motorLB.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorRF.setPower(0);
    }

    public void strafe(Boolean isLeft, double power, long seconds){
        if (isLeft){
            power = -power;
        }

        motorLB.setPower(-power);
        motorLF.setPower(power);
        motorRB.setPower(power);
        motorRF.setPower(-power);
        sleep(seconds*1000);
        motorLB.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorRF.setPower(0);
    }

    public void spinSpinners(double power, long seconds){
        SpinnerL.setPower(power);
        SpinnerR.setPower(power);
        sleep(seconds*1000);
        SpinnerL.setPower(0);
        SpinnerR.setPower(0);
    }


    void driveCm(double speed, int forwardCm, int horizontalCm) {

        int forwardTicks = (int) Math.round(TICKS_PER_CM * forwardCm);
        int horizontalTicks = (int) Math.round(TICKS_PER_CM * horizontalCm);

        int[] currentPositions = {
                motorLF.getCurrentPosition(),
                motorLB.getCurrentPosition(),
                motorRF.getCurrentPosition(),
                motorRB.getCurrentPosition()
        };

        int[] deltaTargets = {
                forwardTicks + horizontalTicks, //lf
                forwardTicks - horizontalTicks, //lb
                forwardTicks - horizontalTicks, //rf
                forwardTicks + horizontalTicks, //rb
        };
        /*telemetry.addData("targetLF", targets[0]);
        telemetry.addData("targetLB", targets[1]);
        telemetry.addData("targetRF", targets[2]);
        telemetry.addData("targetRB", targets[3]);*/



        double[] speeds = new double[4];

        //find the target with greatest magnitude
        int maxDeltaTarget = 0;
        for(int i = 0; i < motors.length; i++) {
            if (maxDeltaTarget < Math.abs(deltaTargets[i])) {
                maxDeltaTarget = Math.abs(deltaTargets[i]);
            }
        }
        telemetry.addData("max", maxDeltaTarget);


        for(int i = 0; i < motors.length; i++) {
            speeds[i] = ((double) deltaTargets[i] / maxDeltaTarget) * speed; //the fastest wheel should be going at the speed "speed"
        }
        telemetry.addData("speed0", speeds[0]);
        telemetry.addData("speed1", speeds[1]);
        telemetry.addData("speed2", speeds[2]);
        telemetry.addData("speed3", speeds[3]);
        telemetry.update();
        for(int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(currentPositions[i] + deltaTargets[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(speeds[i]);
        }

        while(opModeIsActive() && (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy())) {
            //telemetry.addData("pos0", motors[0].getCurrentPosition());//wait for stuff to done
        }

        for(DcMotor motor: motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
