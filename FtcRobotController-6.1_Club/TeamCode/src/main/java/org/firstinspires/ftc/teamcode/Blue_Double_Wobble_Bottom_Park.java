// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Blue Double Wobble Bottom Park", group="autonomous")
public class Blue_Double_Wobble_Bottom_Park extends LinearOpMode
{
    //Declaring all Motors and Servos
    DcMotor Front_Left_Motor;
    DcMotor Front_Right_Motor;
    DcMotor Back_Left_Motor;
    DcMotor Back_Right_Motor;
    DcMotor Collector_Stage_1_Motor;
    DcMotor Collector_Stage_2_Motor;
    DcMotor Wobble_Goal_Arm_Motor;
    DcMotor Shooter_Motor;
    Servo lifter9000;
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Import all hardware names and set them to hardware names
        Front_Left_Motor = hardwareMap.dcMotor.get("Front_Left_Motor");
        Front_Right_Motor = hardwareMap.dcMotor.get("Front_Right_Motor");
        Back_Left_Motor = hardwareMap.dcMotor.get("Back_Left_Motor ");
        Back_Right_Motor = hardwareMap.dcMotor.get("Back_Right_Motor");
        Wobble_Goal_Arm_Motor = hardwareMap.dcMotor.get("Wobble_Goal_Arm_Motor");
        Shooter_Motor = hardwareMap.dcMotor.get("Shooter_Motor");
        Collector_Stage_1_Motor = hardwareMap.dcMotor.get("Collector_Stage_1_Motor");
        Collector_Stage_2_Motor = hardwareMap.dcMotor.get("Collector_Stage_2_Motor");
        lifter9000 = hardwareMap.servo.get("servo1");
        // set motors that need to be reversed to reverse
        Front_Left_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Left_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        // set motors for automatic brake
        Front_Left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_Right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_Left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_Right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wobble_Goal_Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //stoping and resetting the motor encoders
        Front_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wobble_Goal_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pre-initialized hardware
        lifter9000.setPosition(0);
        waitForStart();
        Robot_Forward_Backward(1,1000);
        //were main auto code goes
    }
    //Method that drives robot forward or backward
    private void Robot_Forward_Backward(double Power, int Distance) {
        //set motors target position to a sent down variable
        Front_Right_Motor.setTargetPosition(Distance);
        Front_Left_Motor.setTargetPosition(Distance);
        Back_Right_Motor.setTargetPosition(Distance);
        Back_Left_Motor.setTargetPosition(Distance);
        //set motors to run to encoder position mode
        Front_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set motor power to variable from main
        Front_Right_Motor.setPower(Power);
        Front_Left_Motor.setPower(Power);
        Back_Right_Motor.setPower(Power);
        Back_Left_Motor.setPower(Power);
        //loop that is a place holder while the motors are running so that it can finish it's task
        //without skipping
        while (Back_Left_Motor.isBusy() && Back_Right_Motor.isBusy() && Front_Left_Motor.isBusy() && Front_Right_Motor.isBusy())
        { }
        //reset the motors encoders, getting them ready for next task
        Front_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //Method that rotates the robot clockwise 90 degrees.
    private void Robot_Rotate_Clockwise_90_degrees() {
        //set motors to set positions that rotate robot 90 degrees clockwise
        Front_Right_Motor.setTargetPosition(-2050);
        Front_Left_Motor.setTargetPosition(2050);
        Back_Right_Motor.setTargetPosition(-2050);
        Back_Left_Motor.setTargetPosition(2050);
        //set motors to run to encoder position mode
        Front_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Setting the motor power to max.
        Front_Right_Motor.setPower(1);
        Front_Left_Motor.setPower(1);
        Back_Right_Motor.setPower(1);
        Back_Left_Motor.setPower(1);
        //loop that is a place holder while the motors are running so that it can finish it's task
        //without skipping
        while (Back_Left_Motor.isBusy() && Back_Right_Motor.isBusy() && Front_Left_Motor.isBusy() && Front_Right_Motor.isBusy())
        { }
        //reset the motors encoders, getting them ready for next task
        Front_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void robotLeft()
    {
        Front_Right_Motor.setTargetPosition(1950);
        Front_Left_Motor.setTargetPosition(-1950);
        Back_Right_Motor.setTargetPosition(1950);
        Back_Left_Motor.setTargetPosition(-1950);
        //set motors to run to encoder position mode
        Front_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Setting the motor power to max.
        Front_Right_Motor.setPower(1);
        Front_Left_Motor.setPower(1);
        Back_Right_Motor.setPower(1);
        Back_Left_Motor.setPower(1);
        while (Back_Left_Motor.isBusy() && Back_Right_Motor.isBusy() && Front_Left_Motor.isBusy() && Front_Right_Motor.isBusy())
        { }
        Front_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void robotLeftSideways(double power, int distance)
    {
        Front_Right_Motor.setTargetPosition(distance);
        Front_Left_Motor.setTargetPosition(-distance);
        Back_Right_Motor.setTargetPosition(-distance);
        Back_Left_Motor.setTargetPosition(distance);
        //set motors to run to encoder position mode
        Front_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right_Motor.setPower(power);
        Front_Left_Motor.setPower(power);
        Back_Right_Motor.setPower(power);
        Back_Left_Motor.setPower(power);
        while (Back_Left_Motor.isBusy() && Back_Right_Motor.isBusy() && Front_Left_Motor.isBusy() && Front_Right_Motor.isBusy())
        { }
        Front_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void robotRightSideways(double power, int distance)
    {
        Front_Right_Motor.setTargetPosition(-distance);
        Front_Left_Motor.setTargetPosition(distance);
        Back_Right_Motor.setTargetPosition(distance);
        Back_Left_Motor.setTargetPosition(-distance);
        //set motors to run to encoder position mode
        Front_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right_Motor.setPower(power);
        Front_Left_Motor.setPower(power);
        Back_Right_Motor.setPower(power);
        Back_Left_Motor.setPower(power);
        while (Back_Left_Motor.isBusy() && Back_Right_Motor.isBusy() && Front_Left_Motor.isBusy() && Front_Right_Motor.isBusy())
        { }
        Front_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void wobbleGoalArm(double power, int distance)
    {
        Wobble_Goal_Arm_Motor.setTargetPosition(distance);
        //set motor to run to encoder position mode
        Wobble_Goal_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Wobble_Goal_Arm_Motor.setPower(power);
        while (Wobble_Goal_Arm_Motor.isBusy())
        {
            telemetry.addData("Mode", Wobble_Goal_Arm_Motor.getCurrentPosition());
            telemetry.update();
        }
        Wobble_Goal_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}