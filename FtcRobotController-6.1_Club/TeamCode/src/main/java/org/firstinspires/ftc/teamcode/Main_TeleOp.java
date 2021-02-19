package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name="TeleOp", group="teleop")
public class Main_TeleOp extends OpMode
{
    //Drive Motors
    DcMotor Front_Left_Motor;
    DcMotor Front_Right_Motor;
    DcMotor Back_Left_Motor;
    DcMotor Back_Right_Motor;
    DcMotor Collector_Stage_1_Motor;
    DcMotor Collector_Stage_2_Motor;
    DcMotor Wobble_Goal_Arm_Motor;
    DcMotor Shooter_Motor;
    Servo Wobble_Goal_Grabber_Servo;
    TouchSensor touch;
   //variables
   double Shooter_Power = 0;
   double RPM = 0;
   int count = 0;
   int rotations = 0;
    @Override
    public void init()
    {
        //Import all hardware and get there config name
        Front_Left_Motor = hardwareMap.dcMotor.get("Front_Left_Motor");
        Front_Right_Motor = hardwareMap.dcMotor.get("Front_Right_Motor");
        Back_Left_Motor = hardwareMap.dcMotor.get("Back_Left_Motor");
        Wobble_Goal_Grabber_Servo = hardwareMap.servo.get("Wobble_Goal_Grabber_Servo");
        Back_Right_Motor = hardwareMap.dcMotor.get("Back_Right_Motor");
        Collector_Stage_1_Motor = hardwareMap.dcMotor.get("Collector_Stage_1_Motor");
        Collector_Stage_2_Motor = hardwareMap.dcMotor.get("Collector_Stage_2_Motor");
        Wobble_Goal_Arm_Motor = hardwareMap.dcMotor.get("Wobble_Goal_Arm_Motor");
        Shooter_Motor = hardwareMap.dcMotor.get("Shooter_Motor");
        touch = hardwareMap.touchSensor.get("touch");
        //Enable Braking on motors
        Front_Left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_Right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_Left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_Right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Collector_Stage_1_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Collector_Stage_2_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wobble_Goal_Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Enable reverse on needed motors
        Front_Left_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Left_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //set motors to to stop and reset encoders
        Shooter_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set the motors to run using the encoders
        Shooter_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //pre-initialize needed hardware
        Wobble_Goal_Grabber_Servo.setPosition(1);
    }
    @Override
    public void loop()
    {
        //variables set from controller inputs
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;
        //Drive code
        Front_Left_Motor.setPower(-y + x + r);
        Front_Right_Motor.setPower(-y - x - r);
        Back_Left_Motor.setPower(-y - x + r);
        Back_Right_Motor.setPower(-y + x - r);
        Collector_Stage_2_Motor.setPower(gamepad2.left_stick_y);
        Collector_Stage_1_Motor.setPower(-gamepad2.right_stick_y);
        Shooter_Motor.setPower(Shooter_Power);
        if (gamepad2.left_trigger > 0)
        {
            Wobble_Goal_Arm_Motor.setPower(gamepad2.left_trigger/5);
        }
        if (gamepad2.right_trigger > 0)
        {
            Wobble_Goal_Arm_Motor.setPower(-gamepad2.right_trigger/5);
        }
        if (gamepad2.right_bumper)
        {
            Shooter_Power = Shooter_Power + 0.001;
        }
        if (gamepad2.x)
        {
            Shooter_Power = 0.0;
        }
        if (gamepad2.left_bumper)
        {
            Shooter_Power = Shooter_Power - 0.001;
        }
        if (gamepad2.y)
        {
            Shooter_Power = 0.2;
        }
        if (gamepad2.b)
        {
            Shooter_Power = 0.3;
        }
        if (gamepad2.a)
        {
            Shooter_Power = 0.4;
        }
        if (Shooter_Power >= 1)
        {
            Shooter_Power = 1;
        }
        if (Shooter_Power <= -1)
        {
            Shooter_Power = -1;
        }
        if (Shooter_Power > 0.1)
        {
            rotations = Shooter_Motor.getCurrentPosition();
            count++;
            RPM = rotations / count;
        }
        if (Shooter_Power == 0)
        {
            count = 0;
        }
        if (gamepad2.dpad_up)
        {
            Wobble_Goal_Grabber_Servo.setPosition(1);
        }
        if (gamepad2.dpad_down)
        {
            Wobble_Goal_Grabber_Servo.setPosition(-1);
        }
        telemetry.addData("power", Math.round(Shooter_Power*100));
        telemetry.addData("RPM", RPM);
        telemetry.update();
    }
}
