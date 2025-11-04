package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamePad extends OpMode{

    @Override
    public void init(){


    }

    @Override`
    public void loop(){
        // runs 50x* a second
        telemetry.addData(caption:"x", gamepad1.left_stick_x);
        telemetry.addData(caption:"y", gamepad1.left_stick_y);
        telemetry.addData(caption:"a button",gamepad1.a);


        *// TODO  now you connect it to the game pad. now you are at 7:30 in the
        *// how to orogram a gamepad brogan pratt youtube video
    }
}
