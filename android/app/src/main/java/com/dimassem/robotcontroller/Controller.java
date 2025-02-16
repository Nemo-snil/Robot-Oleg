package com.dimassem.robotcontroller;

import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.Toast;

import androidx.activity.EdgeToEdge;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import java.io.IOException;

import io.github.controlwear.virtual.joystick.android.JoystickView;

public class Controller extends AppCompatActivity {
    private static final int delay = 100;

    ConnectedThread connectedThread;

    float[] linear = new float[3];
    float[] angle = new float[3];

    Handler handler;

    JoystickView linear_joystick;
    SeekBar angle_joystick;
    Button up_button;
    Button down_button;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_controller);
        Toast.makeText(this, "Подключено", Toast.LENGTH_SHORT).show();

        linear_joystick = findViewById(R.id.linear_joystick);
        angle_joystick = findViewById(R.id.angle_joystick);
        up_button = findViewById(R.id.up_button);
        down_button = findViewById(R.id.down_button);

        handler = new Handler(new Handler.Callback() {
            @Override
            public boolean handleMessage(@NonNull Message msg) {
                if (msg.what == 0) {
                    connectedThread.write(linear[0] + "," +
                            linear[1] + "," +
                            linear[2] + "," +
                            angle[0] + "," +
                            angle[1] + "," +
                            angle[2]);

                    handler.sendEmptyMessageDelayed(0, delay);
                } else if (msg.what == 1) {
                    if (msg.arg1 == 0) {
                        connectedThread.write("UP");
                    } else if (msg.arg1 == 1) {
                        connectedThread.write("DOWN");
                    }
                }
                return true;
            }
        });

        connectedThread = new ConnectedThread();
        connectedThread.runnable = new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), "Отключено", Toast.LENGTH_SHORT).show();
                finish();
            }
        };
        connectedThread.start();

        handler.sendEmptyMessageDelayed(0, delay);

        up_button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Message message = new Message();
                message.what = 1;
                message.arg1 = 0;
                handler.sendMessageAtFrontOfQueue(message);
            }
        });

        down_button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Message message = new Message();
                message.what = 1;
                message.arg1 = 1;
                handler.sendMessageAtFrontOfQueue(message);
            }
        });

        linear_joystick.setOnMoveListener(new JoystickView.OnMoveListener() {
            @Override
            public void onMove(int angle, int strength) {
                //Преобразование
                linear[0] = (float) ((strength) * Math.cos(Math.toRadians(angle))) / 100;
                linear[0] = (float) (Math.floor(linear[0] * 100) / 100);
                linear[1] = (float) ((strength) * Math.sin(Math.toRadians(angle))) / 100;
                linear[1] = (float) (Math.floor(linear[1] * 100) / 100);
            }
        });

        angle_joystick.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                angle[2] = (float) (progress - 100) / 100;
                angle[2] = (float) (Math.floor(angle[2] * 100) / 100);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                angle[2] = 0;

                seekBar.setProgress(100);
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try {
            handler.removeMessages(0);
            RobotConnect.mmSocket.close();
        } catch (IOException e) {
            Log.e("Connect", "Could not close the client socket", e);
        }
    }
}