package com.dimassem.robotcontroller;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.util.Log;
import android.widget.Toast;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;

public class ConnectThread extends Thread {
    private final Context myContext;
    Handler handler;

    public ConnectThread(BluetoothDevice device, Context context) {
        // Use a temporary object that is later assigned to mmSocket
        // because mmSocket is final.
        myContext = context;
        BluetoothSocket tmp = null;
        RobotConnect.mmDevice = device;
        handler = new Handler();

        try {
            tmp = (BluetoothSocket) device.getClass().getMethod("createRfcommSocket", new Class[]{Integer.TYPE}).invoke(device, new Object[]{1});
        } catch (IllegalAccessException | IllegalArgumentException |
                 NoSuchMethodException | SecurityException | InvocationTargetException er) {
        }

        RobotConnect.mmSocket = tmp;
    }

    public void run() {
        try {
            // Connect to the remote device through the socket. This call blocks
            // until it succeeds or throws an exception.
            handler.post(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(myContext, "Подключение...", Toast.LENGTH_SHORT).show();
                }
            });

            RobotConnect.mmSocket.connect();
        } catch (IOException connectException) {
            // Unable to connect; close the socket and return.
            try {
                RobotConnect.mmSocket.close();
            } catch (IOException closeException) {
                Log.e("Connect", "Could not close the client socket", closeException);
            }
            handler.post(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(myContext, "Не удалось подключиться", Toast.LENGTH_SHORT).show();
                }
            });
            return;
        }

        RobotConnect.bluetoothAdapter.cancelDiscovery();
        myContext.startActivity(new Intent(myContext, Controller.class));
    }
}