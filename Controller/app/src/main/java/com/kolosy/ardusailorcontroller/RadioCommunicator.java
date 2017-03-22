package com.kolosy.ardusailorcontroller;

import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Binder;
import android.os.Environment;
import android.os.IBinder;
import android.os.SystemClock;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.List;

public class RadioCommunicator {

    public class State {
        public double htw, heading, dtw, wind;

        public State (double wind, double htw, double heading, double dtw) {
            this.wind = wind;
            this.htw = htw;
            this.heading = heading;
            this.dtw = dtw;
        }
    }

    public synchronized void setState(int rudder, int winch) {
        radioCommand = String.format("[%d;%d]", rudder - 50, (int)(winch * 0.9)).getBytes();
    }

    public synchronized State getState() {
        return new State(wind, wp_heading, ahrs_heading, wp_distance);
    }

    private String gps_aprs_lat, gps_aprs_lon;
    private int current_rudder, current_winch, wind;
    private double gps_course, ahrs_heading, gps_lat, gps_lon, gps_speed, current_roll, heel_adjust, wp_heading, wp_distance, voltage;
    private long millis, cycle;

    private String lastLine;

    private long lastLineTime;
    private int failLineCount;

    private static final char END_CHAR = '\n';

    private int rudderPos;
    private int winchPos;

    private byte[] radioCommand;

    OutputStream dataOut;

    UsbSerialPort port;

    private volatile boolean terminated = true;

    private static final String TAG = "RADIOSERVICE";

    private static final int TRANSMIT_EVERY = 500;

    private UsbManager manager;
    private UsbSerialDriver driver;

    private SerialInputOutputManager mSerialIoManager;

    private final Object newDataFlag = new Object();
    private byte[] newData = null;

    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {

                @Override
                public void onRunError(Exception e) {
                    Log.d(TAG, "Runner stopped.");
                }

                @Override
                public void onNewData(final byte[] data) {
                    RadioCommunicator.this.pushNewData(data);
                }
            };

    private void pushNewData(byte[] data) {
        synchronized (newDataFlag) {
            newData = data;

            newDataFlag.notifyAll();
        }
    }

    public RadioCommunicator(Context ctx, PendingIntent intent) {
        File path = Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_PICTURES);
        File file = new File(path, "capture" + System.currentTimeMillis() + ".log");

        try {
            dataOut = new FileOutputStream(file);
        } catch (FileNotFoundException e) {
            Log.e(TAG, "Exception while creating log file", e);
        }

// Find all available drivers from attached devices.
        manager = (UsbManager) ctx.getSystemService(Context.USB_SERVICE);
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
        if (availableDrivers.isEmpty()) {
            Log.d(TAG, "No devices found");
            return;
        }

// Open a connection to the first available driver.
        driver = availableDrivers.get(0);
        manager.requestPermission(driver.getDevice(), intent);

    }

    public void start() {
        terminated = false;

        try {
            UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
            if (connection == null) {
                Log.d(TAG, "Unable to connect");
                // You probably need to call UsbManager.requestPermission(driver.getDevice(), ..)

                terminated = true;

                return;
            }
            port = driver.getPorts().get(0);
            port.open(connection);
            port.setParameters(9600, 8, 1, 0);
            port.setDTR(true);
        } catch (Exception e) {
            Log.e(TAG, "Exception while connecting to device", e);
            terminated = true;
        }

        startIoManager();

        synchronized (this) {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {

                        new Thread(new Runnable() {
                            @Override
                            public void run() {
                                mSerialIoManager.run();
                            }
                        }).start();

                                read();
                    } catch (IOException e) {
                        Log.e(TAG, "Exception while reading", e);
                    }
                }
            }).start();
        }
    }

    public synchronized void stop() {
        try {
            if (dataOut != null)
                dataOut.close();
            if (port != null)
                port.close();
        } catch (IOException ex) {
            Log.e(TAG, "Exception while shutting down", ex);
        } finally {
            terminated = true;
        }
    }

    private void stopIoManager() {
        if (mSerialIoManager != null) {
            Log.i(TAG, "Stopping io manager ..");
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if (port != null) {
            Log.i(TAG, "Starting io manager ..");
            mSerialIoManager = new SerialInputOutputManager(port, mListener);
        }
    }

    private void onDeviceStateChange() {
        stopIoManager();
        startIoManager();
    }

    private void read() throws IOException {
        StringBuilder sb = new StringBuilder();

        long lastTransmit = 0;

        byte[] buffer = null;

        while (!terminated) {

            synchronized (newDataFlag) {

                try {
                    newDataFlag.wait();
                } catch (InterruptedException e) {
                    Log.e(TAG, "Exception waiting for data", e);
                }

                if (newData != null)
                    buffer = newData.clone();
            }

            for (int i=0; i<buffer.length; i++) {
                if (buffer[i] != END_CHAR)
                    sb.append((char)buffer[i]);
                else {
                    synchronized (this) {
                        processLine(sb.toString());

                        if (System.currentTimeMillis() - lastTransmit > TRANSMIT_EVERY) {
                            transmit();
                            lastTransmit = System.currentTimeMillis();
                        }
                    }

                    sb = new StringBuilder();
                }
            }
        }
    }

    private void transmit() throws IOException {
        if (radioCommand != null)
            port.write(radioCommand, 200);
    }

    /*
4155.62N, 08739.22W, 41.927066, -87.653717, 1042, 0.00, 7.60, 113.78, 3.88, 0.00, 4, 103.52, 1926.04, 90, 60, 12.23, 2800
4155.62N, 08739.22W, 41.927066, -87.653717, 569, 0.00, 7.60, 110.37, 3.88, 0.00, 4, 103.52, 1926.04, 90, 60, 12.25, 2805
4155.62N, 08739.22W, 41.927066, -87.653717, 250, 0.00, 7.60, 117.42, 3.89, 0.00, 4, 103.52, 1926.04, 90, 60, 12.25, 2810
4155.62N, 08739.22W, 41.927066, -87.653717, 860, 0.00, 7.60, 113.69, 3.89, 0.00, 4, 103.52, 1926.04, 90, 60, 12.24, 2815
4155.62N, 08739.22W, 41.927066, -87.653717, 344, 0.00, 7.60, 113.36, 3.90, 0.00, 4, 103.52, 1926.04, 90, 60, 12.24, 2820
4155.62N, 08739.22W, 41.927066, -87.653717, 936, 0.00, 7.60, 113.04, 3.90, 0.00, 4, 103.52, 1926.04, 90, 60, 12.24, 2825

     */
    private void processLine(String lineIn) throws IOException {
//        Log.d(TAG, lineIn);

        String[] parts = lineIn.split(",");

        if (parts.length != 17) {
            failLineCount++;
            return;
        }

        lastLineTime = System.currentTimeMillis();
        failLineCount = 0;

        int i = 0;

        gps_aprs_lat = parts[i++].trim();
        gps_aprs_lon = parts[i++].trim();
        gps_lat = Double.parseDouble(parts[i++].trim());
        gps_lon = Double.parseDouble(parts[i++].trim());
        millis = Integer.parseInt(parts[i++].trim());
        gps_speed = Double.parseDouble(parts[i++].trim());
        gps_course = Double.parseDouble(parts[i++].trim());
        ahrs_heading = Double.parseDouble(parts[i++].trim());
        current_roll = Double.parseDouble(parts[i++].trim());
        heel_adjust = Double.parseDouble(parts[i++].trim());
        wind = Integer.parseInt(parts[i++].trim());
        wp_heading = Double.parseDouble(parts[i++].trim());
        wp_distance = Double.parseDouble(parts[i++].trim());
        current_rudder = Integer.parseInt(parts[i++].trim());
        current_winch = Integer.parseInt(parts[i++].trim());
        voltage = Double.parseDouble(parts[i++].trim());
        cycle = Integer.parseInt(parts[i++].trim());

        lastLine = lineIn;

        dataOut.write(lineIn.getBytes());
    }

    public synchronized boolean isRunning() { return !terminated; }
}
