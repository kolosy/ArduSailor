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

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.List;

public class RadioCommunicator {

    public class State {
        public int wind;
        public double htw;
        public int heading;
        public double dtw;

        public State (int wind, double htw, int heading, double dtw) {
            this.wind = wind;
            this.htw = htw;
            this.heading = heading;
            this.dtw = dtw;
        }
    }

    public synchronized void setState(int winch, int rudder) {
        radioCommand = new byte[] { START_CHAR, (byte)winch, (byte)rudder, END_CHAR };
    }

    public synchronized State getState() {
        return new State(wind, wp_heading, ahrs_heading, wp_distance);
    }

    private String gps_aprs_lat, gps_aprs_lon;
    private int gps_course, ahrs_heading, current_rudder, current_winch, wind;
    private double gps_lat, gps_lon, gps_speed, current_roll, heel_adjust, wp_heading, wp_distance, voltage;
    private long millis, cycle;

    private String lastLine;

    private long lastLineTime;
    private int failLineCount;

    private static final char START_CHAR = 'x';
    private static final char END_CHAR = 'd';

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
        UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
        if (connection == null) {
            Log.d(TAG, "Unable to connect");
            // You probably need to call UsbManager.requestPermission(driver.getDevice(), ..)
            return;
        }

        terminated = false;

        synchronized (this) {
            port = driver.getPorts().get(0);
            try {
                port.open(connection);
                port.setParameters(9600, 8, 1, 0);
            } catch (IOException e) {
                Log.e(TAG, "Exception while connecting to device", e);
            }

            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
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
            terminated = true;
        } catch (IOException ex) {
            Log.e(TAG, "Exception while shutting down", ex);
        }
    }

    private void read() throws IOException {
        StringBuilder sb = new StringBuilder();

        long lastTransmit = 0;

        byte[] buffer = new byte[1];

        while (!terminated) {
            if (port.read(buffer, 100) < 1)
                continue;

            char current = (char) buffer[0];

            if (current != START_CHAR)
                continue;

            while (current != END_CHAR) {
                if (port.read(buffer, 100) < 1)
                    break;

                current = (char) buffer[0];
                sb.append(current);
            }

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

    private void transmit() throws IOException {
        port.write(radioCommand, 200);
    }

    private String validate(String line) {
        // first byte is 'x', last byte is XOR byte, without the x

        if (line.charAt(0) != START_CHAR)
            return null;

//        int xor = line.charAt(line.length() - 1);
//        int comp = 0;
//
//        for (int i=1; i<line.length(); i++)
//            comp ^= line.charAt(i);
//
//        if (xor != comp)
//            return null;

        return line.substring(1, line.length() - 2);
    }

    private void processLine(String lineIn) {

        String validated = validate(lineIn);

        if (validated == null) {
            failLineCount++;
            return;
        }

        lastLineTime = System.currentTimeMillis();
        failLineCount = 0;

        String[] parts = lineIn.split(",");

        int i = 0;

        gps_aprs_lat = parts[++i];
        gps_aprs_lon = parts[++i];
        gps_lat = Double.parseDouble(parts[++i]);
        gps_lon = Double.parseDouble(parts[++i]);
        millis = Integer.parseInt(parts[++i]);
        gps_speed = Double.parseDouble(parts[++i]);
        gps_course = Integer.parseInt(parts[++i]);
        ahrs_heading = Integer.parseInt(parts[++i]);
        current_roll = Double.parseDouble(parts[++i]);
        heel_adjust = Double.parseDouble(parts[++i]);
        wind = Integer.parseInt(parts[++i]);
        wp_heading = Double.parseDouble(parts[++i]);
        wp_distance = Double.parseDouble(parts[++i]);
        current_rudder = Integer.parseInt(parts[++i]);
        current_winch = Integer.parseInt(parts[++i]);
        voltage = Double.parseDouble(parts[++i]);
        cycle = Integer.parseInt(parts[++i]);

        lastLine = lineIn;
    }

    public synchronized boolean isRunning() { return !terminated; }
}
