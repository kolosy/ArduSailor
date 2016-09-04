package com.kolosy.ardusailorcontroller;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Handler;
import android.os.IBinder;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.SeekBar;
import android.widget.TextView;


public class MainActivity extends ActionBarActivity {

    GaugeView gvWind, gvHTW, gvHeading;
    TextView lblDTW;

    SeekBar sbRudder, sbWinch;

    private RadioCommunicator communicator;
    private boolean mResumed;

    private final Handler handler=new Handler();

    private final Runnable updateTask=new Runnable() {
        @Override
        public void run() {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {

                    RadioCommunicator.State state = communicator.getState();

                    gvHeading.setTargetValue(state.heading);
                    gvWind.setTargetValue(state.wind);
                    gvHTW.setTargetValue((float) state.htw);

                    lblDTW.setText(String.valueOf(state.dtw));
                }
            });

            handler.postDelayed(this,1000);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        lblDTW = (TextView)findViewById(R.id.lblDTW);
        lblDTW.setText("0");

        gvWind = (GaugeView)findViewById(R.id.gvWind);
        gvHTW = (GaugeView)findViewById(R.id.gvHTW);
        gvHeading = (GaugeView)findViewById(R.id.gvHeading);

        gvWind.setTargetValue(0);
        gvHeading.setTargetValue(0);
        gvHTW.setTargetValue(0);

        sbRudder = (SeekBar)findViewById(R.id.sbRudder);
        sbWinch = (SeekBar)findViewById(R.id.sbWinch);

        PendingIntent mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
        IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
        registerReceiver(mUsbReceiver, filter);

        communicator = new RadioCommunicator(this, mPermissionIntent);

        sbRudder.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                communicator.setState(progress, sbWinch.getProgress());
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        sbWinch.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                communicator.setState(sbRudder.getProgress(), progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

    }

    @Override
    public void onResume() {
        super.onResume();

        startTimer();

        if (!communicator.isRunning())
            communicator.start();
    }

    @Override
    public void onPause() {
        super.onPause();

        stopTimer();
        communicator.stop();
    }

    private void startTimer() {
        handler.postDelayed(updateTask,1000);
    }

    private void stopTimer() {
        handler.removeCallbacks(updateTask);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onStop() {
        super.onStop();

        communicator.stop();
        unregisterReceiver(mUsbReceiver);
    }

    private static final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";

    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {

        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this) {
                    UsbDevice device = (UsbDevice)intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);

                    if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                        if (device != null && !communicator.isRunning())
                            communicator.start();
                    }
                    else {
                        Log.d("RA", "permission denied for device " + device);
                    }
                }
            }
        }
    };

//    @Override
//    public void onWindowFocusChanged(boolean hasFocus) {
//        super.onWindowFocusChanged(hasFocus);
//        if (hasFocus) {
//            findViewById(R.id.rootLayout).setSystemUiVisibility(
//                    View.SYSTEM_UI_FLAG_LAYOUT_STABLE
//                            | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
//                            | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
//                            | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
//                            | View.SYSTEM_UI_FLAG_FULLSCREEN
//                            | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);}
//    }
}
