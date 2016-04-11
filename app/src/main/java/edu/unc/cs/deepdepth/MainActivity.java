package edu.unc.cs.deepdepth;

import android.graphics.ImageFormat;
import android.os.Bundle;
import android.hardware.Camera;
import android.support.design.widget.FloatingActionButton;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.JsonWriter;
import android.util.Log;
import android.view.SurfaceView;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

public class MainActivity extends AppCompatActivity {

    private static String TAG = "DeepDepth/MainActivity";

    protected int mCameraIndex = -1;
    protected Camera mCamera;
    protected Tango mTango;
    protected TangoConfig mTangoConfig;
    protected TangoXyzIjData mCurrentPointCloud;
    protected TangoPoseData mCurrentPose;
    protected TangoPoseData mPointCloudPose;

    protected void setupCameraPreview() throws IOException {
        if (mCameraIndex >= 0) {
            mCamera.stopPreview();
            mCamera.release();
            mCameraIndex += 1;
            if (mCameraIndex > 2) {
                mCameraIndex = 0;
            }
        } else {
            mCameraIndex = 0;
        }
        SurfaceView sv = (SurfaceView) findViewById(R.id.surfaceView);
        mCamera = Camera.open(mCameraIndex);
        Camera.Parameters params = mCamera.getParameters();
        params.setPictureFormat(ImageFormat.JPEG);
        params.setJpegQuality(100);
        mCamera.setPreviewDisplay(sv.getHolder());
        mCamera.startPreview();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        mTango = new Tango(this);
        mTangoConfig = new TangoConfig();
        mTangoConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
        mTangoConfig.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
        mTangoConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        FloatingActionButton fab = (FloatingActionButton) findViewById(R.id.fab);
        fab.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                try {
                    setupCameraPreview();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
        FloatingActionButton takePictureCab = (FloatingActionButton) findViewById(R.id.take_picture);
        takePictureCab.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                mCamera.takePicture(null, null, new Camera.PictureCallback() {
                    @Override
                    public void onPictureTaken(byte[] data, Camera camera) {
                        File dir = new File("/sdcard/deep_depth/");
                        dir.mkdirs();
                        String timeString = Long.toString(System.currentTimeMillis());
                        File file = new File(dir, timeString + ".jpg");
                        File pcFile = new File(dir, timeString + ".vtk");
                        File poseFile = new File(dir, timeString + ".json");
                        File intrinsicsFile = new File(dir, timeString + ".intrinsics");
                        try {

                            // write intrinsics
                            TangoCameraIntrinsics cal = mTango.getCameraIntrinsics(mCameraIndex);
                            Log.d(TAG, "Intrinsics");
                            Log.d(TAG, Double.toString(cal.fx));
                            Log.d(TAG, Double.toString(cal.fy));
                            Log.d(TAG, Double.toString(cal.cx));
                            Log.d(TAG, Double.toString(cal.cy));
                            JsonWriter writer = new JsonWriter(new OutputStreamWriter(new FileOutputStream(intrinsicsFile)));
                            writer.beginObject();
                            writer.name("fx").value(cal.fx);
                            writer.name("fy").value(cal.fy);
                            writer.name("cx").value(cal.cx);
                            writer.name("cy").value(cal.cy);
                            writer.endObject();
                            writer.close();

                            Log.d(TAG, mCurrentPose.toString());
                            double[] rot = mCurrentPose.rotation;
                            double[] trans = mCurrentPose.translation;
                            writer = new JsonWriter(new OutputStreamWriter(new FileOutputStream(poseFile)));
                            writer.beginObject();
                            writer.name("p");
                            writer.beginArray();
                            for (int i = 0; i < trans.length; ++i) {
                                writer.value(trans[i]);
                            }
                            writer.endArray();
                            writer.name("q");
                            writer.beginArray();
                            for (int i = 0; i < rot.length; ++i) {
                                writer.value(rot[i]);
                            }
                            writer.endArray();
                            writer.endObject();
                            writer.close();

                            // write image
                            FileOutputStream f = new FileOutputStream(file);
                            f.write(data);
                            f.close();

                            // write point cloud
                            DataOutputStream out = new DataOutputStream(new BufferedOutputStream(
                                    new FileOutputStream(pcFile)));
                            out.write(("# vtk DataFile Version 3.0\n" +
                                    "vtk output\n" +
                                    "BINARY\n" +
                                    "DATASET POLYDATA\n" +
                                    "POINTS " + mCurrentPointCloud.xyzCount + " float\n").getBytes());
                            for (int i = 0; i < mCurrentPointCloud.xyzCount; i++) {
                                out.writeFloat(mCurrentPointCloud.xyz.get(3 * i));
                                out.writeFloat(mCurrentPointCloud.xyz.get(3 * i + 1));
                                out.writeFloat(mCurrentPointCloud.xyz.get(3 * i + 2));
                            }
                            out.write(("\nVERTICES 1 " + String.valueOf(mCurrentPointCloud.xyzCount + 1) + "\n").getBytes());
                            out.writeInt(mCurrentPointCloud.xyzCount);
                            for (int i = 0; i < mCurrentPointCloud.xyzCount; i++) {
                                out.writeInt(i);
                            }
                            out.write(("\nFIELD FieldData 1\n" + "timestamp 1 1 float\n").getBytes());
                            out.writeFloat((float) mCurrentPointCloud.timestamp);
                            out.close();
                        } catch (FileNotFoundException e) {
                            e.printStackTrace();
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                        camera.startPreview();
                    }
                });
            }
        });

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

    private void setupTangoListeners() {
        final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        // Listen for new Tango data
        mTango.connectListener(framePairs, new OnTangoUpdateListener() {

            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                mCurrentPose = pose;
            }

            @Override
            public void onXyzIjAvailable(TangoXyzIjData pointCloud) {
                Log.d(TAG, "XyzIj Callback");
                mCurrentPointCloud = pointCloud;
                mPointCloudPose = mCurrentPose;
            }

            @Override
            public void onFrameAvailable(int i) {
            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
            }
        });
    }

    @Override
    protected void onPause() {
        super.onPause();
        try {
            mTango.disconnect();
        } catch (TangoErrorException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        try {
            mTango.connect(mTangoConfig);
        } catch (TangoOutOfDateException e) {
            e.printStackTrace();
        }
        setupTangoListeners();
    }
}
