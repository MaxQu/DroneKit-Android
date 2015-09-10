package org.droidplanner.services.android.ui.fragment;


import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;

import org.droidplanner.services.android.R;

import org.droidplanner.services.android.core.helpers.orientation.AxisAngle;
import org.droidplanner.services.android.core.helpers.orientation.EulerAngles;
import org.droidplanner.services.android.core.helpers.orientation.GravityVector;
import org.droidplanner.services.android.core.helpers.orientation.Quaternion;
import org.droidplanner.services.android.core.helpers.orientation.Vector3f;

import static android.util.FloatMath.cos;
import static android.util.FloatMath.sin;
import static android.util.FloatMath.sqrt;

/**
 * Provide a view of recommended apps that are compatible with 3DR Services.
 */
public class GCSAttitudeAppsFragment extends Fragment implements SensorEventListener {

    private static final int UPDATE_THRESHOLD = 200;
    private static final float EPSILON = 0.01f;
    private static final float NS2S = 1.0f / 1000000000.0f;

    private SensorManager mSensorManager;
    private Sensor mAccelerometer, mGyroscope;

    //Raw sensor data storage
    private static final GravityVector accelVector = new GravityVector(0,0,0);
    private static final Vector3f gyroVector= new Vector3f(0,0,0);
    //delta means the instataneous rotation segment representation
    private static final Quaternion deltaQuaternion = new Quaternion(0,0,0,1); //Quaternion, initialized at zero rotation
    private static final Quaternion quaternionCurrent = new Quaternion(0,0,0,1); //initialized at zero rotation
    //current rotation representation
    private static final EulerAngles eulerQuatCurrent=new EulerAngles(0,0,0);
    //freeze rotation representation when hit Lock
    private static final Quaternion quaternionFreeze = new Quaternion(0,0,0,1);
    //The difference rotation between freeze and current
    private static final EulerAngles diffEulerQuat = new EulerAngles(0,0,0);
    //use to perform gravity correction
    private static final AxisAngle axisAngle=new AxisAngle(0,0,0,0);
    private static final Quaternion correctQuat= new Quaternion(0,0,0,1);

    //For visualization
    private TextView aXValueView, aYValueView, aZValueView;
    private TextView gXValueView, gYValueView, gZValueView;
    private TextView eXRvValueView, eYRvValueView, eZRvValueView;

    //For visualization update
    private static float etimestamp; //time log for calcualte euler angles
    private long aLastUpdate, gLastUpdate;

    //For synchronization
    protected final Object syncToken = new Object();

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState){
        return inflater.inflate(R.layout.fragment_gcs_attitude_apps, container, false);
    }

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        //view composition
        aXValueView = (TextView) view.findViewById(R.id.a_x_value_view);
        aYValueView = (TextView) view.findViewById(R.id.a_y_value_view);
        aZValueView = (TextView) view.findViewById(R.id.a_z_value_view);
        gXValueView = (TextView) view.findViewById(R.id.g_x_value_view);
        gYValueView = (TextView) view.findViewById(R.id.g_y_value_view);
        gZValueView = (TextView) view.findViewById(R.id.g_z_value_view);

        eXRvValueView = (TextView) view.findViewById(R.id.e_x_rv_value_view);
        eYRvValueView = (TextView) view.findViewById(R.id.e_y_rv_value_view);
        eZRvValueView = (TextView) view.findViewById(R.id.e_z_rv_value_view);

        final Button resetRV_Button = (Button) view.findViewById(R.id.reset_rv_button);

        resetRV_Button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Perform action on click
                quaternionFreeze.set(quaternionCurrent);
            }
        });


        // Get reference to SensorManager
        mSensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);

        // Get reference to Accelerometer
        if (null == (mAccelerometer = mSensorManager
                .getDefaultSensor(Sensor.TYPE_ACCELEROMETER)) ||
                null == (mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)))
            getActivity().finish();
    }
    // Process new reading
    @Override
    public void onSensorChanged(SensorEvent event) {

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

            synchronized (syncToken) {
                accelVector.setXYZ(event.values);
            }

            long actualTime = System.currentTimeMillis();

            if (actualTime - aLastUpdate > UPDATE_THRESHOLD) {

                aLastUpdate = actualTime;


                aXValueView.setText(String.format("%.4f", accelVector.getX()));
                aYValueView.setText(String.format("%.4f", accelVector.getY()));
                aZValueView.setText(String.format("%.4f", accelVector.getZ()));

            }
        }


        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
//			handle gyro reading
            long actualTime = System.currentTimeMillis();
            gyroVector.setXYZ(event.values);

            if (etimestamp != 0) {
                final float dT = (event.timestamp - etimestamp) * NS2S;
                // Axis of the rotation sample, not normalized yet.
                float axisX = gyroVector.getX();
                float axisY = gyroVector.getY();
                float axisZ = gyroVector.getZ();

                // Calculate the angular speed of the sample
                float omegaMagnitude = sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);

                // Normalize the rotation vector if it's big enough to get the axis
                // (that is, EPSILON should represent your maximum allowable margin of error)
                //this normalization make sure the [axisX, axisY, axisZ] is a unit vector
                if (omegaMagnitude > EPSILON) {
                    axisX /= omegaMagnitude;
                    axisY /= omegaMagnitude;
                    axisZ /= omegaMagnitude;
                }

                // Integrate around this axis with the angular speed by the timestep
                // in order to get a delta rotation from this sample over the timestep
                // We will convert this axis-angle representation of the delta rotation
                // into a quaternion before turning it into the rotation matrix.
                float thetaOverTwo = omegaMagnitude * dT / 2.0f;
                float sinThetaOverTwo = sin(thetaOverTwo);
                float cosThetaOverTwo = cos(thetaOverTwo);
                //the rotation vector is a quaternion
                deltaQuaternion.setX(sinThetaOverTwo * axisX);
                deltaQuaternion.setY(sinThetaOverTwo * axisY);
                deltaQuaternion.setZ(sinThetaOverTwo * axisZ);
                deltaQuaternion.setW(cosThetaOverTwo);
            }
            etimestamp = event.timestamp;


            ////////////////////////////////////////////
            //option 2, using Quat//////////////////////
            //////////////////////////////////////////
            //R b to e is recursive, R=Rz*Ry*Rx, see ref2. page 22
            synchronized (syncToken) {
                quaternionCurrent.multiplyQuat(deltaQuaternion, quaternionCurrent);
                quaternionCurrent.toCorrectAxisAngleByGravity(accelVector, axisAngle);
                axisAngle.toQuaternion(correctQuat);
                quaternionCurrent.multiplyQuat(correctQuat, quaternionCurrent);
                quaternionCurrent.toEulerAngles(eulerQuatCurrent);
                quaternionFreeze.getDiffQuaternionToTarget(quaternionCurrent).toEulerAngles(diffEulerQuat);
            }


            if (actualTime - gLastUpdate > UPDATE_THRESHOLD) {

                gLastUpdate = actualTime;

//				float gx = event.values[0], gy = event.values[1], gz = event.values[2];

                gXValueView.setText(String.format("%.4f", gyroVector.getX()));
                gYValueView.setText(String.format("%.4f", gyroVector.getY()));
                gZValueView.setText(String.format("%.4f", gyroVector.getZ()));


                eXRvValueView.setText(String.format("%.4f",diffEulerQuat.getX()*EulerAngles.R2D));
                eYRvValueView.setText(String.format("%.4f",diffEulerQuat.getY()*EulerAngles.R2D));
                eZRvValueView.setText(String.format("%.4f",diffEulerQuat.getZ()*EulerAngles.R2D));
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // N/A
    }


    @Override
    public void onResume() {
        super.onResume();

        mSensorManager.registerListener(this, mAccelerometer,
                SensorManager.SENSOR_DELAY_UI);

        aLastUpdate = System.currentTimeMillis();

        mSensorManager.registerListener(this, mGyroscope,
                SensorManager.SENSOR_DELAY_UI);

        gLastUpdate = System.currentTimeMillis();


    }

    // Unregister listener
    @Override
    public void onPause() {
        mSensorManager.unregisterListener(this);
        super.onPause();
    }

    @Override
    public void onStart(){
        super.onStart();

        mSensorManager.registerListener(this, mAccelerometer,
                SensorManager.SENSOR_DELAY_UI);

        aLastUpdate = System.currentTimeMillis();

        mSensorManager.registerListener(this, mGyroscope,
                SensorManager.SENSOR_DELAY_UI);

        gLastUpdate = System.currentTimeMillis();


    }

    @Override
    public void onStop(){
        mSensorManager.unregisterListener(this);
        super.onStop();
    }
}
