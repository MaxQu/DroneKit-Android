package org.droidplanner.services.android.ui.fragment;


import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.support.v4.content.LocalBroadcastManager;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;

import com.o3dr.services.android.lib.drone.property.Attitude;
import com.o3dr.services.android.lib.drone.property.Vector3;
import com.o3dr.services.android.lib.gcs.event.GCSEvent;

import org.droidplanner.services.android.R;
import org.droidplanner.services.android.api.DroidPlannerService;
import org.droidplanner.services.android.core.helpers.orientation.EulerAngles;
import org.droidplanner.services.android.ui.activity.MainActivity;

/**
 * Provide a view of recommended apps that are compatible with 3DR Services.
 */
public class GCSAttitudeAppsFragment extends Fragment{

    private final static String TAG = MainActivity.class.getSimpleName();
    public static final String ACTION_GCS_INIT_ATT_LOCKED = GCSEvent.GCS_INIT_ATTITUDE_LOCKED;
    public static final String ACTION_GCS_GYRO_UPDATED = GCSEvent.GCS_GYRO_UPDATED;
    public static final String ACTION_GCS_ACCEL_UPDATED = GCSEvent.GCS_ACCEL_UPDATED;
    public static final String ACTION_GCS_ATTITUDE_UPDATED = GCSEvent.GCS_ATTITUDE_UPDATED;

    private MainActivity parent;
    private LocalBroadcastManager lbm;

    //For visualization
    private static final long UPDATE_INTERVAL = 100;
    private TextView aXValueView, aYValueView, aZValueView;
    private TextView gXValueView, gYValueView, gZValueView;
    private TextView eXRvValueView, eYRvValueView, eZRvValueView;
    private Vector3 gyroVector = new Vector3();
    private Vector3 accelVector = new Vector3();
    private Attitude attVector = new Attitude();

    //For visualization update
    private static float etimestamp; //time log for calcualte euler angles
    private long aLastUpdate, gLastUpdate, tLastUpdate;

    private final static IntentFilter gcsAttitudeFilter = new IntentFilter();
    static {
        gcsAttitudeFilter.addAction(ACTION_GCS_GYRO_UPDATED);
        gcsAttitudeFilter.addAction(ACTION_GCS_ACCEL_UPDATED);
        gcsAttitudeFilter.addAction(ACTION_GCS_ATTITUDE_UPDATED);
    }

    private final BroadcastReceiver gcsAttEventReceiver = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            switch (action) {
                case ACTION_GCS_GYRO_UPDATED:
                    updateGCSGyroView();
                    break;

                case ACTION_GCS_ACCEL_UPDATED:
                    updateGCSAccelView();
                    break;

                case ACTION_GCS_ATTITUDE_UPDATED:
                    updateGCSAttView();
                    break;
            }

        }
    };

    @Override
    public void onAttach(Activity activity){
        super.onAttach(activity);
        if(!(activity instanceof MainActivity)){
            throw new IllegalStateException("Parent must be an instance of " + MainActivity.class.getName());
        }

        parent = (MainActivity) activity;
    }


    @Override
    public void onDetach(){
        super.onDetach();
        parent = null;
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState){
        return inflater.inflate(R.layout.fragment_gcs_attitude_apps, container, false);
    }

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        lbm = LocalBroadcastManager.getInstance(getActivity().getApplicationContext());
        lbm.registerReceiver(gcsAttEventReceiver, gcsAttitudeFilter);
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
                lbm.sendBroadcast(new Intent(ACTION_GCS_INIT_ATT_LOCKED));
                getActivity().getApplicationContext().sendBroadcast(new Intent(ACTION_GCS_INIT_ATT_LOCKED));
            }
        });

    }

    void updateGCSGyroView() {

        long actualTime = System.currentTimeMillis();
        if ((actualTime-gLastUpdate)>UPDATE_INTERVAL) {
            if (parent!=null && parent.getDroneAccess()!=null) {
                gyroVector=parent.getDroneAccess().getDPService().getGCSGyro();
            }
            gXValueView.setText(String.format("%.4f", gyroVector.getX()));
            gYValueView.setText(String.format("%.4f", gyroVector.getY()));
            gZValueView.setText(String.format("%.4f", gyroVector.getZ()));
            gLastUpdate = actualTime;
        }
    }

    void updateGCSAccelView() {
        long actualTime = System.currentTimeMillis();
        if ((actualTime-aLastUpdate)>UPDATE_INTERVAL) {
            if (parent!=null && parent.getDroneAccess()!=null) {
                accelVector = parent.getDroneAccess().getDPService().getGCSAccel();
            }
            aXValueView.setText(String.format("%.4f", accelVector.getX()));
            aYValueView.setText(String.format("%.4f", accelVector.getY()));
            aZValueView.setText(String.format("%.4f", accelVector.getZ()));
            aLastUpdate = actualTime;
        }
    }

    void updateGCSAttView() {
        long actualTime = System.currentTimeMillis();
        if ((actualTime-tLastUpdate)>UPDATE_INTERVAL) {
            if (parent!=null && parent.getDroneAccess()!=null) {
                attVector = parent.getDroneAccess().getDPService().getGCSAttitude();
            }
            eXRvValueView.setText(String.format("%.4f", attVector.getYaw() * EulerAngles.R2D));
            eYRvValueView.setText(String.format("%.4f", attVector.getPitch() * EulerAngles.R2D));
            eZRvValueView.setText(String.format("%.4f", attVector.getRoll() * EulerAngles.R2D));
            tLastUpdate = actualTime;
//            Log.e(TAG,"update GCS attitude view");
        }

    }


    @Override
    public void onResume() {
        super.onResume();
        lbm.registerReceiver(gcsAttEventReceiver, gcsAttitudeFilter);
        aLastUpdate = System.currentTimeMillis();
        gLastUpdate = System.currentTimeMillis();
    }

    // Unregister listener
    @Override
    public void onPause() {
        lbm.unregisterReceiver(gcsAttEventReceiver);
        super.onPause();
    }

    @Override
    public void onStart(){
        super.onStart();
        lbm.registerReceiver(gcsAttEventReceiver, gcsAttitudeFilter);
        gLastUpdate = System.currentTimeMillis();


    }

    @Override
    public void onStop(){
        lbm.unregisterReceiver(gcsAttEventReceiver);
        super.onStop();
    }
}
