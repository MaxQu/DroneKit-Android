package org.droidplanner.services.android.api;

import android.os.Bundle;
import android.os.RemoteException;
import android.util.Log;

import com.o3dr.services.android.lib.drone.connection.ConnectionParameter;
import com.o3dr.services.android.lib.drone.property.Attitude;
import com.o3dr.services.android.lib.drone.property.Vector3;
import com.o3dr.services.android.lib.gcs.event.GCSEvent;
import com.o3dr.services.android.lib.model.IApiListener;
import com.o3dr.services.android.lib.model.IDroidPlannerServices;
import com.o3dr.services.android.lib.model.IDroneApi;
import com.o3dr.services.android.lib.util.version.VersionUtils;

import org.droidplanner.services.android.BuildConfig;
import org.droidplanner.services.android.core.drone.DroneManager;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by fhuya on 11/3/14.
 */
final class DPServices extends IDroidPlannerServices.Stub {

    private final static String TAG = DPServices.class.getSimpleName();

    private DroidPlannerService serviceRef;

    DPServices(DroidPlannerService service) {
        serviceRef = service;
    }

    void destroy(){
        serviceRef = null;
    }

    @Override
    public int getServiceVersionCode() throws RemoteException {
        return BuildConfig.VERSION_CODE;
    }

    @Override
    public int getApiVersionCode() throws RemoteException {
        return VersionUtils.getCoreLibVersion(serviceRef.getApplicationContext());
    }

    @Override
    public IDroneApi registerDroneApi(IApiListener listener, String appId) throws RemoteException {
        return serviceRef.registerDroneApi(listener, appId);
    }

    @Override
    public Bundle[] getConnectedApps(String requesterId) throws RemoteException {
        Log.d(TAG, "List of connected apps request from " + requesterId);

        List<Bundle> appsInfo = new ArrayList<>();
        for(DroneApi droneApi : serviceRef.droneApiStore.values()){
            if(droneApi.isConnected()){
                DroneManager droneManager = droneApi.getDroneManager();
                if(droneManager != null) {
                    final ConnectionParameter droneParams = droneApi.getDroneManager().getConnectionParameter();
                    final ConnectionParameter sanitizedParams = new ConnectionParameter(droneParams.getConnectionType(),
                            droneParams.getParamsBundle(), null);

                    Bundle info = new Bundle();
                    info.putString(GCSEvent.EXTRA_APP_ID, droneApi.getOwnerId());
                    info.putParcelable(GCSEvent.EXTRA_VEHICLE_CONNECTION_PARAMETER, sanitizedParams);

                    appsInfo.add(info);
                }
            }
        }

        return appsInfo.toArray(new Bundle[appsInfo.size()]);
    }

    @Override
    public void releaseDroneApi(IDroneApi dpApi) throws RemoteException {
        Log.d(TAG, "Releasing acquired drone api handle.");
        if(dpApi instanceof DroneApi) {
            serviceRef.releaseDroneApi(((DroneApi) dpApi).getOwnerId());
        }
    }

    @Override
    public Attitude getGCSAttitude() {
        return serviceRef.getGCSAttitude();
    }

    @Override
    public Attitude getGCSAttitudeLocked() {
        return serviceRef.getGCSAttitudeLocked();
    }

    @Override
    public Vector3 getGCSGyro() {
        return serviceRef.getGCSGyro();
    }

    @Override
    public Vector3 getGCSAccel() {
        return serviceRef.getGCSAccel();
    }

    @Override
    public boolean hasGCSAccel() {return serviceRef.hasGCSAccel();}

    @Override
    public boolean hasGCSGyro() {return serviceRef.hasGCSGyro();}
}
