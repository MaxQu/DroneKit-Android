package org.droidplanner.services.android.api;

import android.annotation.SuppressLint;
import android.app.Notification;
import android.app.PendingIntent;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.support.v4.content.LocalBroadcastManager;
import android.text.TextUtils;
import android.util.Log;
import android.widget.TextView;

import com.google.android.gms.analytics.HitBuilders;
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter;
import com.o3dr.services.android.lib.drone.connection.ConnectionType;
import com.o3dr.services.android.lib.drone.mission.item.complex.CameraDetail;
import com.o3dr.services.android.lib.drone.property.Attitude;
import com.o3dr.services.android.lib.drone.property.Vector3;
import com.o3dr.services.android.lib.gcs.event.GCSEvent;
import com.o3dr.services.android.lib.model.IApiListener;
import com.o3dr.services.android.lib.model.IDroidPlannerServices;

import org.droidplanner.services.android.core.MAVLink.connection.MavLinkConnection;
import org.droidplanner.services.android.core.MAVLink.connection.MavLinkConnectionListener;
import org.droidplanner.services.android.core.helpers.orientation.AxisAngle;
import org.droidplanner.services.android.core.helpers.orientation.EulerAngles;
import org.droidplanner.services.android.core.helpers.orientation.GravityVector;
import org.droidplanner.services.android.core.helpers.orientation.Quaternion;
import org.droidplanner.services.android.core.helpers.orientation.Vector3f;
import org.droidplanner.services.android.core.survey.CameraInfo;
import org.droidplanner.services.android.R;
import org.droidplanner.services.android.communication.connection.AndroidMavLinkConnection;
import org.droidplanner.services.android.communication.connection.AndroidTcpConnection;
import org.droidplanner.services.android.communication.connection.AndroidUdpConnection;
import org.droidplanner.services.android.communication.connection.BluetoothConnection;
import org.droidplanner.services.android.communication.connection.usb.UsbConnection;
import org.droidplanner.services.android.core.drone.DroneManager;
import org.droidplanner.services.android.exception.ConnectionException;
import org.droidplanner.services.android.core.drone.DroneEventsListener;
import org.droidplanner.services.android.ui.activity.MainActivity;
import org.droidplanner.services.android.utils.Utils;
import org.droidplanner.services.android.utils.analytics.GAUtils;
import org.droidplanner.services.android.utils.file.IO.CameraInfoLoader;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

import timber.log.Timber;

import static android.util.FloatMath.cos;
import static android.util.FloatMath.sin;
import static android.util.FloatMath.sqrt;

/**
 * 3DR Services background service implementation.
 */
public class DroidPlannerService extends Service implements SensorEventListener {
    private final static String TAG = MainActivity.class.getSimpleName();
    /**
     * Status bar notification id
     */
    private static final int FOREGROUND_ID = 101;

    /**
     * Set of actions to notify the local app's components of the service events.
     */
    public static final String ACTION_DRONE_CREATED = Utils.PACKAGE_NAME + ".ACTION_DRONE_CREATED";
    public static final String ACTION_DRONE_DESTROYED = Utils.PACKAGE_NAME + ".ACTION_DRONE_DESTROYED";
    public static final String ACTION_KICK_START_DRONESHARE_UPLOADS = Utils.PACKAGE_NAME + ".ACTION_KICK_START_DRONESHARE_UPLOADS";
    public static final String ACTION_RELEASE_API_INSTANCE = Utils.PACKAGE_NAME + ".action.RELEASE_API_INSTANCE";
    public static final String EXTRA_API_INSTANCE_APP_ID = "extra_api_instance_app_id";
    public static final String ACTION_GCS_GYRO_UPDATED = GCSEvent.GCS_GYRO_UPDATED;
    public static final String ACTION_GCS_ATTITUDE_UPDATED = GCSEvent.GCS_ATTITUDE_UPDATED;
    public static final String ACTION_GCS_ACCEL_UPDATED = GCSEvent.GCS_ACCEL_UPDATED;
    public static final String ACTION_GCS_INIT_ATT_LOCKED = GCSEvent.GCS_INIT_ATTITUDE_LOCKED;

    /**
     * used to calculate gcs attitude
     */
    private static final int BCAST_INTERVAL = 50;
    private static final float EPSILON = 0.01f;
    private static final float NS2S = 1.0f / 1000000000.0f;

    private SensorManager mSensorManager;
    private Sensor mAccelerometer, mGyroscope;

    //Use for send data between interfaces
    private static final Attitude gcsAtt = new Attitude();
    private static final Attitude gcsAttLocked = new Attitude();
    private static final Vector3 gcsGyro = new Vector3();
    private static final Vector3 gcsAccel = new Vector3();

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
    private static final EulerAngles eulerQuatFreeze = new EulerAngles(0,0,0);
    //The difference rotation between freeze and current
    private static final EulerAngles diffEulerQuat = new EulerAngles(0,0,0);
    //use to perform gravity correction
    private static final AxisAngle axisAngle=new AxisAngle(0,0,0,0);
    private static final Quaternion correctQuat= new Quaternion(0,0,0,1);

    //For visualization update
    private static float etimestamp; //time log for calcualte euler angles
    private long aLastBcast, gLastBcast;

    //For synchronization
    protected final Object syncToken = new Object();

    private final static IntentFilter gcsInitAttLockFilter = new IntentFilter();
    static {
        gcsInitAttLockFilter.addAction(ACTION_GCS_INIT_ATT_LOCKED);
    }

    private final BroadcastReceiver gcsInitAttLockReceiver = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
//            Log.e(TAG,"message received");
            gcsInitAttLock();
        }
    };

    /**
     * Used to broadcast service events.
     */
    private LocalBroadcastManager lbm;
    private Context context;

    /**
     * Stores drone api instances per connected client. The client are denoted by their app id.
     */
    final ConcurrentHashMap<String, DroneApi> droneApiStore = new ConcurrentHashMap<>();

    /**
     * Caches mavlink connections per connection type.
     */
    final ConcurrentHashMap<String, AndroidMavLinkConnection> mavConnections = new ConcurrentHashMap<>();

    /**
     * Caches drone managers per connection type.
     */
    final ConcurrentHashMap<ConnectionParameter, DroneManager> droneManagers = new ConcurrentHashMap<>();

    private DPServices dpServices;
    private DroneAccess droneAccess;
    private MavLinkServiceApi mavlinkApi;

    private CameraInfoLoader cameraInfoLoader;
    private List<CameraDetail> cachedCameraDetails;

    /**
     * Generate a drone api instance for the client denoted by the given app id.
     *
     * @param listener Used to retrieve api information.
     * @param appId    Application id of the connecting client.
     * @return a IDroneApi instance
     */
    DroneApi registerDroneApi(IApiListener listener, String appId) {
        if (listener == null)
            return null;

        DroneApi droneApi = new DroneApi(this, listener, appId);
        droneApiStore.put(appId, droneApi);
        lbm.sendBroadcast(new Intent(ACTION_DRONE_CREATED));
        broadcastGCSAttData();
        updateForegroundNotification();
        return droneApi;
    }

    /**
     * Release the drone api instance attached to the given app id.
     *
     * @param appId Application id of the disconnecting client.
     */
    void releaseDroneApi(String appId) {
        if (appId == null)
            return;

        DroneApi droneApi = droneApiStore.remove(appId);
        if (droneApi != null) {
            Timber.d("Releasing drone api instance for " + appId);
            droneApi.destroy();
            lbm.sendBroadcast(new Intent(ACTION_DRONE_DESTROYED));
            broadcastGCSAttData();
            updateForegroundNotification();
        }
    }

    /**
     * Establish a connection with a vehicle using the given connection parameter.
     *
     * @param connParams Parameters used to connect to the vehicle.
     * @param appId      Application id of the connecting client.
     * @param listener   Callback to receive drone events.
     * @return A DroneManager instance which acts as router between the connected vehicle and the listeneing client(s).
     * @throws ConnectionException
     */
    DroneManager connectDroneManager(ConnectionParameter connParams, String appId, DroneEventsListener listener) throws ConnectionException {
        if (connParams == null || TextUtils.isEmpty(appId) || listener == null)
            return null;

        DroneManager droneMgr = droneManagers.get(connParams);
        if (droneMgr == null) {
            Timber.d("Generating new drone manager.");
            droneMgr = new DroneManager(getApplicationContext(), connParams, new Handler(Looper.getMainLooper()),
                    mavlinkApi);
            droneManagers.put(connParams, droneMgr);
        }

        Timber.d("Drone manager connection for " + appId);
        droneMgr.connect(appId, listener);
        return droneMgr;
    }

    /**
     * Disconnect the given client from the vehicle managed by the given drone manager.
     *
     * @param droneMgr Handler for the connected vehicle.
     * @param appId    Application id of the disconnecting client.
     * @throws ConnectionException
     */
    void disconnectDroneManager(DroneManager droneMgr, String appId) throws ConnectionException {
        if (droneMgr == null || TextUtils.isEmpty(appId))
            return;

        Timber.d("Drone manager disconnection for " + appId);
        droneMgr.disconnect(appId);
        if (droneMgr.getConnectedAppsCount() == 0) {
            Timber.d("Destroying drone manager.");
            droneMgr.destroy();
            droneManagers.remove(droneMgr.getConnectionParameter());
        }
    }

    /**
     * Setup a MAVLink connection using the given parameter.
     *
     * @param connParams  Parameter used to setup the MAVLink connection.
     * @param listenerTag Used to identify the connection requester.
     * @param listener    Callback to receive the connection events.
     */
    void connectMAVConnection(ConnectionParameter connParams, String listenerTag, MavLinkConnectionListener listener) {
        AndroidMavLinkConnection conn = mavConnections.get(connParams.getUniqueId());
        final int connectionType = connParams.getConnectionType();
        final Bundle paramsBundle = connParams.getParamsBundle();
        if (conn == null) {

            //Create a new mavlink connection

            switch (connectionType) {
                case ConnectionType.TYPE_USB:
                    final int baudRate = paramsBundle.getInt(ConnectionType.EXTRA_USB_BAUD_RATE,
                            ConnectionType.DEFAULT_USB_BAUD_RATE);
                    conn = new UsbConnection(getApplicationContext(), baudRate);
                    Timber.d("Connecting over usb.");
                    break;

                case ConnectionType.TYPE_BLUETOOTH:
                    //Retrieve the bluetooth address to connect to
                    final String bluetoothAddress = paramsBundle.getString(ConnectionType.EXTRA_BLUETOOTH_ADDRESS);
                    conn = new BluetoothConnection(getApplicationContext(), bluetoothAddress);
                    Timber.d("Connecting over bluetooth.");
                    break;

                case ConnectionType.TYPE_TCP:
                    //Retrieve the server ip and port
                    final String tcpServerIp = paramsBundle.getString(ConnectionType.EXTRA_TCP_SERVER_IP);
                    final int tcpServerPort = paramsBundle.getInt(ConnectionType
                            .EXTRA_TCP_SERVER_PORT, ConnectionType.DEFAULT_TCP_SERVER_PORT);
                    conn = new AndroidTcpConnection(getApplicationContext(), tcpServerIp, tcpServerPort);
                    Timber.d("Connecting over tcp.");
                    break;

                case ConnectionType.TYPE_UDP:
                    final int udpServerPort = paramsBundle
                            .getInt(ConnectionType.EXTRA_UDP_SERVER_PORT, ConnectionType.DEFAULT_UDP_SERVER_PORT);
                    conn = new AndroidUdpConnection(getApplicationContext(), udpServerPort);
                    Timber.d("Connecting over udp.");
                    break;

                default:
                    Timber.e("Unrecognized connection type: %s", connectionType);
                    return;
            }

            mavConnections.put(connParams.getUniqueId(), conn);
        }

        if (connectionType == ConnectionType.TYPE_UDP) {
            final String pingIpAddress = paramsBundle.getString(ConnectionType.EXTRA_UDP_PING_RECEIVER_IP);
            if (!TextUtils.isEmpty(pingIpAddress)) {
                try {
                    final InetAddress resolvedAddress = InetAddress.getByName(pingIpAddress);

                    final int pingPort = paramsBundle.getInt(ConnectionType.EXTRA_UDP_PING_RECEIVER_PORT);
                    final long pingPeriod = paramsBundle.getLong(ConnectionType.EXTRA_UDP_PING_PERIOD,
                            ConnectionType.DEFAULT_UDP_PING_PERIOD);
                    final byte[] pingPayload = paramsBundle.getByteArray(ConnectionType.EXTRA_UDP_PING_PAYLOAD);

                    ((AndroidUdpConnection) conn).addPingTarget(resolvedAddress, pingPort, pingPeriod, pingPayload);

                } catch (UnknownHostException e) {
                    Timber.e(e, "Unable to resolve UDP ping server ip address.");
                }
            }
        }

        conn.addMavLinkConnectionListener(listenerTag, listener);
        if (conn.getConnectionStatus() == MavLinkConnection.MAVLINK_DISCONNECTED) {
            conn.connect();

            // Record which connection type is used.
            GAUtils.sendEvent(new HitBuilders.EventBuilder()
                    .setCategory(GAUtils.Category.MAVLINK_CONNECTION)
                    .setAction("MavLink connect")
                    .setLabel(connParams.toString()));
        }
    }

    /**
     * Disconnect the MAVLink connection for the given listener.
     *
     * @param connParams  Connection parameters
     * @param listenerTag Listener to be disconnected.
     */
    void disconnectMAVConnection(ConnectionParameter connParams, String listenerTag) {
        final AndroidMavLinkConnection conn = mavConnections.get(connParams.getUniqueId());
        if (conn == null)
            return;

        conn.removeMavLinkConnectionListener(listenerTag);

        if (conn.getMavLinkConnectionListenersCount() == 0 && conn.getConnectionStatus() !=
                MavLinkConnection.MAVLINK_DISCONNECTED) {
            Timber.d("Disconnecting...");
            conn.disconnect();

            GAUtils.sendEvent(new HitBuilders.EventBuilder()
                    .setCategory(GAUtils.Category.MAVLINK_CONNECTION)
                    .setAction("MavLink disconnect")
                    .setLabel(connParams.toString()));
        }
    }

    /**
     * Register a log listener.
     *
     * @param connParams      Parameters whose connection's data to log.
     * @param tag             Tag for the listener.
     * @param loggingFilePath File path for the logging file.
     */
    void addLoggingFile(ConnectionParameter connParams, String tag, String loggingFilePath) {
        AndroidMavLinkConnection conn = mavConnections.get(connParams.getUniqueId());
        if (conn == null)
            return;

        conn.addLoggingPath(tag, loggingFilePath);
    }

    /**
     * Unregister a log listener.
     *
     * @param connParams Connection parameters from whom to stop the logging.
     * @param tag        Tag for the listener.
     */
    void removeLoggingFile(ConnectionParameter connParams, String tag) {
        AndroidMavLinkConnection conn = mavConnections.get(connParams.getUniqueId());
        if (conn == null)
            return;

        conn.removeLoggingPath(tag);
    }

    /**
     * Retrieves the set of camera info provided by the app.
     *
     * @return a list of {@link CameraDetail} objects.
     */
    synchronized List<CameraDetail> getCameraDetails() {
        if (cachedCameraDetails == null) {
            List<String> cameraInfoNames = cameraInfoLoader.getCameraInfoList();

            List<CameraInfo> cameraInfos = new ArrayList<>(cameraInfoNames.size());
            for (String infoName : cameraInfoNames) {
                try {
                    cameraInfos.add(cameraInfoLoader.openFile(infoName));
                } catch (Exception e) {
                    Timber.e(e, e.getMessage());
                }
            }

            List<CameraDetail> cameraDetails = new ArrayList<>(cameraInfos.size());
            for (CameraInfo camInfo : cameraInfos) {
                cameraDetails.add(new CameraDetail(camInfo.name, camInfo.sensorWidth,
                        camInfo.sensorHeight, camInfo.sensorResolution, camInfo.focalLength,
                        camInfo.overlap, camInfo.sidelap, camInfo.isInLandscapeOrientation));
            }

            cachedCameraDetails = cameraDetails;
        }

        return cachedCameraDetails;
    }

    @Override
    public IBinder onBind(Intent intent) {
        Timber.d("Binding intent: " + intent);
        final String action = intent.getAction();
        if (IDroidPlannerServices.class.getName().equals(action)) {
            // Return binder to ipc client-server interaction.
            return dpServices;
        } else {
            // Return binder to the service.
            return droneAccess;
        }
    }

    @SuppressLint("NewApi")
    @Override
    public void onCreate() {
        super.onCreate();
        Timber.d("Creating 3DR Services.");

        context = getApplicationContext();

        mavlinkApi = new MavLinkServiceApi(this);
        droneAccess = new DroneAccess(this);
        dpServices = new DPServices(this);
        lbm = LocalBroadcastManager.getInstance(context);
        this.cameraInfoLoader = new CameraInfoLoader(context);
        updateForegroundNotification();
        Log.e(TAG, "DP service started");

        context.registerReceiver(gcsInitAttLockReceiver, gcsInitAttLockFilter);

        mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        if (hasGCSAccel() && hasGCSGyro()) {

            mSensorManager.registerListener(this, mAccelerometer,
                    SensorManager.SENSOR_DELAY_UI);

            aLastBcast = System.currentTimeMillis();

            mSensorManager.registerListener(this, mGyroscope,
                    SensorManager.SENSOR_DELAY_UI);

            gLastBcast = System.currentTimeMillis();

            broadcastGCSAttData();
        }

    }

    void gcsInitAttLock() {
        quaternionFreeze.set(quaternionCurrent);
    }

    // Process new reading
    @Override
    public void onSensorChanged(SensorEvent event) {

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            if ((etimestamp !=0)) {
                synchronized (syncToken) {
                    accelVector.setXYZ(event.values);
                }
            }

        }

        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
//			handle gyro reading
            gyroVector.setXYZ(event.values);

            if ((etimestamp != 0)) {
                final float dT = (event.timestamp - etimestamp) * NS2S;
                // Axis of the rotation sample, not normalized yet.
                float axisX = gyroVector.getX();
                float axisY = gyroVector.getY();
                float axisZ = gyroVector.getZ();

                // Calculate the angular speed of the sample
                float omegaMagnitude = sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);

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


            /////////////////////////////////////////////////////////
            //Calculate Euler Angle, using Quat//////////////////////
            /////////////////////////////////////////////////////////
            //R b to e is recursive, R=Rz*Ry*Rx, see ref2. page 22
                synchronized (syncToken) {
                    quaternionCurrent.multiplyQuat(deltaQuaternion, quaternionCurrent);
                    quaternionCurrent.toCorrectAxisAngleByGravity(accelVector, axisAngle);
                    axisAngle.toQuaternion(correctQuat);
                    quaternionCurrent.multiplyQuat(correctQuat, quaternionCurrent);
                    quaternionCurrent.toEulerAngles(eulerQuatCurrent);
                    quaternionFreeze.getDiffQuaternionToTarget(quaternionCurrent).toEulerAngles(diffEulerQuat);
                    quaternionFreeze.toEulerAngles(eulerQuatFreeze);
                }
            }
            etimestamp = event.timestamp;
        }

        broadcastGCSAttData();

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // N/A
    }

    public Attitude getGCSAttitude() {
        return gcsAtt;
    }

    public Attitude getGCSAttitudeLocked() {
        return gcsAttLocked;
    }

    public Vector3 getGCSGyro() {
        return gcsGyro;
    }

    public Vector3 getGCSAccel() {
        return gcsAccel;
    }

    public boolean hasGCSGyro() {
        return (null != (mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)));
    }

    public boolean hasGCSAccel() {
        return (null != (mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)));
    }

    @SuppressLint("NewApi")
    private void updateForegroundNotification() {
        final Context context = getApplicationContext();

        //Put the service in the foreground
        final Notification.Builder notifBuilder = new Notification.Builder(context)
                .setContentTitle("3DR Services")
                .setSmallIcon(R.drawable.ic_stat_notify)
                .setContentIntent(PendingIntent.getActivity(context, 0, new Intent(context,
                        MainActivity.class).addFlags(Intent.FLAG_ACTIVITY_NEW_TASK), 0));

        final int connectedCount = droneApiStore.size();
        if (connectedCount > 0) {
            if (connectedCount == 1) {
                notifBuilder.setContentText("1 connected app");
            } else {
                notifBuilder.setContentText(connectedCount + " connected apps");
            }
        }

        final Notification notification = Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN
                ? notifBuilder.build()
                : notifBuilder.getNotification();
        startForeground(FOREGROUND_ID, notification);
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        Timber.d("Destroying 3DR Services.");

        for (DroneApi droneApi : droneApiStore.values()) {
            droneApi.destroy();
        }
        droneApiStore.clear();

        for (AndroidMavLinkConnection conn : mavConnections.values()) {
            conn.disconnect();
            conn.removeAllMavLinkConnectionListeners();
        }

        mavConnections.clear();
        dpServices.destroy();
        mSensorManager.unregisterListener(this);
        context.unregisterReceiver(gcsInitAttLockReceiver);
        stopForeground(true);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        if (intent != null) {
            final String action = intent.getAction();
            switch (action) {
                case ACTION_KICK_START_DRONESHARE_UPLOADS:
                    for (DroneManager droneMgr : droneManagers.values()) {
                        droneMgr.kickStartDroneShareUpload();
                    }
                    break;

                case ACTION_RELEASE_API_INSTANCE:
                    final String appId = intent.getStringExtra(EXTRA_API_INSTANCE_APP_ID);
                    releaseDroneApi(appId);
                    break;
            }
        }

        if (hasGCSAccel()&&hasGCSGyro()) {
            mSensorManager.registerListener(this, mAccelerometer,
                    SensorManager.SENSOR_DELAY_UI);

            aLastBcast = System.currentTimeMillis();

            mSensorManager.registerListener(this, mGyroscope,
                    SensorManager.SENSOR_DELAY_UI);

            gLastBcast = System.currentTimeMillis();


            broadcastGCSAttData();
        }
        stopSelf();
        return START_NOT_STICKY;
    }

    private void broadcastGCSAttData() {
        gcsAttLocked.setYaw((double) eulerQuatFreeze.getX());
        gcsAttLocked.setPitch((double) eulerQuatFreeze.getY());
        gcsAttLocked.setRoll((double) eulerQuatFreeze.getZ());
        long actualTime = System.currentTimeMillis();

        if ((actualTime-aLastBcast)>BCAST_INTERVAL) {
            gcsAccel.setX((double) accelVector.getX());
            gcsAccel.setY((double) accelVector.getY());
            gcsAccel.setZ((double) accelVector.getZ());
            lbm.sendBroadcast(new Intent(ACTION_GCS_ACCEL_UPDATED));
            aLastBcast = actualTime;
        }
        if ((actualTime-gLastBcast)>BCAST_INTERVAL) {
            gcsGyro.setX((double) gyroVector.getX());
            gcsGyro.setY((double) gyroVector.getY());
            gcsGyro.setZ((double) gyroVector.getZ());
            lbm.sendBroadcast(new Intent(ACTION_GCS_GYRO_UPDATED));

            gcsAtt.setYaw((double) diffEulerQuat.getX());
            gcsAtt.setPitch((double) diffEulerQuat.getY());
            gcsAtt.setRoll((double) diffEulerQuat.getZ());
            //formulate the intent
//          Intent gcsAttIntent = new Intent(ACTION_GCS_ATTITUDE_UPDATED);
//          gcsAttIntent.putExtra(ACTION_GCS_ATTITUDE_UPDATED, gcsAtt);
//          lbm.sendBroadcast(gcsAttIntent);
            lbm.sendBroadcast(new Intent(ACTION_GCS_ATTITUDE_UPDATED));
            context.sendBroadcast(new Intent(ACTION_GCS_ATTITUDE_UPDATED));
//          Log.e(TAG, "msg sent");
//          Log.e(TAG,ACTION_GCS_ATTITUDE_UPDATED);
//          Log.e(TAG, gcsAtt.toString());
            gLastBcast = actualTime;
        }
    }

}
