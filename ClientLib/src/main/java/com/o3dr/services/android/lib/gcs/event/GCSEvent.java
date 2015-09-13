package com.o3dr.services.android.lib.gcs.event;

/**
 * Stores the list of gcs events (as action), and their extra parameters.
 * The defined events are used in system broadcasts.
 */
public class GCSEvent {

    private static final String PACKAGE_NAME = "com.o3dr.services.android.lib.gcs.event";

    /**
     * Key to retrieve the app id for the client that caused the event.
     */
    public static final String EXTRA_APP_ID = PACKAGE_NAME + ".extra.APP_ID";

    /**
     * Broadcast action: a connection with a vehicle was established.
     */
    public static final String ACTION_VEHICLE_CONNECTION = PACKAGE_NAME + ".action.VEHICLE_CONNECTION";

    /**
     * Key to retrieve the parameter for the connection.
     */
    public static final String EXTRA_VEHICLE_CONNECTION_PARAMETER = PACKAGE_NAME + ".extra" +
            ".VEHICLE_CONNECTION_PARAMETER";

    /**
     * Broadcast action: the connection with the vehicle was broken.
     */
    public static final String ACTION_VEHICLE_DISCONNECTION = PACKAGE_NAME + ".action.VEHICLE_DISCONNECTION";

    /**
     * GCS acceleration vector attribute events.
     */
    public static final String GCS_ACCEL_UPDATED = PACKAGE_NAME + ".GCS_ACCEL_UPDATED";

    /**
     * GCS gyro vector attribute events.
     */
    public static final String GCS_GYRO_UPDATED = PACKAGE_NAME + ".GCS_GYRO_UPDATED";

    /**
     * GCS Attitude attribute events.
     */
    public static final String GCS_ATTITUDE_UPDATED = PACKAGE_NAME + ".GCS_ATTITUDE_UPDATED";

    /**
     * GCS Initial Attitude lock events.
     */
    public static final String GCS_INIT_ATTITUDE_LOCKED = PACKAGE_NAME + ".GCS_INIT_ATTITUDE_LOCKED";


    //Not instantiable
    private GCSEvent(){}
}
