package frc.robot.logging;

import frc.robot.Constants.TelemetryConstants;

public abstract class LogEntry<T> {
    private static double lastCheckedTimestamp = 0;

    private T lastItem;

    public LogEntry() {
        if (!TelemetryConstants.kLoggingEnabled) {
            return;
        }

        checkNetwork(false);
        checkDatalog();

        lastItem = null;
    }

    protected abstract void enableDatalog();
    protected abstract void enableNetwork();

    protected abstract boolean isDatalogEnabled();
    protected abstract boolean isNetworkEnabled();

    protected abstract void sendData(T data);
    
    public void update(T data) {
        if (!TelemetryConstants.kLoggingEnabled) {
            return;
        }

        if (!isDatalogEnabled()) {
            checkDatalog();
        }

        if (!isNetworkEnabled()) {
            checkNetwork(true);
        }

        if (data != null && data != lastItem) {
            sendData(data);
            lastItem = data;
        }
    }

    private void checkNetwork(boolean checkTimestamp) {
        if (TelemetryConstants.kDisableNetworkLog) {
            return;
        }

        long timestamp = System.currentTimeMillis();
        if (checkTimestamp && timestamp - lastCheckedTimestamp < TelemetryConstants.kFMSCheckDelayMillis) {
            return;
        }

        lastCheckedTimestamp = timestamp;
        enableNetwork();
    }

    private void checkDatalog() {
        if (TelemetryConstants.kDisableDataLog) {
            return;
        }

        enableDatalog();
    }
}
