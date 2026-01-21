package org.firstinspires.ftc.teamcode.utils;

import purejavahidapi.*;

public class DualSenseLightbar {

    // Sony DualSense USB Vendor & Product IDs
    private static final int VENDOR_ID  = 0x054C;
    private static final int PRODUCT_ID = 0x0CE6;

    /**
     * Set the DualSense lightbar colour (USB only, not Bluetooth)
     * @param r Red (0–255)
     * @param g Green (0–255)
     * @param b Blue (0–255)
     */
    public static void setColor(int r, int g, int b) {
        HidDevice device = null;
        try {
            HidDeviceInfo deviceInfo = null;
            for (HidDeviceInfo info : PureJavaHidApi.enumerateDevices()) {
                if (info.getVendorId() == VENDOR_ID && info.getProductId() == PRODUCT_ID) {
                    deviceInfo = info;
                    break;
                }
            }

            if (deviceInfo == null) {
                System.err.println("DualSense not found. Plug it in via USB.");
                return;
            }

            device = PureJavaHidApi.openDevice(deviceInfo);
            if (device == null) {
                System.err.println("Failed to open DualSense HID device.");
                return;
            }

            // Build USB output report (Report ID 0x02)
            byte[] report = new byte[32];
            report[0] = 0x02; // report ID
            // Lightbar RGB values (common offsets)
            report[6] = (byte) r;
            report[7] = (byte) g;
            report[8] = (byte) b;

            // Send the report
            device.setOutputReport(report[0], report, report.length);

            System.out.printf("Lightbar set to RGB(%d, %d, %d)%n", r, g, b);
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            if (device != null) device.close();
        }
    }
}
