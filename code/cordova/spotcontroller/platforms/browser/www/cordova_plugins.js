cordova.define('cordova/plugin_list', function(require, exports, module) {
module.exports = [
    {
        "file": "plugins/cordova-background-timer/www/BackgroundTimer.js",
        "id": "cordova-background-timer.BackgroundTimer",
        "pluginId": "cordova-background-timer",
        "clobbers": [
            "window.BackgroundTimer"
        ]
    },
    {
        "file": "plugins/cordova-plugin-device/www/device.js",
        "id": "cordova-plugin-device.device",
        "pluginId": "cordova-plugin-device",
        "clobbers": [
            "device"
        ]
    },
    {
        "file": "plugins/cordova-plugin-device/src/browser/DeviceProxy.js",
        "id": "cordova-plugin-device.DeviceProxy",
        "pluginId": "cordova-plugin-device",
        "runs": true
    },
    {
        "file": "plugins/cordova-plugin-ble-central/www/ble.js",
        "id": "cordova-plugin-ble-central.ble",
        "pluginId": "cordova-plugin-ble-central",
        "clobbers": [
            "ble"
        ]
    },
    {
        "file": "plugins/cordova-plugin-ble-central/src/browser/BLECentralPlugin.js",
        "id": "cordova-plugin-ble-central.BLECentralPlugin",
        "pluginId": "cordova-plugin-ble-central",
        "merges": [
            "ble"
        ]
    },
    {
        "file": "plugins/cordova-plugin-screen-orientation/www/screenorientation.js",
        "id": "cordova-plugin-screen-orientation.screenorientation",
        "pluginId": "cordova-plugin-screen-orientation",
        "clobbers": [
            "cordova.plugins.screenorientation"
        ]
    }
];
module.exports.metadata = 
// TOP OF METADATA
{
    "cordova-background-timer": "0.0.4",
    "cordova-plugin-device": "2.0.3",
    "cordova-plugin-ble-central": "1.2.4",
    "cordova-plugin-whitelist": "1.3.4",
    "cordova-plugin-geolocation": "4.0.2",
    "cordova-plugin-screen-orientation": "3.0.2"
}
// BOTTOM OF METADATA
});