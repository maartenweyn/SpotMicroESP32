var app = {
    initialize: function() {
        this.bindEvents();
    },
    bindEvents: function() {
        document.addEventListener('deviceready', this.onDeviceReady, false);
    },
    onDeviceReady: function() {
        //navigator.geolocation.getCurrentPosition (function () {}, function () {});
        var onSuccess = function(position) {
            console.log('Latitude: '          + position.coords.latitude          + '\n' +
                  'Longitude: '         + position.coords.longitude         + '\n' +
                  'Altitude: '          + position.coords.altitude          + '\n' +
                  'Accuracy: '          + position.coords.accuracy          + '\n' +
                  'Altitude Accuracy: ' + position.coords.altitudeAccuracy  + '\n' +
                  'Heading: '           + position.coords.heading           + '\n' +
                  'Speed: '             + position.coords.speed             + '\n' +
                  'Timestamp: '         + position.timestamp                + '\n');
        };
    
        // onError Callback receives a PositionError object
        //
        function onError(error) {
            console.log('code: '    + error.code    + '\n' +
                  'message: ' + error.message + '\n');
        }
    
        // HACK to enable BLE in android 10 with the current pluggin
        navigator.geolocation.getCurrentPosition(function () {}, onError);
        app.scan();
    },
    scan: function() {
        app.status("Scanning for Heart Rate Monitor");

        var foundHeartRateMonitor = false;

        function onScan(peripheral) {
            console.log(JSON.stringify(peripheral));

        }

        function scanFailure(reason) {
            alert("BLE Scan Failed");
        }
              ble.startScan(['00FF'], onScan, scanFailure);

    },
    status: function(message) {
        console.log(message);    
    },

};

app.initialize();