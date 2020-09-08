var app = {
    user: {},
    debug: false,
    joystick : {
        joy1X: null,
        joy1Y: null,
        joy2X : null,
        joy2y: null,
        joy1: null,
        joy2: null,
    },
    background_timer_settings: {
        timerInterval: 1000, // interval between ticks of the timer in milliseconds (Default: 60000)
        startOnBoot: false, // enable this to start timer after the device was restarted (Default: false)
        stopOnTerminate: true, // set to true to force stop timer in case the app is terminated (User closed the app and etc.) (Default: true)
        hours: -1, // delay timer to start at certain time (Default: -1)
        minutes: -1, // delay timer to start at certain time (Default: -1)
    },
    initialize: function () {
        console.log('app initialize');

    },

    onDeviceReady: function () {
        debug.log('device ready', 'success');
        app.bindEvents();
        bluetooth.initialize();
        controller.initialize();

        app.joystick.joy1 = Joy1;
        app.joystick.joy2 = Joy2;
        app.joystick.joy1X = document.getElementById("joy1X");
        app.joystick.joy1Y = document.getElementById("joy1Y");
        app.joystick.joy2X = document.getElementById("joy2X");
        app.joystick.joy2Y = document.getElementById("joy2Y");

        // setInterval(app.getJoyStickValues, 50);

        window.BackgroundTimer.onTimerEvent(app.getJoyStickValues);
        window.BackgroundTimer.start(app.timerstart_successCallback, app.timerstart_errorCallback, app.background_timer_settings);

    },

    bindEvents: function () {
        // setTimeout(function () {
        //     mqttclient.addMessage('app,1');
        // }, 3000);

        document.addEventListener("pause", app.onDevicePause, false);
        document.addEventListener("resume", app.onDeviceResume, false);
        document.addEventListener("menubutton", app.onMenuKeyDown, false);
    },

    timerstart_successCallback: function() {
        debug.log("BLE: timer started", 'success');
    },
    timerstart_errorCallback: function(e) {
        debug.log("BLE error: could not start timer", 'error');
    },
    timerstop_successCallback: function() {
        debug.log("BLE: timer stopped", 'success');
    },
    timerstop_errorCallback: function(e) {
        debug.log("BLE error: could not stop timer", 'error');
    },

    onDevicePause: function () {
        debug.log('in pause');
    },
    onDeviceResume: function () {
        debug.log('out of pause');
        bluetooth.refreshDeviceList();
    },
    onMenuKeyDown: function () {
        debug.log('menubuttonpressed');
    },
    onError: function (error) {
        debug.log(JSON.stringify(error), 'error');
    },
    getJoyStickValues: function() {
        // app.joystick.joy1X.value=app.joystick.joy1.GetPosX();
        // app.joystick.joy1Y.value=app.joystick.joy1.GetPosY(); 
        // app.joystick.joy2X.value=app.joystick.joy2.GetPosX();
        // app.joystick.joy2Y.value=app.joystick.joy2.GetPosY(); 

        // var joy1x = document.getElementById("joy1X");
        // joy1x.value=Joy1.GetPosX();

        console.log("Joy " + Joy1.GetPosX());
    }
};

app.initialize();