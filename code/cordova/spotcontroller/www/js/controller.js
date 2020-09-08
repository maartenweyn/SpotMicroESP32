var controller = {
    useSonar: true,
    rotationRanges: {
        omega: 30,
        phi: 30,
        psi: 30,
        xm: 20,
        ym: 20,
        zm: 20,
    },
    initialize: function () {
        debug.log("UUID " + device.uuid, "success");

        setInterval(controller.timer_callback, 100);
    },
    sendButton: function (button) { 
        var buttonvalue = button.charCodeAt(0);
        debug.log("Button " + button + " " + buttonvalue, "success");
        var data = new Uint8Array(2);
        data[0] = 1; // control command
        data[1] = buttonvalue; 
        bluetooth.sendData(data.buffer);
    },
    toggleSonar: function() {
        controller.useSonar = !controller.useSonar;
        debug.log("useSonar " + controller.useSonar, "success");

        var data = new Uint8Array(3);
        data[0] = 2; // options command
        data[1] = 1;  // sonar 
        data[2] = controller.useSonar; 
        bluetooth.sendData(data.buffer);

        if (controller.useSonar) {
            document.querySelector('#icon_sonar').setAttribute('icon', 'md-eye');
        } else {
            document.querySelector('#icon_sonar').setAttribute('icon', 'md-eye-off');
        }
    },
    timer_callback: function() {
        var Joy1X = Joy1.GetPosX();
        var Joy1Y = Joy1.GetPosY();
        var Joy2X = Joy2.GetPosX();
        var Joy2Y = Joy2.GetPosY();
        // console.log("Joy1 " + Joy1X + " " + Joy1Y);
        // console.log("Joy2 " + Joy2X + " " + Joy2Y);

        omega = (controller.rotationRanges.omega * (Joy1X - 100) / 50).toFixed(0)
        phi = (controller.rotationRanges.phi * (Joy1Y - 100) / 50).toFixed(0);
        psi = 0;
        zm = (controller.rotationRanges.xm * (Joy2X - 100) / 50).toFixed(0);
        xm = (controller.rotationRanges.xm * (Joy2Y - 100) / 50).toFixed(0);
        ym = 0;

        // console.log("Rotation " + omega + " " + phi + " " + psi + " " + xm + " " + ym + " " + zm);
        bluetooth.sendOrientation(omega, phi, psi, xm, ym, zm);
    },
}