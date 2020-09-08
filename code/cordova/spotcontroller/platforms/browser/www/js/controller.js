var controller = {
    useSonar: true,
    initialize: function () {
        debug.log("UUID " + device.uuid, "success");
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
    }
}