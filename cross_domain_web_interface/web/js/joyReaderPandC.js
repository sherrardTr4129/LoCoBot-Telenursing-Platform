/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: June 7th, 2021
 */

// read fwdRev data joystick and send data to flask app on 200 ms interval
var Joy4 = new JoyStick('joy4');

var fwdRevURL = "http://robotcontrol.live:5000/setFwdRev";
var spinURL = "http://robotcontrol.live:5000/setSpin";
var fwdRevData = {
        "fwdRev": 0
}

var spinData = {
        "spin": 0
}

function sendFwdRevData()
{
        $.ajax({type: 'POST',
                url: fwdRevURL,
                data: JSON.stringify (fwdRevData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

function sendSpinData()
{
        $.ajax({type: 'POST',
                url: spinURL,
                data: JSON.stringify (spinData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

setInterval(function(){ spinData.spin = Joy4.GetX(); }, 300);
setInterval(function(){ fwdRevData.fwdRev = Joy4.GetY(); }, 300);
setInterval(function(){ sendFwdRevData() }, 500);
setInterval(function(){ sendSpinData() }, 500);
