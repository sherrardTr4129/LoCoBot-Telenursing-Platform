/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: June 7th, 2021
 */

// read fwdRev data joystick and send data to flask app on 200 ms interval
var Joy4 = new JoyStick('joy4');
var frURL = "http://localhost:5000/fwdRevJoyPost";
var frData = {
	"FwdRev": 0,
};

// read spin data joystick and send data to flask app on 200 ms interval
var spinURL = "http://localhost:5000/spinJoyPost"
var spinData = {
	"spin": 0
}

function sendFRData()
{
	$.ajax({type: 'POST',
		url: frURL,
		data: JSON.stringify (frData),
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

setInterval(function(){ frData.FwdRev=Joy4.GetY(); }, 50);
setInterval(function(){ sendFRData() }, 200);
setInterval(function(){ spinData.spin=Joy4.GetX(); }, 50);
setInterval(function(){ sendSpinData() }, 200);

