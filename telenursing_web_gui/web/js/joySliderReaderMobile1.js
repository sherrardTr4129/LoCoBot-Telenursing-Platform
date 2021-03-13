/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: January 26th, 2021
 */

// read fwdRev data joystick and send data to flask app on 200 ms interval
var Joy1 = new JoyStick('joy1');
var frURL = "http://localhost:5000/fwdRevJoyPost";
var frData = {
	"FwdRev": 0,
};

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

setInterval(function(){ frData.FwdRev=Joy1.GetY(); }, 50);
setInterval(function(){ sendFRData() }, 200);


var Slider1 = new JoyStickSlider('slider1');
var sliderURL = "http://localhost:5000/sliderPost"
var sliderData = {
	"slider": 0
}

function sendSliderData()
{
	$.ajax({type: 'POST',
		url: sliderURL,
		data: JSON.stringify (sliderData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}
setInterval(function(){ sliderData.slider=Slider1.GetX(); }, 50);
setInterval(function(){ sendSliderData() }, 200);

