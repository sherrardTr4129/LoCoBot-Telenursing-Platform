/* Author: Trevor Sherrard
 * Course: Human Robot Interaction
 * Project: Autonomous Camera Control Optimization
 * Since: November 1st, 2020
 */

var Joy1 = new JoyStick('joy1');
var xURL = "http://robotcontrol.live:5000/setArmOffsetX";
var yURL = "http://robotcontrol.live:5000/setArmOffsetY";

var xData = {
	"xArmOffset": 0
};
var yData = {
        "yArmOffset": 0
};

function sendXData()
{
	$.ajax({type: 'POST',
		url: xURL,
		data: JSON.stringify (xData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

function sendYData()
{
        $.ajax({type: 'POST',
                url: yURL,
                data: JSON.stringify (yData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}


setInterval(function(){ xData.xArmOffset=Joy1.GetY(); }, 300);
setInterval(function(){ yData.yArmOffset=Joy1.GetX(); }, 300);
setInterval(function(){ sendXData() }, 500);
setInterval(function(){ sendYData() }, 500);


var Joy2 = new JoyStick('joy2');
var zURL = "http://robotcontrol.live:5000/setArmOffsetZ"
var zData = {
	"zArmOffset": 0
}

function sendZData()
{
	$.ajax({type: 'POST',
		url: zURL,
		data: JSON.stringify (zData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

setInterval(function(){ zData.zArmOffset=Joy2.GetY(); }, 300);
setInterval(function(){ sendZData() }, 500);

var Joy3 = new JoyStick('joy3');
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

setInterval(function(){ spinData.spin = Joy3.GetX(); }, 300);
setInterval(function(){ fwdRevData.fwdRev = Joy3.GetY(); }, 300);
setInterval(function(){ sendFwdRevData() }, 500);
setInterval(function(){ sendSpinData() }, 500);



