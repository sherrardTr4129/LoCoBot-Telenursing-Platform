/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: June 7th, 2021
 * Purpose: handle click events in image stream canvas, 
 * and send data to backend when click event occurs
 */

// set up data prototypes for sending P&C locations
var imagePointURL = "http://robotcontrol.live:5000/sendCamSelection";
var imagePointData = {
    "imgX":0,
    "imgY":0
};

function sendPointData(x, y)
{
    // construct data packet
    imagePointData.imgX = x;
    imagePointData.imgY = y;

    // make POST request
    $.ajax({type: 'POST',
	url: imagePointURL,
	data: JSON.stringify (imagePointData),
	success: function(data) {
		if(data.success == true){
			setTimeout(function() { alert('selected point is on floor! Moving to location...'); }, 1);
		}
		else{
			alert('selected point is not safe! Please try again!');
		}

	},
	contentType: "application/json",
	dataType: 'json'
    });
}

$(document).ready(function() {
    $("img").on("click", function(event) {
	// extract click location
        var x = event.pageX - this.offsetLeft;
        var y = event.pageY - this.offsetTop;

	// send data to backend
        console.log("X Coordinate: " + x + " Y Coordinate: " + y);
	sendPointData(x, y);
    });
});

