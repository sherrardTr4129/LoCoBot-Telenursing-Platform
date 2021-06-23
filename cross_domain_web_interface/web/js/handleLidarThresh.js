/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Point and Click Interface Development
 * Since: June 22nd, 2021
 */

var setThreshURL = "http://robotcontrol.live:5000/sendThreshVal";
var threshData = {
    "close":0,
    "very_close":0
};

function sendTreshData(close_thresh, very_close_thresh)
{
    // construct data packet
    threshData.close = close_thresh;
    threshData.very_close = very_close_thresh;

    // make POST request
    $.ajax({type: 'POST',
	url: setThreshURL,
	data: JSON.stringify (threshData),
	success: function(data) {console.log('data');},
	contentType: "application/json",
	dataType: 'json'
    });
}

function get_thresh_vals(){
    // get input references
    var close_thresh_input = document.getElementById('close_thresh');
    var very_close_thresh_input = document.getElementById('very_close_thresh');

    // grab values
    var close_thresh_value = close_thresh_input.value;
    var very_close_thresh_val = very_close_thresh_input.value;

    // do basic error checking
    if(very_close_thresh_val > close_thresh_value){
        alert('very close thresh must be less than close thresh!'); 
    }
    
    // if input looks good, go ahead and make request to backend to update values
    else{
        sendTreshData(close_thresh_value, very_close_thresh_val);
	console.log("set close_thresh_value: " + close_thresh_value + " ," + very_close_thresh_val);
    }
}
