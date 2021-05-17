/* Author: Trevor Sherrard
 * Course: Human Robot Interaction
 * Project: Autonomous Camera Control Optimization
 * Since: November 1st, 2020
 */

function openFunction()
{
	openURL = "http://robotcontrol.live:5000/setGripperState"
	openData = {"gripperStateData": 1};
	$.ajax({type: 'POST',
		url: openURL,
		data: JSON.stringify (openData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

function closeFunction()
{
        closeURL = "http://robotcontrol.live:5000/setGripperState"
        closeData = {"gripperStateData": 0};
        $.ajax({type: 'POST',
                url: closeURL,
                data: JSON.stringify (closeData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}


