/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: June 8th, 2021
 * Purpose: This javascript function prompts the user 
 * 	    for a new ngrok url for the image stream served
 * 	    by ros_web_video server. The new stream information 
 * 	    is used to change the various image sources throughout
 * 	    the interface
 */

function setIFrameStream()
{
	// see if base URL is not already set from stream
	if(sessionStorage.getItem("streamURL") == null)
	{
		var StreamURL = prompt("please enter ngrok image stream URL");
		sessionStorage.setItem("streamURL", StreamURL);
	}
	
	// if it is, set iframe source accordingly
	else
	{
		var StreamURL = sessionStorage.getItem("streamURL");
		var IFrameStreamURL = StreamURL + "/stream_viewer?topic=/camera/color/image_raw&type=ros_compressed";
		document.getElementById('stream').src = IFrameStreamURL;
	}
}

function setImgStream()
{
        // see if base URL is not already set for stream
        if(sessionStorage.getItem("streamURL") == null)
        {
                var StreamURL = prompt("please enter ngrok image stream URL");
                sessionStorage.setItem("streamURL", StreamURL);
        }

	// set src image accordingly if base URL is already set
        else
        {
                var StreamURL = sessionStorage.getItem("streamURL");
                var ImgStreamURL = StreamURL + "/stream?topic=/camera/color/image_raw&type=ros_compressed";
		document.getElementById("imgStream").src = ImgStreamURL;
        }
}

