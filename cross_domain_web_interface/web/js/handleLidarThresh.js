/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Point and Click Interface Development
 * Since: June 22nd, 2021
 */

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
        alert('I am a Stub!');
    }
}
