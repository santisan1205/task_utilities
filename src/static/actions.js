// Old-school "ES5" JavaScript compatible with ancient browsers
function runAction(actionName) {
    var statusDiv = document.getElementById("status-msg");
    statusDiv.innerHTML = "⏳ Sending " + actionName + "...";

    // Use XMLHttpRequest instead of 'fetch'
    var xhr = new XMLHttpRequest();
    xhr.open("POST", "/run/" + actionName, true);
    
    xhr.onreadystatechange = function() {
        if (xhr.readyState === 4) { // 4 means request finished
            if (xhr.status === 200) {
                statusDiv.innerHTML = xhr.responseText;
            } else {
                statusDiv.innerHTML = "❌ Error: " + xhr.status;
            }
        }
    };
    
    xhr.onerror = function() {
        statusDiv.innerHTML = "❌ Network Error";
    };

    xhr.send();
}