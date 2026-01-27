function runTrigger(triggerName) {
    var statusDiv = document.getElementById("status-msg");
    statusDiv.innerHTML = "⏳ Processing " + triggerName + "...";

    var xhr = new XMLHttpRequest();
    // Notice we point to /trigger/ now
    xhr.open("POST", "/trigger/" + triggerName, true);
    
    xhr.onreadystatechange = function() {
        if (xhr.readyState === 4) {
            if (xhr.status === 200) {
                statusDiv.innerHTML = xhr.responseText;
            } else {
                statusDiv.innerHTML = "❌ Error";
            }
        }
    };
    xhr.send();
}