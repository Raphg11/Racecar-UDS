// Define some global variables
var rosbridgeServer = null;
var velocityCmdTopic = null;


const boiteStatut = document.getElementById('boiteStatut');
const btnEffacer = document.getElementById('btnEffacer');
const btnAvancez = document.getElementById('btnAvancez');
const btnArretez = document.getElementById('btnArretez');
const btnConnect = document.getElementById('btnConnect');


function ajouterStatut(texte) {

    boiteStatut.value += texte + '\n';
    boiteStatut.scrollTop = boiteStatut.scrollHeight;
}

btnConnect.addEventListener('click',function() {
    connectROS();

});
btnEffacer.addEventListener('click', function() {
    boiteStatut.value = ''; // Vide la boîte de texte
});

btnAvancez.addEventListener('click', function() {
    ajouterStatut('En avant !');
    // Plus tard, c'est ici qu'on enverra le message Twist via ROS (Activité 4)
});

btnArretez.addEventListener('click', function() {
    ajouterStatut('Arrêt !');
    // Plus tard, c'est ici qu'on arrêtera le robot via ROS (Activité 4)
});



// Calls added inside the anynonymous function are triggered after the page is loaded
//$(document).ready(() => {document.getElementById("log").value = "Default text\n"});

// rosbridge / roslibjs function to connect to ROS
function connectROS()
{
    var rosMasterIp = document.getElementById('rosMasterIp').value;
    // Connect to the rosbridge server running on localhost, on port 9090
    // HINT: The rosbridge server SHOULD be closed when disconnecting from ROS
    rosbridgeServer = new ROSLIB.Ros({url : "ws://" + rosMasterIp + ":9090"});

    rosbridgeServer.on("connection", () => {
        console.log("Connected to WebSocket server.");

        // Create a topic object to send propulsion commands to the racecar
        velocityCmdTopic = new ROSLIB.Topic(
            {ros : rosbridgeServer, name : "/prop_cmd", messageType : "geometry_msgs/Twist"});
    });

    rosbridgeServer.on(
        "error", (error) => { console.log("Error connecting to WebSocket server: ", error); });

    rosbridgeServer.on("close", () => { console.log("Closed connection to WebSocket server."); });
}

// Create a message that conforms to the `Twist` structure defined in ROS.
var twist = new ROSLIB.Message(
    {linear : {x : 0.0, y : 0.0, z : 2.0}, angular : {x : 0.0, y : 0.0, z : 0.0}});

// Add timer callbacks here
setInterval(() => {
    if (velocityCmdTopic != null) {
        // HINT: The `twist` object's values SHOULD be updated BEFORE publishing the message.
        // Publish the message
        velocityCmdTopic.publish(twist);
    }
}, 200);
