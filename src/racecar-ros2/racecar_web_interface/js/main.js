// Define some global variables
var rosbridgeServer = null;
var velocityCmdTopic = null;

// Calls added inside the anynonymous function are triggered after the page is loaded
$(document).ready(() => {document.getElementById("log").value = "Default text\n"});

// rosbridge / roslibjs function to connect to ROS
function connectROS()
{
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

// belm2371
function sendAvancez() {
    var twist = new ROSLIB.Message(
        {linear : {x : 0.5, y : 0.0, z : 0.0}, angular : {x : 0.0, y : 0.0, z : 0.0}});
    
    velocityCmdTopic.publish(twist);

    document.getElementById("textStatus").value += "Avancez\n";
    
}

function sendArretez() {
    var twist = new ROSLIB.Message(
        {linear : {x : 0.0, y : 0.0, z : 0.0}, angular : {x : 0.0, y : 0.0, z : 0.0}});
    
    velocityCmdTopic.publish(twist);

    document.getElementById("textStatus").value += "Arrêtez\n";
    
}


// =========================
// Boutons
// =========================

    // Bouton Avance
    document.getElementById("btnAvancez").onclick = function () {
        //sendAvancez();
        document.getElementById("textStatus").value += "Avancez\n";
    };

    // Bouton Arrête
    document.getElementById("btnArretez").onclick = function () {
        //sendArretez();
        document.getElementById("textStatus").value += "Arrêtez\n";
    };

    // Bouton Effacez
    document.getElementById("btnClearHistory").onclick = function () {
        document.getElementById("textStatus").value = "";
    };

// Bouton Connectez!
const btnConnect = document.getElementById("btnConnect");
const textStatus = document.getElementById("textStatus");

if (btnConnect) {
    btnConnect.addEventListener("click", () => {

        // Lire l'IP entrée par l'utilisateur
        rosMasterIp = document.getElementById("ipInput").value.trim();

        if (rosMasterIp === "") {
            textStatus.value = "Erreur : aucune adresse IP fournie.\n";
            return;
        }

        // Fermer une ancienne connexion si elle existe
        if (rosbridgeServer !== null) {
            rosbridgeServer.close();
        }

        // Appeler TA fonction de connexion ROS
        connectROS();

        // Ajouter les handlers pour afficher le statut dans le textarea
        rosbridgeServer.on("connection", () => {
            textStatus.value += "Connecté au Racecar!\n";
        });

        rosbridgeServer.on("error", () => {
            textStatus.value += "Erreur de connexion avec le Racecar!\n";
        });

        rosbridgeServer.on("close", () => {
            textStatus.value += "Connexion fermée.\n";
        });
    });
}