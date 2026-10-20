// On ouvre les trucs en memoire de la page de login
let IpSauvegarde = sessionStorage.getItem('racecar_ip');
let NomSauvegarde = sessionStorage.getItem('racecar_user');
let avance = 0;
document.getElementById('rosMasterIp').value = IpSauvegarde;
document.getElementById('utilisateurracecar').innerText = NomSauvegarde;
 
// Define some global variables
var rosbridgeServer = null;
var velocityCmdTopic = null;
 
 
const boiteStatut = document.getElementById('boiteStatut');
const btnEffacer = document.getElementById('btnEffacer');
const btnAvancez = document.getElementById('btnAvancez');
const btnArretez = document.getElementById('btnArretez');
const btnConnect = document.getElementById('btnConnect');
const deconnectionbtn = document.getElementById('deconnectionbtn');
const connectionracecarstate = document.getElementById('connectionracecarstate');
 
btnAvancez.disabled = true;
btnArretez.disabled = true;
 
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
    avance = 1;
});
 
btnArretez.addEventListener('click', function() {
    ajouterStatut('Arrêt !');
    avance = 0;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    
});
 
deconnectionbtn.addEventListener('click', function(){
 
    deconnection();
});
 
connectROS(); //directement executer la fonction de connexion
 
// Calls added inside the anynonymous function are triggered after the page is loaded
//$(document).ready(() => {document.getElementById("log").value = "Default text\n"});
 
//Creation du joystick
console.log("Initialisation du joystick...");
ajouterStatut("Initialisation du joystick...")
let joystick;
const zone = document.getElementById('zoneJoystick');
 
//if (btnAvancez.disabled == false) {
 
    joystick = new VirtualJoystick({
        container: zone,
        mouseSupport: true,
        limitStickTravel: true,
        stickRadius: 50,
        // stationaryBase: true,
        // baseX: zone.offsetWidth / 2,
        // baseY: 100 //
    });
 
//}
 
 
 
 
// rosbridge / roslibjs function to connect to ROS
function connectROS()
{
    var rosMasterIp = document.getElementById('rosMasterIp').value;
    // Connect to the rosbridge server running on localhost, on port 9090
    // HINT: The rosbridge server SHOULD be closed when disconnecting from ROS
    rosbridgeServer = new ROSLIB.Ros({url : "ws://" + rosMasterIp + ":9090"});
   
    rosbridgeServer.on("connection", () => {
        console.log("Connected to WebSocket server.");
        ajouterStatut("Connecté au Racecar");
        btnAvancez.disabled = false;
        btnArretez.disabled = false;
        connectionracecarstate.innerText = "Connecté au racecar";
        connectionracecarstate.style.color = "green";
        // Create a topic object to send propulsion commands to the racecar
        velocityCmdTopic = new ROSLIB.Topic(
            {ros : rosbridgeServer, name : "/prop_cmd", messageType : "geometry_msgs/Twist"});
    });
 
    rosbridgeServer.on(
        "error", (error) => { console.log("Error connecting to WebSocket server: ", error);
   
        btnAvancez.disabled = true;
        btnArretez.disabled = true;
        connectionracecarstate.innerText = "Non connecté au racecar";
        connectionracecarstate.style.color = "red";
    });
 
    rosbridgeServer.on("close", () => { console.log("Closed connection to WebSocket server.");
        ajouterStatut("Connection échoué");
    });
   
 
 
}
 
// Create a message that conforms to the `Twist` structure defined in ROS.
var twist = new ROSLIB.Message(
    {linear : {x : 0.0, y : 0.0, z : 2.0}, angular : {x : 0.0, y : 0.0, z : 0.0}});
 
// Add timer callbacks here
setInterval(() => {
    if (velocityCmdTopic != null) {
       
       if (joystick.right() || joystick.left() || joystick.up() || joystick.down()) {
        twist.linear.x = -joystick.deltaY() / 50.0;
        twist.angular.z = -joystick.deltaX() / 50.0;
       }else if (avance ==0){
        twist.linear.x=0.0
        twist.angular.z=0.0
       }

       else if (avance ==1) {
        twist.linear.x = 0.5;
       }
       
       
        velocityCmdTopic.publish(twist);
    }
}, 200);
 
 
function deconnection(){
   
    if (rosbridgeServer != null) {
        rosbridgeServer.close();
    }
   
    sessionStorage.clear();
    window.location.href = 'login.html'; //rediriger
}
 