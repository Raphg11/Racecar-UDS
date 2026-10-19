const usernamevalue = "Racecar11"
const passwordvalue = "Raphestbo";
 
 
let connected = false;
 
function verifyconnection()
{
    let username = document.getElementById('Username').value;
    let password = document.getElementById('Password').value;
    let ip = document.getElementById('IpAdress').value;
    if (username == usernamevalue) {
        if (password == passwordvalue){
            document.getElementById('connectionstate').innerText = "Connecté";
 
            //stocker les datas
            sessionStorage.setItem('racecar_user', username);
            sessionStorage.setItem('racecar_ip', ip);
 
            window.location.href = 'index.html'; //rediriger
            return true;
        }
        document.getElementById('connectionstate').innerText = "Erreur d'identifiant";
        return false;
    }
    document.getElementById('connectionstate').innerText = "Erreur d'identifiant";
    return false;
 
}
 
 
const btnconnexion = document.getElementById('btnconnection');
 
 
btnconnexion.addEventListener('click', function(){
    connected = verifyconnection();
   
})