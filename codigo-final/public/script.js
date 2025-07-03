const keysPressed = new Set();
let speed = 0;
let interval = null;
const maxSpeed = 255;
const minSpeed = 0;
const acceleration = 10;
const timeRate = 100; //ms
let currentDirection = null;

function sendMotors(a, b, c, d) {
  fetch(`/setMotors?motorA=${a}&motorB=${b}&motorC=${c}&motorD=${d}`);
}

document.addEventListener("keydown", (e) => {
  const key = e.key.toUpperCase();
  if (!keysPressed.has(key) && ['W', 'A', 'S', 'D'].includes(key)) {
    keysPressed.add(key);
    updateMovement();
  }
});

document.addEventListener("keyup", (e) => {
  keysPressed.delete(e.key.toUpperCase());
  if (keysPressed.size === 0) stop();
});




function updateMovement() {
  if (interval) clearInterval(interval);
  speed = 0;
  interval = setInterval(() => {
    speed += acceleration;
    speed = constrain(speed, minSpeed, maxSpeed);

    const w = keysPressed.has('W');
    const a = keysPressed.has('A');
    const s = keysPressed.has('S');
    const d = keysPressed.has('D');

    // Combinaciones diagonales o simples
    let aVal = 0, bVal = 0, cVal = 0, dVal = 0;

    if (w && !s) { aVal = bVal = cVal = dVal = speed; }
    if (s && !w) { aVal = bVal = cVal = dVal = -speed; }

    if (a && !d) { aVal = cVal = -speed; bVal = dVal = speed; }  // Gira izq
    if (d && !a) { aVal = cVal = speed;  bVal = dVal = -speed; } // Gira der

    sendMotors(
      constrain(aVal, -255, 255),
      constrain(bVal, -255, 255),
      constrain(cVal, -255, 255),
      constrain(dVal, -255, 255)
    );
  }, timeRate);
}

function stop() {
  if (interval) clearInterval(interval);
  sendMotors(0, 0, 0, 0);
  speed = 0;
  currentDirection = null;
}

function constrain(val, min, max) {
  return Math.max(min, Math.min(max, val));
}


function actualizarEstadoMotores() {
  fetch('/getMotors')
    .then(response => response.json())
    .then(data => {
      document.getElementById('motorA').textContent = data.motorA;
      document.getElementById('motorB').textContent = data.motorB;
      document.getElementById('motorC').textContent = data.motorC;
      document.getElementById('motorD').textContent = data.motorD;
    })
    .catch(err => console.error("Error al obtener estado de motores:", err));
}

setInterval(actualizarEstadoMotores, 500);



