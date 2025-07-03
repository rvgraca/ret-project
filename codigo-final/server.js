const express = require('express');
const path = require('path');
const app = express();
const port = 3000;

// Middleware para servir frontend
app.use(express.static(path.join(__dirname, 'public')));

const parseOrDefault = v => {
    const parsed = parseInt(v);
    return isNaN(parsed) ? 0 : parsed;
};



// Endpoint desde ESP32 o desde UI
let motors = { motorA: 0, motorB: 0, motorC: 0, motorD: 0 };
app.get('/setMotors', (req, res) => {
    motors.motorA = parseOrDefault(req.query.motorA);
    motors.motorB = parseOrDefault(req.query.motorB);
    motors.motorC = parseOrDefault(req.query.motorC);
    motors.motorD = parseOrDefault(req.query.motorD);
    console.log(`MotorA: ${motors.motorA}, MotorB: ${motors.motorB}, MotorC: ${motors.motorC}, MotorD: ${motors.motorD}`);
    res.sendStatus(200);
});
app.get('/getMotors', (req, res) => {
    res.json(motors);
});

app.listen(port, () => console.log(`Servidor en http://localhost:${port}`));