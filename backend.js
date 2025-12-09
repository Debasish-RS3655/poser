// pose imitator new backend
// Debashish Buragohain
var express = require('express');
var app = express();
var bodyParser = require('body-parser');
const path = require('path');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const COMport = 5;
const Serial = new SerialPort(
  { path: `COM${COMport}`, baudRate: 9600 },
  function (err) {
    if (err) {
      console.error('The body of the agent is not connected: ', err);
      return null;
    }
  }
);

const parser = Serial.pipe(new ReadlineParser({ delimiter: '\r\n' }));
parser.on('data', function (data) {
  console.log('Received from COM5: ', data);
});

parser.on('error', err => console.log('Error in Serial port: ', err));

app.use(function (req, res, next) {
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, PUT, PATCH, DELETE');
    res.setHeader('Access-Control-Allow-Headers', 'X-Requested-With,content-type');
    res.setHeader('Access-Control-Allow-Credentials', false);
    next();
});
app.use(bodyParser.json({ limit: '50MB' }));
app.use(function (err, req, res, next) {
    console.error("Backend Error: Error in backend main server: ", err.stack)
    res.status(500).send('Something broke in the backend server RIO!')
    next();
});
app.use('/file', express.static(path.join(__dirname)));

if (Serial !== null) {
    console.log(`Connected to arduino at COM ${COMport}.`);
    app.post('/poseImitatorBackend', function (req, res) {
        console.log('Degrees from frontend: ', req.body.data);
        //because of limitations, we send only 8 reduced servo degrees instead of 13
        //thus ignoring the headServoVertical, left ear, right ear, left elbow and right elbow
        var poseDegrees = [];
        for (var i = 0; i < req.body.data.length; i++) if (i !== 1 && i < 9) poseDegrees.push(req.body.data[i]);
        console.log("Degrees to be sent to Arduino: ", poseDegrees)
        //we reduce the servo degrees since the Arduino is unable to handle all of them
        for (var i = 0; i < poseDegrees.length; i++) poseDegrees[i] = Math.floor(poseDegrees[i] / 10);
        console.log("Reduced degrees: ", poseDegrees);
        var sendData = "";
        //because of RF length limitations, we send only 11 servo degrees
        for (var i = 0; i < poseDegrees.length; i++) {
            if (poseDegrees[i] >= 10) sendData += String(poseDegrees[i]);
            else sendData += String(0) + String(poseDegrees[i]);           //we make each degree of two figures
        }
        Serial.write(sendData);
        console.log("Data sent to arduino: ", sendData);
        res.json({ received: true })
    })
}
app.listen(4040, () => console.log("Backend server listening to port 4040."));