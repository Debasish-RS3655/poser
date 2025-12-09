//Debashish Buragohain
//Pose imitator robot
const webcamElement1 = document.getElementById("webcam")
const intervalDelay = 2500;
var canvas = document.getElementById('postureOutput');
var ctx = canvas.getContext('2d');
ctx.lineWidth = "3";
ctx.strokeStyle = "blue";
async function main() {
    const webcam = await tf.data.webcam(webcamElement1);
    console.log("Loading PoseNet.")
    const ModWidth = parseInt(document.getElementsByTagName('VIDEO')[0].getAttribute('width'));
    const ModHeight = parseInt(document.getElementsByTagName('VIDEO')[0].getAttribute('height'));
    const pose_net = await posenet.load({
        architecture: 'ResNet50', //MobileNetV1
        outputStride: 32,//32
        inputResolution: { width: ModWidth, height: ModHeight },
        //multiplier: 0.75
        quantBytes: 2
    });
    console.log("PoseNet loaded.");
    const sleep = ms => new Promise(req => setTimeout(req, ms));
    while (true) {
        const img = await webcam.capture();
        const pose = await pose_net.estimateSinglePose(img, {
            flipHorizontal: false
        });
        if (pose !== undefined) {
            drawPoseDiagram(pose);                          //time to draw the pose diagram
            let servoDegrees = analyze_poseDetection(pose)
            document.getElementById('pose_detection').innerHTML = `
            head horizontal servo: ${servoDegrees[0]} <br>
            right shoulder vertical servo: ${servoDegrees[2]} <br>
            left shoulder vertical servo: ${servoDegrees[3]} <br>
            right shoulder horizontal servo: ${servoDegrees[4]} <br>
            left shoulder horizontal servo: ${servoDegrees[5]} <br>
            right elbow servo: ${servoDegrees[6]} <br>
            left elbow servo: ${servoDegrees[7]} <br>
            base servo: ${servoDegrees[8]} <br>
            `;
            //send the analyzed pose detections to the backend now
            await fetch('http://127.0.0.1:4040/poseImitatorBackend', {
                headers: {
                    'Accept': 'application/json',
                    'Content-Type': 'application/json'
                },
                method: "POST",
                body: JSON.stringify({ data: servoDegrees })
            })
                .catch(err => {
                    console.log("Error sending servo pose degrees to the backend: ", err);
                });
        }
        img.dispose();
        await sleep(intervalDelay);
    }
}

main();
//Pose detection analysis
//order of the parts in the array------
//nose                  //leftHip
//leftEye               //rightHip
//rightEye              //leftKnee
//leftEar               //rightKnee
//rightEar              //leftAnkle
//leftShoulder          //rightAnkle
//rightShoulder         
//leftElbow
//rightElbow
//leftWrist
//rightWrist
//pose detection tensorflow variables as it has been transferred to frontend
//this function takes in the pose detection response and converts it into appropriate servo degrees
let RightUpperArmMaxLength;
let LeftUpperArmMaxLength;
let RightLowerArmMaxLength;
let LeftLowerArmMaxLength;
let maxHorHeadLength;
let maxHorBodyLength;
let maxBodyDist;

function analyze_poseDetection(positions) {
    var detectedPositionsX = new Array(0);
    var detectedPositionsY = new Array(0);
    for (var x = 0; x < positions.keypoints.length; x++) {
        detectedPositionsX[x] = positions.keypoints[x].position.x;
        detectedPositionsY[x] = positions.keypoints[x].position.y;
    }
    //here we have three arrays containing info about the details
    //code to extract the distances from the poses detected

    //get the maximum distance of the body (in the vertical direction)
    //midpoint of left shoulder and right shoulder
    var upperMidpointX = round((detectedPositionsX[5] + detectedPositionsX[6]) / 2);
    var upperMidpointY = round((detectedPositionsY[5] + detectedPositionsY[6]) / 2);
    //midpoint of the left ankle and right ankle
    var lowerMidpointX = round((detectedPositionsX[15] + detectedPositionsX[16]) / 2);
    var lowerMidpointY = round((detectedPositionsY[15] + detectedPositionsY[16]) / 2);
    //get the cartesian distance of the body (However the scale here is not constant and might hamper)
    var thismaxBodyDist = Math.sqrt(pow(upperMidpointX - lowerMidpointX, 2) + pow(upperMidpointY - lowerMidpointY, 2));
    if (maxBodyDist == undefined || thismaxBodyDist > maxBodyDist)
        maxBodyDist = thismaxBodyDist;
    //from this body distance we might be able to get the size of the body and then scale our degrees based on this size
    //get the degrees my measuring the shoulder and elbow length
    //Cartesian distance between the elbow and shoulder servos of respective arms
    var leftShoulder_leftElbow = Math.sqrt(pow(detectedPositionsX[5] - detectedPositionsX[7], 2) + pow(detectedPositionsY[5] - detectedPositionsY[7], 2));
    var rightShoulder_rightElbow = Math.sqrt(pow(detectedPositionsX[6] - detectedPositionsX[8], 2) + pow(detectedPositionsY[6] - detectedPositionsY[8], 2));
    //from this we are now getting the actual servo degrees
    var leftShoulderYServoUpperArm;
    var rightShoulderYServoUpperArm;

    //make sure we don't exceed boundaries
    if (LeftUpperArmMaxLength == undefined || leftShoulder_leftElbow > LeftUpperArmMaxLength) {
        //the arm length cannot exceed the vertical body distance
        if (leftShoulder_leftElbow > maxBodyDist) {
            LeftUpperArmMaxLength = maxBodyDist;
            leftShoulder_leftElbow = LeftUpperArmMaxLength;
        }
        else
            LeftUpperArmMaxLength = leftShoulder_leftElbow;
    }
    if (RightUpperArmMaxLength == undefined || rightShoulder_rightElbow > RightUpperArmMaxLength) {
        if (rightShoulder_rightElbow > maxBodyDist) {
            RightUpperArmMaxLength = maxBodyDist;
            rightShoulder_rightElbow = RightUpperArmMaxLength;
        }
        else
            RightUpperArmMaxLength = rightShoulder_rightElbow;
    }

    //here we get the actual servo degrees for the shoulder servos
    if (detectedPositionsY[5] - detectedPositionsY[7] < 0) { //left elbow is higher than the left shoulder
        //left hand is moving up, left shoulder servo also will be moving up
        leftShoulderYServoUpperArm = map(leftShoulder_leftElbow, 0, LeftUpperArmMaxLength, 90, 180);
    }
    else {
        //left hand is moving down, left shoulder servo also will be moving down
        leftShoulderYServoUpperArm = map(leftShoulder_leftElbow, 0, LeftUpperArmMaxLength, 90, 0);
    }

    if (detectedPositionsY[6] - detectedPositionsY[8] < 0) { //right elbow is higher than the right shoulder
        //right hand is moving up
        rightShoulderYServoUpperArm = map(rightShoulder_rightElbow, 0, RightUpperArmMaxLength, 90, 180);
    }
    else {
        //hand is moving down
        rightShoulderYServoUpperArm = map(rightShoulder_rightElbow, 0, RightUpperArmMaxLength, 90, 0);
    }

    //get the degrees by measuring the length of the elbow and the wrist
    let leftLowerArmlength = Math.sqrt(pow(detectedPositionsX[7] - detectedPositionsX[9], 2) + pow(detectedPositionsY[7] - detectedPositionsY[9], 2));
    let rightLowerArmlength = Math.sqrt(pow(detectedPositionsX[8] - detectedPositionsX[10], 2) + pow(detectedPositionsY[8] - detectedPositionsY[10], 2));
    //no boundaries exceed
    if (LeftLowerArmMaxLength == undefined || LeftLowerArmMaxLength < leftLowerArmlength) {
        if (leftLowerArmlength > maxBodyDist) {
            LeftLowerArmMaxLength = maxBodyDist;
            leftLowerArmlength = LeftLowerArmMaxLength;
        }
        else
            LeftLowerArmMaxLength = leftLowerArmlength;
    }
    if (RightLowerArmMaxLength == undefined || RightLowerArmMaxLength < rightLowerArmlength) {
        if (rightLowerArmlength > maxBodyDist) {
            RightLowerArmMaxLength = maxBodyDist;
            rightLowerArmlength = RightLowerArmMaxLength;
        }
        else
            RightLowerArmMaxLength = rightLowerArmlength;
    }
    //the output servo degrees
    var leftShoulderYServo = round(leftShoulderYServoUpperArm);
    var rightShoulderYServo = round(rightShoulderYServoUpperArm);

    var leftShoulderYServoLowerArm;
    var rightShoulderYServoLowerArm;
    //for the left arm
    if (detectedPositionsY[7] - detectedPositionsY[9] < 0) {
        //lower arm is moving up, lower arm servo will also move up
        leftShoulderYServoLowerArm = round(map(leftLowerArmlength, 0, LeftLowerArmMaxLength, 90, 180));
    }
    else {
        //lower arm moving down, lower arm servo will also move down
        leftShoulderYServoLowerArm = round(map(leftLowerArmlength, 0, LeftLowerArmMaxLength, 90, 0));
    }

    //the right arm
    if (detectedPositionsY[8] - detectedPositionsY[10] < 0) {
        //hand is moving up
        rightShoulderYServoLowerArm = round(map(rightLowerArmlength, 0, RightLowerArmMaxLength, 90, 180));
    }
    else {
        //hand is moving down
        rightShoulderYServoLowerArm = round(map(rightLowerArmlength, 0, RightLowerArmMaxLength, 90, 0));
    }
    //still haven't included the elbow servo degrees in the output
    var leftElbowServo = round(leftShoulderYServoLowerArm);
    var rightElbowServo = round(rightShoulderYServoLowerArm);
    //--------------------------------Horizontal shoulder servo-------------------------------------
    var posDegreesLeft = round(Math.abs(angleWithYaxis(detectedPositionsX[5], detectedPositionsY[5], detectedPositionsX[7], detectedPositionsY[7])));
    var posDegreesRight = round(Math.abs(angleWithYaxis(detectedPositionsX[6], detectedPositionsY[6], detectedPositionsX[8], detectedPositionsY[8])));
    //horizontal servo degrees
    //right hand servo segrees are opposite of the left hand servo degrees
    var leftShoulderXServo = posDegreesLeft;
    var rightShoulderXServo = posDegreesRight;

    //--------------------------------Wrist Servo----------------------------------------------------
    let leftWristDegrees = round(Math.abs(angleWithYaxis(detectedPositionsX[7], detectedPositionsY[7], detectedPositionsX[9], detectedPositionsY[9])));
    let rightWristDegrees = round(Math.abs(angleWithYaxis(detectedPositionsX[8], detectedPositionsY[8], detectedPositionsX[10], detectedPositionsY[10])));
    //in the current prototype version of the program, we are not using the stabilizer although it is a very useful algorithm
    //elbow servo degrees
    var leftWristServo = leftWristDegrees;
    var rightWristServo = rightWristDegrees;

    //----------------------------body movement including all---------------------------------------------------
    var leftShoulder_leftHip = Math.sqrt(pow(detectedPositionsX[5] - detectedPositionsX[11], 2) + pow(detectedPositionsY[5] - detectedPositionsY[11], 2));
    var rightShoulder_rightHip = Math.sqrt(pow(detectedPositionsX[6] - detectedPositionsX[12], 2) + pow(detectedPositionsY[6] - detectedPositionsY[12], 2));
    var leftShoulder_rightShoulder = Math.sqrt(pow(detectedPositionsX[5] - detectedPositionsX[6], 2) + pow(detectedPositionsY[5] - detectedPositionsY[6], 2));
    //we then determine the side the human is looking based upon perspective (the farther side has the smaller distance)
    if (maxHorBodyLength == undefined || leftShoulder_rightShoulder > maxHorBodyLength) {
        if (leftShoulder_rightShoulder > maxBodyDist) {
            maxHorBodyLength = maxBodyDist;
            leftShoulder_rightShoulder = maxHorBodyLength;
        }
        else
            maxHorBodyLength = leftShoulder_rightShoulder;
    }
    let baseServoDegrees;
    if (leftShoulder_leftHip > rightShoulder_rightHip)
        baseServoDegrees = round(map(leftShoulder_rightShoulder, maxHorBodyLength, 0, 90, 0));    //human is looking right
    else baseServoDegrees = round(map(leftShoulder_rightShoulder, maxHorBodyLength, 0, 90, 180)); //human is looking left

    //--------------------------head movement only------------------------------------------------
    var leftEar_Nose = Math.sqrt(pow(detectedPositionsX[0] - detectedPositionsX[3], 2) + pow(detectedPositionsY[0] - detectedPositionsY[3], 2));
    var rightEar_Nose = Math.sqrt(pow(detectedPositionsX[0] - detectedPositionsX[4], 2) + pow(detectedPositionsY[0] - detectedPositionsY[4], 2));
    var leftEar_rightEar = Math.sqrt(pow(detectedPositionsX[3] - detectedPositionsX[4], 2) + pow(detectedPositionsY[3] - detectedPositionsY[4], 2));
    if (maxHorHeadLength == undefined || maxHorHeadLength < leftEar_rightEar) {
        if (leftEar_rightEar > maxBodyDist) {
            maxHorHeadLength = maxBodyDist;
            leftEar_rightEar = maxHorHeadLength;
        }
        else
            maxHorHeadLength = leftEar_rightEar;
    }
    let headServoHorDegree;
    if (leftEar_Nose > rightEar_Nose) {
        //human is looking right
        headServoHorDegree = round(map(leftEar_rightEar, maxHorHeadLength, 0, 90, 0));
    }
    else {
        //human is looking left
        headServoHorDegree = round(map(leftEar_rightEar, maxHorHeadLength, 0, 90, 180));
    }

    return [headServoHorDegree, 90, rightShoulderYServo, leftShoulderYServo, rightShoulderXServo, leftShoulderXServo,
        rightWristServo, leftWristServo, baseServoDegrees, 90, 90, rightElbowServo, leftElbowServo];
    //return the poses with respect to the connections and the series in the Arduino Code
    //head horizontal, head vertical, right shoulder vertical ,left shoulder vertical ,right shoulder horizontal
    //left shoulder horizontal, right elbow, left elbow, base ,right ear,left ear,right elbow, left elbow
    //function to map the data to the given range
    function angleWithYaxis(x1, y1, x2, y2) {
        //calculates the angle in non-negative degreees of line joining two points with the Y axis
        const m1 = (y2 - y1) / (x2 - x1);
        const m2 = 0;                                       //for the x axis
        let tangent = Math.abs((m1 - m2) / (1 + m1 * m2));
        let atan = (Math.atan(tangent) * 180 / 3.14) + 90;  //positive degree measure [0 to 180 degrees]
        return (atan);
    }
    function map(x, in_min, in_max, out_min, out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    //we round off the given function, I know i am being stupid doing it this way
    function round(el) {
        return Math.round(el)
    }
    function pow(el, power) {
        return Math.pow(el, power)
    }
}

async function drawPoseDiagram(positions) {
    //extract the positions now
    var detectedPositionsX = new Array(0);
    var detectedPositionsY = new Array(0);
    for (var x = 0; x < positions.keypoints.length; x++) {
        detectedPositionsX[x] = positions.keypoints[x].position.x;
        detectedPositionsY[x] = positions.keypoints[x].position.y;
    }
    //order of the parts in the array------
    //0 nose                  //11 leftHip
    //1 leftEye               //12 rightHip
    //2 rightEye              //13 leftKnee
    //3 leftEar               //14 rightKnee
    //4 rightEar              //15 leftAnkle
    //5 leftShoulder          //16 rightAnkle
    //6 rightShoulder         
    //7 leftElbow
    //8 rightElbow
    //9 leftWrist
    //10 rightWrist
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    //drawing all the points
    for (var i = 0; i < detectedPositionsX.length; i++) {
        ctx.fillRect(detectedPositionsX[i], detectedPositionsY[i], 5, 5)
    }
    //finally drawing out the poitions
    //between right shoulder and left shoulder
    ctx.moveTo(detectedPositionsX[5], detectedPositionsY[5]);
    ctx.lineTo(detectedPositionsX[6], detectedPositionsY[6]);
    //right shoulder and right elbow
    ctx.moveTo(detectedPositionsX[6], detectedPositionsY[6]);
    ctx.lineTo(detectedPositionsX[8], detectedPositionsY[8]);
    //right elbow and right wrist
    ctx.moveTo(detectedPositionsX[8], detectedPositionsY[8]);
    ctx.lineTo(detectedPositionsX[10], detectedPositionsY[10]);
    //left shoulder and left elbow
    ctx.moveTo(detectedPositionsX[5], detectedPositionsY[5]);
    ctx.lineTo(detectedPositionsX[7], detectedPositionsY[7]);
    //left elbow and left wrist
    ctx.moveTo(detectedPositionsX[7], detectedPositionsY[7]);
    ctx.lineTo(detectedPositionsX[9], detectedPositionsY[9]);
    //right shoulder and right hip
    ctx.moveTo(detectedPositionsX[6], detectedPositionsY[6]);
    ctx.lineTo(detectedPositionsX[12], detectedPositionsY[12]);
    //left shoulder and left hip
    ctx.moveTo(detectedPositionsX[5], detectedPositionsY[5]);
    ctx.lineTo(detectedPositionsX[11], detectedPositionsY[11]);
    //left hip and right hip
    ctx.moveTo(detectedPositionsX[11], detectedPositionsY[11]);
    ctx.lineTo(detectedPositionsX[12], detectedPositionsY[12]);
    //right hip and right knee
    ctx.moveTo(detectedPositionsX[12], detectedPositionsY[12]);
    ctx.lineTo(detectedPositionsX[14], detectedPositionsY[14]);
    //left hip and left knee
    ctx.moveTo(detectedPositionsX[11], detectedPositionsY[11]);
    ctx.lineTo(detectedPositionsX[13], detectedPositionsY[13]);
    //right knee and right ankle
    ctx.moveTo(detectedPositionsX[14], detectedPositionsY[14]);
    ctx.lineTo(detectedPositionsX[16], detectedPositionsY[16]);
    //left knee and left ankle
    ctx.moveTo(detectedPositionsX[13], detectedPositionsY[13]);
    ctx.lineTo(detectedPositionsX[15], detectedPositionsY[15]);
    //nose and neck joint
    const neckJointX = (detectedPositionsX[5] + detectedPositionsX[6]) / 2;
    const neckJointY = (detectedPositionsY[5] + detectedPositionsY[6]) / 2;
    ctx.moveTo(detectedPositionsX[0], detectedPositionsY[0]);
    ctx.lineTo(neckJointX, neckJointY);
    ctx.stroke();
}