// Pose imitator upload-based frontend
const fileInput = document.getElementById('imageUpload');
const imgEl = document.getElementById('uploadedImage');
const canvas = document.getElementById('postureOutput');
const ctx = canvas.getContext('2d');
ctx.lineWidth = "3";
ctx.strokeStyle = "blue";
const statusEl = document.getElementById('uploadStatus');

async function init() {
    console.log('Loading PoseNet...');
    const ModWidth = canvas.width;
    const ModHeight = canvas.height;
    const pose_net = await posenet.load({
        architecture: 'ResNet50',
        outputStride: 32,
        inputResolution: { width: ModWidth, height: ModHeight },
        quantBytes: 2
    });
    console.log('PoseNet loaded.');

    fileInput.addEventListener('change', async (e) => {
        const file = e.target.files[0];
        if (!file) return;
        try {
            statusEl.textContent = 'Uploading...';
            const form = new FormData();
            form.append('image', file);
            const resp = await fetch('http://127.0.0.1:4040/uploadImage', {
                method: 'POST',
                body: form
            });
            const json = await resp.json();
            if (!json.url) throw new Error('Upload failed');
            statusEl.textContent = 'Upload complete. Rendering...';
            imgEl.src = json.url;
            await new Promise(resolve => { imgEl.onload = resolve; });

            // Draw image into canvas scaled to canvas size
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            // maintain aspect ratio
            const scale = Math.min(canvas.width / imgEl.naturalWidth, canvas.height / imgEl.naturalHeight);
            const dw = imgEl.naturalWidth * scale;
            const dh = imgEl.naturalHeight * scale;
            const dx = (canvas.width - dw) / 2;
            const dy = (canvas.height - dh) / 2;
            ctx.drawImage(imgEl, dx, dy, dw, dh);

            // Estimate pose using tensor from the image element
            const imgTensor = tf.browser.fromPixels(imgEl);
            const resized = tf.image.resizeBilinear(imgTensor, [canvas.height, canvas.width]);
            const pose = await pose_net.estimateSinglePose(resized, { flipHorizontal: false });
            imgTensor.dispose();
            resized.dispose();

            if (pose) {
                drawPoseDiagram(pose, dx, dy, dw, dh);
                const servoDegrees = analyze_poseDetection(pose);
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

                // send servo degrees to backend (same as webcam version)
                await fetch('http://127.0.0.1:4040/poseImitatorBackend', {
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json'
                    },
                    method: 'POST',
                    body: JSON.stringify({ data: servoDegrees })
                }).catch(err => console.log('Error sending servo degrees:', err));
            }
            statusEl.textContent = 'Done.';
        } catch (err) {
            console.error('Upload/processing error', err);
            statusEl.textContent = 'Error: ' + err.message;
        }
    });
}

// The following functions are adapted from the original poseImitator.js
function analyze_poseDetection(positions) {
    var detectedPositionsX = [];
    var detectedPositionsY = [];
    for (var x = 0; x < positions.keypoints.length; x++) {
        detectedPositionsX[x] = positions.keypoints[x].position.x;
        detectedPositionsY[x] = positions.keypoints[x].position.y;
    }
    let RightUpperArmMaxLength;
    let LeftUpperArmMaxLength;
    let RightLowerArmMaxLength;
    let LeftLowerArmMaxLength;
    let maxHorHeadLength;
    let maxHorBodyLength;
    let maxBodyDist;

    var upperMidpointX = round((detectedPositionsX[5] + detectedPositionsX[6]) / 2);
    var upperMidpointY = round((detectedPositionsY[5] + detectedPositionsY[6]) / 2);
    var lowerMidpointX = round((detectedPositionsX[15] + detectedPositionsX[16]) / 2);
    var lowerMidpointY = round((detectedPositionsY[15] + detectedPositionsY[16]) / 2);
    var thismaxBodyDist = Math.sqrt(pow(upperMidpointX - lowerMidpointX, 2) + pow(upperMidpointY - lowerMidpointY, 2));
    if (maxBodyDist == undefined || thismaxBodyDist > maxBodyDist)
        maxBodyDist = thismaxBodyDist;

    var leftShoulder_leftElbow = Math.sqrt(pow(detectedPositionsX[5] - detectedPositionsX[7], 2) + pow(detectedPositionsY[5] - detectedPositionsY[7], 2));
    var rightShoulder_rightElbow = Math.sqrt(pow(detectedPositionsX[6] - detectedPositionsX[8], 2) + pow(detectedPositionsY[6] - detectedPositionsY[8], 2));

    if (LeftUpperArmMaxLength == undefined || leftShoulder_leftElbow > LeftUpperArmMaxLength) {
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

    var leftShoulderYServoUpperArm;
    var rightShoulderYServoUpperArm;

    if (detectedPositionsY[5] - detectedPositionsY[7] < 0) {
        leftShoulderYServoUpperArm = map(leftShoulder_leftElbow, 0, LeftUpperArmMaxLength, 90, 180);
    }
    else {
        leftShoulderYServoUpperArm = map(leftShoulder_leftElbow, 0, LeftUpperArmMaxLength, 90, 0);
    }

    if (detectedPositionsY[6] - detectedPositionsY[8] < 0) {
        rightShoulderYServoUpperArm = map(rightShoulder_rightElbow, 0, RightUpperArmMaxLength, 90, 180);
    }
    else {
        rightShoulderYServoUpperArm = map(rightShoulder_rightElbow, 0, RightUpperArmMaxLength, 90, 0);
    }

    let leftLowerArmlength = Math.sqrt(pow(detectedPositionsX[7] - detectedPositionsX[9], 2) + pow(detectedPositionsY[7] - detectedPositionsY[9], 2));
    let rightLowerArmlength = Math.sqrt(pow(detectedPositionsX[8] - detectedPositionsX[10], 2) + pow(detectedPositionsY[8] - detectedPositionsY[10], 2));
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

    var leftShoulderYServo = round(leftShoulderYServoUpperArm);
    var rightShoulderYServo = round(rightShoulderYServoUpperArm);

    var leftShoulderYServoLowerArm;
    var rightShoulderYServoLowerArm;
    if (detectedPositionsY[7] - detectedPositionsY[9] < 0) {
        leftShoulderYServoLowerArm = round(map(leftLowerArmlength, 0, LeftLowerArmMaxLength, 90, 180));
    }
    else {
        leftShoulderYServoLowerArm = round(map(leftLowerArmlength, 0, LeftLowerArmMaxLength, 90, 0));
    }

    if (detectedPositionsY[8] - detectedPositionsY[10] < 0) {
        rightShoulderYServoLowerArm = round(map(rightLowerArmlength, 0, RightLowerArmMaxLength, 90, 180));
    }
    else {
        rightShoulderYServoLowerArm = round(map(rightLowerArmlength, 0, RightLowerArmMaxLength, 90, 0));
    }

    var leftElbowServo = round(leftShoulderYServoLowerArm);
    var rightElbowServo = round(rightShoulderYServoLowerArm);

    var posDegreesLeft = round(Math.abs(angleWithYaxis(detectedPositionsX[5], detectedPositionsY[5], detectedPositionsX[7], detectedPositionsY[7])));
    var posDegreesRight = round(Math.abs(angleWithYaxis(detectedPositionsX[6], detectedPositionsY[6], detectedPositionsX[8], detectedPositionsY[8])));
    var leftShoulderXServo = posDegreesLeft;
    var rightShoulderXServo = posDegreesRight;

    let leftWristDegrees = round(Math.abs(angleWithYaxis(detectedPositionsX[7], detectedPositionsY[7], detectedPositionsX[9], detectedPositionsY[9])));
    let rightWristDegrees = round(Math.abs(angleWithYaxis(detectedPositionsX[8], detectedPositionsY[8], detectedPositionsX[10], detectedPositionsY[10])));
    var leftWristServo = leftWristDegrees;
    var rightWristServo = rightWristDegrees;

    var leftShoulder_leftHip = Math.sqrt(pow(detectedPositionsX[5] - detectedPositionsX[11], 2) + pow(detectedPositionsY[5] - detectedPositionsY[11], 2));
    var rightShoulder_rightHip = Math.sqrt(pow(detectedPositionsX[6] - detectedPositionsX[12], 2) + pow(detectedPositionsY[6] - detectedPositionsY[12], 2));
    var leftShoulder_rightShoulder = Math.sqrt(pow(detectedPositionsX[5] - detectedPositionsX[6], 2) + pow(detectedPositionsY[5] - detectedPositionsY[6], 2));
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
        baseServoDegrees = round(map(leftShoulder_rightShoulder, maxHorBodyLength, 0, 90, 0));
    else baseServoDegrees = round(map(leftShoulder_rightShoulder, maxHorBodyLength, 0, 90, 180));

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
        headServoHorDegree = round(map(leftEar_rightEar, maxHorHeadLength, 0, 90, 0));
    }
    else {
        headServoHorDegree = round(map(leftEar_rightEar, maxHorHeadLength, 0, 90, 180));
    }

    return [headServoHorDegree, 90, rightShoulderYServo, leftShoulderYServo, rightShoulderXServo, leftShoulderXServo,
        rightWristServo, leftWristServo, baseServoDegrees, 90, 90, rightElbowServo, leftElbowServo];

    function angleWithYaxis(x1, y1, x2, y2) {
        const m1 = (y2 - y1) / (x2 - x1);
        const m2 = 0;
        let tangent = Math.abs((m1 - m2) / (1 + m1 * m2));
        let atan = (Math.atan(tangent) * 180 / 3.14) + 90;
        return (atan);
    }
    function map(x, in_min, in_max, out_min, out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    function round(el) { return Math.round(el) }
    function pow(el, power) { return Math.pow(el, power) }
}

function drawPoseDiagram(positions, dx = 0, dy = 0, dw = canvas.width, dh = canvas.height) {
    var detectedPositionsX = [];
    var detectedPositionsY = [];
    for (var x = 0; x < positions.keypoints.length; x++) {
        detectedPositionsX[x] = positions.keypoints[x].position.x * (dw / positions.inputSize?.width || 1) + dx;
        detectedPositionsY[x] = positions.keypoints[x].position.y * (dh / positions.inputSize?.height || 1) + dy;
    }
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    for (var i = 0; i < detectedPositionsX.length; i++) {
        ctx.fillRect(detectedPositionsX[i], detectedPositionsY[i], 5, 5);
    }
    ctx.moveTo(detectedPositionsX[5], detectedPositionsY[5]);
    ctx.lineTo(detectedPositionsX[6], detectedPositionsY[6]);
    ctx.moveTo(detectedPositionsX[6], detectedPositionsY[6]);
    ctx.lineTo(detectedPositionsX[8], detectedPositionsY[8]);
    ctx.moveTo(detectedPositionsX[8], detectedPositionsY[8]);
    ctx.lineTo(detectedPositionsX[10], detectedPositionsY[10]);
    ctx.moveTo(detectedPositionsX[5], detectedPositionsY[5]);
    ctx.lineTo(detectedPositionsX[7], detectedPositionsY[7]);
    ctx.moveTo(detectedPositionsX[7], detectedPositionsY[7]);
    ctx.lineTo(detectedPositionsX[9], detectedPositionsY[9]);
    ctx.moveTo(detectedPositionsX[6], detectedPositionsY[6]);
    ctx.lineTo(detectedPositionsX[12], detectedPositionsY[12]);
    ctx.moveTo(detectedPositionsX[5], detectedPositionsY[5]);
    ctx.lineTo(detectedPositionsX[11], detectedPositionsY[11]);
    ctx.moveTo(detectedPositionsX[11], detectedPositionsY[11]);
    ctx.lineTo(detectedPositionsX[12], detectedPositionsY[12]);
    ctx.moveTo(detectedPositionsX[12], detectedPositionsY[12]);
    ctx.lineTo(detectedPositionsX[14], detectedPositionsY[14]);
    ctx.moveTo(detectedPositionsX[11], detectedPositionsY[11]);
    ctx.lineTo(detectedPositionsX[13], detectedPositionsY[13]);
    ctx.moveTo(detectedPositionsX[14], detectedPositionsY[14]);
    ctx.lineTo(detectedPositionsX[16], detectedPositionsY[16]);
    ctx.moveTo(detectedPositionsX[13], detectedPositionsY[13]);
    ctx.lineTo(detectedPositionsX[15], detectedPositionsY[15]);
    const neckJointX = (detectedPositionsX[5] + detectedPositionsX[6]) / 2;
    const neckJointY = (detectedPositionsY[5] + detectedPositionsY[6]) / 2;
    ctx.moveTo(detectedPositionsX[0], detectedPositionsY[0]);
    ctx.lineTo(neckJointX, neckJointY);
    ctx.stroke();
}

init();
