@echo off
title Pose Imitiator
start "C:\Program Files\Google\Chrome\Application\chrome.exe" http://127.0.0.1:4040/file/poseImitatorFrontend.html
node backend.js
exit