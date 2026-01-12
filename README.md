# pc-screen-to-hub75
Real-time Screen Mirroring to HUB75 led matrix (esp32s3)

ESP32-S3와 HUB75 LED 매트릭스를 활용하여 PC 화면을 실시간으로 스트리밍하는 프로젝트입니다.

MAC OS 
board : HUIDU HD-WF2 (ESP32S3)
matrix : p2 hub75 128x64 ( DP3246 + SM5368 )

https://youtu.be/-X4493Losbk?si=YjUdqubvIYlr2KFF
## 📺 Demo Video
[![Watch the Demo](https://img.youtube.com/vi/YOUR_VIDEO_ID/maxresdefault.jpg)](https://www.youtube.com/watch?v=YjUdqubvIYlr2KFF)

모니터 화면을 hub75 led matrix 로 스트리밍 하는 예제 입니다. 
파이썬 프로그램은 맥OS 에서만 테스트 되었습니다. 

정확한 이유는 아직 모르겠는데 ESP32 코어 버전이 3.3.3 에서 잘 동작 합니다. 현재 최신 버전(3.3.5) 에서 프레임이 떨어집니다. 


