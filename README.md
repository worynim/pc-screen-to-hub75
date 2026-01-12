# pc-screen-to-hub75
Real-time Screen Mirroring to HUB75 led matrix (esp32s3)

ESP32-S3와 HUB75 LED 매트릭스를 활용하여 PC 화면을 실시간으로 스트리밍하는 프로젝트입니다.

MAC OS 
board : HUIDU HD-WF2 (ESP32S3)
matrix : p2 hub75 128x64 ( DP3246 + SM5368 )

https://youtu.be/-X4493Losbk?si=YjUdqubvIYlr2KFF
<p align="center">
  <a href="https://www.youtube.com/watch?v=-X4493Losbk">
    <img src="https://img.youtube.com/vi/-X4493Losbk/maxresdefault.jpg" width="600">
  </a>
  <br>
  <b>Real-time Screen Mirroring to HUB75 LED matrix (ESP32-S3)</b>
</p>

파이썬 프로그램은 맥OS 에서만 테스트 되었습니다. 

정확한 이유는 아직 모르겠는데 ESP32 코어 버전이 3.3.3 에서 잘 동작 합니다. 현재 최신 버전(3.3.5) 에서 프레임이 떨어집니다. 

<img width="440" height="585" alt="image" src="https://github.com/user-attachments/assets/99435ef4-53d2-4a24-aa05-2ad7f36cff3e" />
arduino IDE 보드 설정을 ESP32S3 Dev Module 로 설정 합니다. 
Huidu HD-WF2로 설정 할 경우 AP정보가 저장이 되지 않는 버그가 있습니다. 



ESP32_HUB75_LED_MATRIX_PANEL_DMA_Display 라이브러리의
/src/platforms/esp32s3/esp32s3-default-pins.hpp 파일을 아래 내용으로 수정하세요. 

#pragma once
// Avoid and QSPI pins
#define R1_PIN_DEFAULT 2
#define G1_PIN_DEFAULT 6
#define B1_PIN_DEFAULT 10
#define R2_PIN_DEFAULT 3
#define G2_PIN_DEFAULT 7
#define B2_PIN_DEFAULT 11
#define A_PIN_DEFAULT  39
#define B_PIN_DEFAULT  38
#define C_PIN_DEFAULT  37
#define D_PIN_DEFAULT  -1
#define E_PIN_DEFAULT  -1 // required for 1/32 scan panels, like 64x64. Any available pin would do, i.e. IO32
#define LAT_PIN_DEFAULT 33
#define OE_PIN_DEFAULT  35
#define CLK_PIN_DEFAULT 34




