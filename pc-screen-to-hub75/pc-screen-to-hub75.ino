/**
 * ESP32-S3 HUB75 LED Matrix 실시간 스트리밍 수신기
 * * 주요 기능:
 * 1. WiFiManager를 통한 간편한 Wi-Fi 설정
 * 2. UDP 패킷 재조립 (분할 전송된 JPEG 데이터 복원)
 * 3. TJpg_Decoder를 이용한 고속 하드웨어 가속 디코딩
 * 4. 자동 IP 탐색(Discovery) 기능을 통한 서버 연결 편의성
 * * 참고: ESP32 Arduino Core 3.3.3 버전에서 최적의 성능(FPS)을 보입니다.
 */

#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <TJpg_Decoder.h>

// -------------------------------------------------------------------------
// 1. 설정 및 상수 정의
// -------------------------------------------------------------------------
const int udpPort = 12345;                       // 영상 데이터 수신 포트
const int discoveryPort = 12346;                 // 서버(파이썬) 탐색용 포트
const char* discoveryMsg = "ESP32_MATRIX_OFFER"; // 서버에 보낼 응답 키워드
const unsigned long discoveryInterval = 3000;    // 탐색 패킷 송신 간격 (3초)

#define WIDTH 128                                // 매트릭스 가로 해상도
#define HEIGHT 64                                // 매트릭스 세로 해상도
#define FRAME_SIZE (WIDTH * HEIGHT * 2)          // JPEG 수신 버퍼 크기
#define HEADER_SIZE 3                            // UDP 패킷 헤더 크기 (ID, Total, Index)
#define CHUNK_PAYLOAD_SIZE 1024                  // 패킷당 데이터 페이로드 크기
#define MAX_PACKET_SIZE (CHUNK_PAYLOAD_SIZE + HEADER_SIZE)
#define FRAME_RECV_TIMEOUT_MS 100                // 데이터 수신 타임아웃 (100ms)

const uint8_t WF2_LED = 40;                      // 상태 표시용 온보드 LED

// -------------------------------------------------------------------------
// 2. 전역 객체 및 버퍼 선언
// -------------------------------------------------------------------------
WiFiUDP udp;
WiFiUDP discoveryUDP;
MatrixPanel_I2S_DMA* dma_display = nullptr;

uint8_t partial_frame_buffer[FRAME_SIZE];        // 재조립용 프레임 버퍼
uint8_t packet_buffer[MAX_PACKET_SIZE];          // 개별 패킷 수신 버퍼

// 프레임 재조립 상태 변수
uint8_t current_frame_id = 0;
bool is_assembling = false;
uint32_t received_chunk_mask = 0;                // 수신된 조각 체크 (비트마스크)
uint8_t total_chunks_for_frame = 0;
unsigned long last_packet_time = 0;
uint16_t current_data_len = 0;                   // 현재까지 쌓인 데이터 길이
unsigned long lastDiscoveryTime = 0;

// FPS 계산용 변수
unsigned long fps_last_time = 0;
int frame_count = 0;
float current_fps = 0;

// -------------------------------------------------------------------------
// 3. 유틸리티 함수 및 콜백
// -------------------------------------------------------------------------

/**
 * JPEG 디코더 콜백 함수
 * 디코더가 JPEG의 MCU 조각을 처리할 때마다 호출되어 화면에 출력합니다.
 */
bool tjpgd_callback(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (dma_display == nullptr) return false;
  dma_display->drawRGBBitmap(x, y, bitmap, w, h);
  return true;
}

/**
 * 브로드캐스트 패킷 전송
 * 파이썬 클라이언트가 ESP32의 IP를 자동으로 찾을 수 있도록 정보를 보냅니다.
 */
void sendDiscoveryPacket() {
  IPAddress broadcastIP = WiFi.localIP();
  broadcastIP[3] = 255;  // 서브넷 브로드캐스트 주소 설정

  discoveryUDP.beginPacket(broadcastIP, discoveryPort);
  discoveryUDP.print(discoveryMsg);
  discoveryUDP.endPacket();
}

/**
 * 매트릭스 하드웨어 설정 초기화
 */
void setupMatrix() {
  Serial.println("Initializing LED Matrix...");
  HUB75_I2S_CFG mxconfig(WIDTH, HEIGHT, 1);

  mxconfig.double_buff = true;               // 플리커 방지를 위한 더블 버퍼링 활성화
  mxconfig.driver = HUB75_I2S_CFG::DP3246;   // 드라이버 IC 타입 설정
  mxconfig.line_decoder = HUB75_I2S_CFG::SM5368;
  mxconfig.latch_blanking = 1;
  mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_20M; // 데이터 전송 속도 (20MHz)
  mxconfig.clkphase = true;

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);

  if (!dma_display->begin()) {
    Serial.println("****** ERROR: I2S Memory Allocation Failed! ******");
  }
  dma_display->setBrightness8(128);          // 밝기 설정 (0~255)
}

void reset_assembly_state() {
  is_assembling = false;
  received_chunk_mask = 0;
  current_data_len = 0;
}

// -------------------------------------------------------------------------
// 4. 초기 설정 (Setup)
// -------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupMatrix(); // 매트릭스 초기화

  pinMode(WF2_LED, OUTPUT);
  digitalWrite(WF2_LED, HIGH);

  // WiFi 연결 전 안내 문구 표시
  dma_display->fillScreen(0);
  dma_display->setTextSize(1);
  dma_display->setTextColor(dma_display->color565(128, 128, 128));
  dma_display->setCursor(2, 25);
  dma_display->println("AP : ESP_WIFI_SCREEN");
  dma_display->println("Setup WiFi via Phone");
  dma_display->flipDMABuffer();

  // WiFiManager를 통해 설정 포털 생성
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(120);
  if (!wifiManager.autoConnect("ESP_WIFI_SCREEN")) {
    Serial.println("WiFi Connection Timeout!");
    delay(100);
  }

  WiFi.setSleep(false); // 성능을 위해 WiFi 절전 모드 비활성화
  while (WiFi.status() != WL_CONNECTED) delay(500);

  udp.begin(udpPort);
  discoveryUDP.begin(discoveryPort);

  // 연결된 IP 주소 출력
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  dma_display->fillScreen(0);
  dma_display->setCursor(2, 4);
  dma_display->println("Connected IP:");
  dma_display->setCursor(2, 14);
  dma_display->println(WiFi.localIP().toString());
  dma_display->flipDMABuffer();

  delay(2000);

  // JPEG 디코더 설정
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tjpgd_callback);
}

// -------------------------------------------------------------------------
// 5. 메인 루프 (Loop)
// -------------------------------------------------------------------------

void loop() {
  unsigned long currentMillis = millis();

  // [Task 1] 서버 탐색(Discovery) 및 패킷 수신 대기 제어
  if (currentMillis - last_packet_time >= FRAME_RECV_TIMEOUT_MS) {
    if (currentMillis - lastDiscoveryTime >= discoveryInterval) {
      lastDiscoveryTime = currentMillis;
      sendDiscoveryPacket(); // 데이터가 안 올 때만 자신을 알림
      Serial.print("Waiting for Server... IP: ");
      Serial.println(WiFi.localIP());
    }
  }

  // [Task 2] 데이터 수신 중 타임아웃 발생 시 상태 초기화
  if (is_assembling && (currentMillis - last_packet_time > FRAME_RECV_TIMEOUT_MS)) {
    reset_assembly_state();
    Serial.println("Packet Loss: Timeout Reset");
  }

  // [Task 3] UDP 데이터 수신 처리
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  int len = udp.read(packet_buffer, MAX_PACKET_SIZE);
  if (len < HEADER_SIZE) return;

  last_packet_time = currentMillis;

  uint8_t frame_id   = packet_buffer[0];
  uint8_t total_chunks = packet_buffer[1];
  uint8_t chunk_index  = packet_buffer[2];

  // 새로운 프레임이 감지되면 초기화
  if (current_frame_id != frame_id) {
    current_frame_id = frame_id;
    total_chunks_for_frame = total_chunks;
    received_chunk_mask = 0;
    current_data_len = 0;
    is_assembling = true;
  }

  if (is_assembling) {
    // 중복 패킷이 아닌 경우에만 처리
    if (!(received_chunk_mask & (1UL << chunk_index))) {
      received_chunk_mask |= (1UL << chunk_index);
      
      int data_len = len - HEADER_SIZE;
      int offset = chunk_index * CHUNK_PAYLOAD_SIZE;

      // 버퍼 범위 내에서 데이터 복사
      if (offset + data_len <= FRAME_SIZE) {
        memcpy(&partial_frame_buffer[offset], &packet_buffer[HEADER_SIZE], data_len);
        if (offset + data_len > current_data_len) current_data_len = offset + data_len;
      }

      // 모든 조각이 도착했는지 확인
      uint32_t all_chunks_mask = (1UL << total_chunks_for_frame) - 1;
      if (received_chunk_mask == all_chunks_mask) {
        is_assembling = false;

        // [핵심] JPEG 디코딩 및 매트릭스 출력
        int8_t res = TJpgDec.drawJpg(0, 0, partial_frame_buffer, current_data_len);

        if (res == 0) { // 디코딩 성공 시
          dma_display->flipDMABuffer(); // 버퍼 스왑
          frame_count++;
          digitalWrite(WF2_LED, !digitalRead(WF2_LED)); // LED 토글
        }
        reset_assembly_state();
      }
    }
  }

  // [Task 4] FPS 모니터링 (1초 간격)
  if (currentMillis - fps_last_time >= 1000) {
    current_fps = frame_count;
    if (current_fps > 0) {
      Serial.print("Performance: ");
      Serial.print(current_fps);
      Serial.println(" FPS");
    }
    frame_count = 0;
    fps_last_time = currentMillis;
  }
}
