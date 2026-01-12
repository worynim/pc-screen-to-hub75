// esp32 3.3.3 에서 fps 더 잘나옴 3.3.5 는 왜인지 잘안나옴
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <TJpg_Decoder.h>

// WiFi 설정 (기존과 동일)
const int udpPort = 12345;

WiFiUDP udp;
WiFiUDP discoveryUDP;
const char* discoveryMsg = "ESP32_MATRIX_OFFER";  // 파이썬과 약속한 키워드
const int discoveryPort = 12346;
unsigned long lastDiscoveryTime = 0;           // 마지막 전송 시간 저장
const unsigned long discoveryInterval = 3000;  // 간격 (3000ms = 3초)
void sendDiscoveryPacket() {
  IPAddress broadcastIP = WiFi.localIP();
  broadcastIP[3] = 255;  // 예: 192.168.1.255 (네트워크 전체에 전송)

  discoveryUDP.beginPacket(broadcastIP, discoveryPort);
  discoveryUDP.print(discoveryMsg);
  discoveryUDP.endPacket();
}

#define WIDTH 128
#define HEIGHT 64
// JPEG 데이터는 원본보다 작으므로 기존 버퍼 크기를 그대로 써도 충분합니다.
#define FRAME_SIZE (WIDTH * HEIGHT * 2)

#define HEADER_SIZE 3
#define CHUNK_PAYLOAD_SIZE 1024
#define MAX_PACKET_SIZE (CHUNK_PAYLOAD_SIZE + HEADER_SIZE)
#define FRAME_RECV_TIMEOUT_MS 100

const uint8_t WF2_LED = 40;

MatrixPanel_I2S_DMA* dma_display = nullptr;

uint8_t partial_frame_buffer[FRAME_SIZE];
uint8_t packet_buffer[MAX_PACKET_SIZE];

uint8_t current_frame_id = 0;
bool is_assembling = false;
uint32_t received_chunk_mask = 0;  // 32비트로 확장
uint8_t total_chunks_for_frame = 0;
unsigned long last_packet_time = 0;
uint16_t current_data_len = 0;  // 수신된 실제 JPEG 바이트 길이 저장


// FPS 측정용 변수
unsigned long fps_last_time = 0;
int frame_count = 0;
float current_fps = 0;

// [중요] JPEG 디코딩 콜백 함수
// 디코더가 JPEG의 한 조각(MCU)을 풀 때마다 호출되어 매트릭스에 그립니다.
bool tjpgd_callback(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (dma_display == nullptr) return false;
  // 디코딩된 비트맵 조각을 화면에 출력
  dma_display->drawRGBBitmap(x, y, bitmap, w, h);
  return true;
}

void setupMatrix() {
  Serial.println("init matrix... ");
  HUB75_I2S_CFG mxconfig(WIDTH, HEIGHT, 1);

  mxconfig.double_buff = true;
  mxconfig.driver = HUB75_I2S_CFG::DP3246;
  mxconfig.line_decoder = HUB75_I2S_CFG::SM5368;

  mxconfig.latch_blanking = 1;
  mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_20M;
  mxconfig.clkphase = true;

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);

  if (not dma_display->begin()) {
    Serial.println("****** !KABOOM! I2S memory allocation failed ***********");
    delay(100);
    // ESP.restart();  // 타임아웃 시 재부팅하여 다시 시도
  }
  dma_display->setBrightness8(128);
}
void reset_assembly_state() {
  is_assembling = false;
  received_chunk_mask = 0;
  current_data_len = 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("program start~!");
  delay(1000);

  setupMatrix();

  pinMode(WF2_LED, OUTPUT);
  digitalWrite(WF2_LED, HIGH);

  dma_display->fillScreen(0);
  dma_display->setTextWrap(false);
  dma_display->setTextSize(1);
  dma_display->setTextColor(dma_display->color565(128, 128, 128));  // 흰색
  dma_display->setCursor(2, 25);
  dma_display->print("AP : ");
  dma_display->println("ESP_WIFI_SCREEN");
  dma_display->println("Set SSID & PASSWORD");
  dma_display->flipDMABuffer();  // 화면에 표시
  delay(100);
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(120);
  if (!wifiManager.autoConnect("ESP_WIFI_SCREEN")) {
    Serial.println("failed to connect and hit timeout");
    delay(100);

  }

  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  udp.begin(udpPort);

  // WiFi 연결 후 IP 주소를 시리얼과 매트릭스에 표시
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  dma_display->fillScreen(0);
  dma_display->setTextWrap(false);
  dma_display->setTextSize(1);
  dma_display->setTextColor(dma_display->color565(128, 128, 128));  // 흰색
  dma_display->setCursor(2, 4);
  dma_display->println("IP Address:");
  dma_display->setCursor(2, dma_display->getCursorY() + 2);
  dma_display->println(WiFi.localIP().toString());
  dma_display->flipDMABuffer();  // 화면에 표시

  delay(2000);  // 5초간 IP 주소 표시
  discoveryUDP.begin(discoveryPort);

  // JPEG 디코더 설정
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tjpgd_callback);
}


void loop() {
  unsigned long currentMillis = millis();

  // --- 1. 브로드캐스트 제어 로직 ---
  // 마지막 패킷 수신 후 1초(FRAME_RECV_TIMEOUT_MS)가 지났을 때만 브로드캐스트 모드 활성화
  if (currentMillis - last_packet_time >= FRAME_RECV_TIMEOUT_MS) {

    // 3초 간격으로 "나 여기 있어" 외치기
    if (currentMillis - lastDiscoveryTime >= discoveryInterval) {
      lastDiscoveryTime = currentMillis;

      sendDiscoveryPacket();  // IP 주소 브로드캐스팅
      Serial.print("Discovery Mode: ");
      Serial.println(WiFi.localIP());
    }
  }

  // --- 2. 패킷 수신 전 타임아웃 처리 (기존 로직) ---
  if (is_assembling && (currentMillis - last_packet_time > FRAME_RECV_TIMEOUT_MS)) {
    reset_assembly_state();
    Serial.println("Streaming Timeout - Resetting...");
  }

  // --- 3. UDP 패킷 파싱 ---
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  int len = udp.read(packet_buffer, MAX_PACKET_SIZE);
  if (len < HEADER_SIZE) return;

  // 패킷을 성공적으로 읽었으므로 마지막 수신 시간 갱신 (브로드캐스트 중단됨)
  last_packet_time = currentMillis;

  uint8_t frame_id = packet_buffer[0];
  uint8_t total_chunks = packet_buffer[1];
  uint8_t chunk_index = packet_buffer[2];

  // 새 프레임 시작점 감지
  if (current_frame_id != frame_id) {
    current_frame_id = frame_id;
    total_chunks_for_frame = total_chunks;
    received_chunk_mask = 0;
    current_data_len = 0;
    is_assembling = true;
  }

  if (is_assembling) {
    // (중복 수신 방지 및 데이터 저장)
    if (!(received_chunk_mask & (1UL << chunk_index))) {
      received_chunk_mask |= (1UL << chunk_index);
      int data_len = len - HEADER_SIZE;
      int offset = chunk_index * CHUNK_PAYLOAD_SIZE;

      if (offset + data_len <= FRAME_SIZE) {
        memcpy(&partial_frame_buffer[offset], &packet_buffer[HEADER_SIZE], data_len);
        if (offset + data_len > current_data_len) current_data_len = offset + data_len;
      }

      uint32_t all_chunks_mask = (1UL << total_chunks_for_frame) - 1;

      // 모든 조각이 다 모였을 때 디코딩
      if (received_chunk_mask == all_chunks_mask) {
        is_assembling = false;

        // TJpgDec를 이용한 화면 출력
        int8_t res = TJpgDec.drawJpg(0, 0, partial_frame_buffer, current_data_len);

        if (res == 0) {
          dma_display->flipDMABuffer();  // 더블 버퍼링 교체

          // [추가] 프레임 출력 성공 시 카운트 증가
          frame_count++;
          digitalWrite(WF2_LED, !digitalRead(WF2_LED));
        }

        // 상태 초기화
        received_chunk_mask = 0;
        current_data_len = 0;
      }
    }
  }

  // [추가] 1초마다 시리얼 모니터에 FPS 출력
  if (currentMillis - fps_last_time >= 1000) {
    current_fps = frame_count;  // 1초 동안 쌓인 프레임 수

    if (current_fps > 0) {
      Serial.print("Real-time Display FPS: ");
      Serial.println(current_fps);
    }

    // 초기화
    frame_count = 0;
    fps_last_time = currentMillis;
  }
}