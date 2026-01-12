import io
import socket
import time
import threading
import queue
import numpy as np
from PIL import Image, ImageEnhance, ImageTk
import mss
import tkinter as tk
from tkinter import messagebox
import subprocess
import os

# --- 설정 및 전역 변수 ---
UDP_PORT = 12345
DISCOVERY_PORT = 12346
WIDTH, HEIGHT = 128, 64
CHUNK_PAYLOAD_SIZE = 1024
TARGET_FPS = 35

streaming_active = False
send_queue = queue.Queue(maxsize=2)

def reset_replayd():
    """ macOS 리소스 초기화 """
    try:
        if os.name == 'posix' and os.uname().sysname == 'Darwin':
            subprocess.run(["killall", "-9", "replayd"], check=False)
    except:
        pass

# --- 1. 전송 전용 스레드 ---
def udp_sender_thread(target_ip_func):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        if not streaming_active:
            time.sleep(0.1)
            continue
        try:
            target_ip = target_ip_func()
            frame_id, jpeg_data = send_queue.get(timeout=1)
            total_size = len(jpeg_data)
            total_chunks = (total_size + CHUNK_PAYLOAD_SIZE - 1) // CHUNK_PAYLOAD_SIZE
            for i in range(total_chunks):
                offset = i * CHUNK_PAYLOAD_SIZE
                chunk_size = min(CHUNK_PAYLOAD_SIZE, total_size - offset)
                header = bytes([frame_id & 0xFF, total_chunks & 0xFF, i & 0xFF])
                packet = header + jpeg_data[offset:offset + chunk_size]
                sock.sendto(packet, (target_ip, UDP_PORT))
                time.sleep(0.0001)
            send_queue.task_done()
        except:
            continue

# --- 2. 이미지 보정 및 압축 ---
def process_and_compress(img: Image.Image):
    img_resized = img.resize((WIDTH, HEIGHT), Image.BOX)
    img_resized = img_resized.point(lambda p: p if p > 20 else 0)
    r, g, b = img_resized.split()
    
    # ESP32용 (BGR)
    img_esp = Image.merge("RGB", (b, g, r))
    img_esp = ImageEnhance.Color(img_esp).enhance(1.5)
    img_esp = ImageEnhance.Contrast(img_esp).enhance(1.3)
    
    buffer = io.BytesIO()
    img_esp.save(buffer, format="JPEG", quality=80, optimize=True)
    
    # 프리뷰용 (RGB)
    img_preview = Image.merge("RGB", (r, g, b))
    img_preview = ImageEnhance.Color(img_preview).enhance(1.5)
    
    return buffer.getvalue(), img_preview

# --- 3. IP 자동 검색 ---
def discover_esp32_ip():
    scanner = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    scanner.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try: scanner.bind(('', DISCOVERY_PORT))
    except: return None
    scanner.settimeout(3.0)
    try:
        while True:
            data, addr = scanner.recvfrom(1024)
            if data.decode() == "ESP32_MATRIX_OFFER":
                return addr[0]
    except: return None
    finally: scanner.close()

# --- 4. 메인 스트리밍 루프 ---
def start_streaming():
    global streaming_active
    frame_id = 0
    fps_frame_count = 0
    last_fps_update_time = time.perf_counter()

    with mss.mss() as sct:
        monitor = sct.monitors[1]
        target_ratio = WIDTH / HEIGHT
        region = {
            "top": monitor["top"] + (monitor['height'] - int(monitor['width'] / target_ratio)) // 2,
            "left": monitor["left"],
            "width": monitor['width'],
            "height": int(monitor['width'] / target_ratio)
        }

        while streaming_active:
            start_time = time.perf_counter()
            sct_img = sct.grab(region)
            img = Image.frombytes("RGB", sct_img.size, sct_img.bgra, "raw", "BGRX")
            jpeg_data, preview_img = process_and_compress(img)

            if send_queue.full():
                try: send_queue.get_nowait()
                except: pass
            send_queue.put((frame_id, jpeg_data))

            preview_resized = preview_img.resize((256, 128), Image.BOX)
            img_tk = ImageTk.PhotoImage(preview_resized)
            label_preview.config(image=img_tk)
            label_preview.image = img_tk

            frame_id += 1
            fps_frame_count += 1
            curr_time = time.perf_counter()
            if curr_time - last_fps_update_time >= 1.0:
                actual_fps = fps_frame_count / (curr_time - last_fps_update_time)
                fps_var.set(f"FPS: {actual_fps:.1f}")
                fps_frame_count = 0
                last_fps_update_time = curr_time

            elapsed = time.perf_counter() - start_time
            time.sleep(max(0, (1.0 / TARGET_FPS) - elapsed))

# --- 5. GUI 이벤트 ---
def toggle_streaming():
    global streaming_active
    if not streaming_active:
        streaming_active = True
        btn_start.config(text="STOP STREAMING")
        threading.Thread(target=start_streaming, daemon=True).start()
    else:
        streaming_active = False
        btn_start.config(text="START STREAMING")
        fps_var.set("FPS: 0.0")

def auto_find_ip():
    btn_find.config(text="Searching...", state=tk.DISABLED)
    root.update()
    found_ip = discover_esp32_ip()
    if found_ip:
        entry_ip.delete(0, tk.END)
        entry_ip.insert(0, found_ip)
        btn_find.config(text="🔍 Found!", state=tk.NORMAL)
        root.after(2000, lambda: btn_find.config(text="🔍 Auto Find"))
    else:
        btn_find.config(text="🔍 Retry", state=tk.NORMAL)

# --- 6. GUI 레이아웃 (무채색 & 이전 배치 복구) ---
root = tk.Tk()
root.title("ESP32 Matrix Streamer Pro")
root.geometry("350x480")

# 미리보기
tk.Label(root, text="[ Preview ]", font=("Arial", 10)).pack(pady=5)
black_img = ImageTk.PhotoImage(Image.new('RGB', (256, 128), (0, 0, 0)))
label_preview = tk.Label(root, image=black_img, relief="solid", borderwidth=1)
label_preview.pack(pady=10)

# FPS 표시
fps_var = tk.StringVar(value="FPS: 0.0")
label_fps = tk.Label(root, textvariable=fps_var, font=("Courier", 14, "bold"))
label_fps.pack(pady=5)

# IP 입력 및 찾기 버튼 (이전 배치 복구)
frame_ip = tk.Frame(root)
frame_ip.pack(pady=20)
tk.Label(frame_ip, text="IP: ").pack(side=tk.LEFT)
entry_ip = tk.Entry(frame_ip, width=15, font=("Arial", 12))
entry_ip.insert(0, "192.168.10.34")
entry_ip.pack(side=tk.LEFT, padx=5)
btn_find = tk.Button(frame_ip, text="🔍 Auto Find", command=auto_find_ip)
btn_find.pack(side=tk.LEFT)

# 시작 버튼 (이전 크기 및 무채색 적용)
btn_start = tk.Button(root, text="START STREAMING", command=toggle_streaming, 
                      width=20, height=2, font=("Arial", 12, "bold"), 
                      bg="#eeeeee", fg="#333333")
btn_start.pack(pady=10)

# 전송 스레드 가동
threading.Thread(target=udp_sender_thread, args=(lambda: entry_ip.get().strip(),), daemon=True).start()

if __name__ == "__main__":
    reset_replayd()
    root.mainloop()
