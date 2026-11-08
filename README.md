# HUSTROY-HRC2026

ESP32 + PlatformIO, giai đoạn kiểm tra TB6612FNG qua Serial.

**Mặc định khóa đầu ra động cơ**: chưa có GPIO được Toàn xác nhận.
Không nối nguồn động cơ cho đến khi kiểm tra xong phần cứng.

```sh
pio run -e esp32dev
python3 tests/run_host_tests.py
```

Serial 115200 baud: `HELP`, `STATUS`, `LEFT 30`, `RIGHT -30`, `BOTH 30`,
`BOTH -30`, `STOP`. Nhấn Enter sau mỗi lệnh; `!` dừng không cần Enter.
Mỗi lệnh chuyển động hết hạn sau 500 ms tính từ lệnh chuyển động hợp lệ gần nhất.

Đọc [hướng dẫn tiếng Việt: phần cứng, 5 bài học, cách thử và giới hạn kiểm chứng](docs/tb6612-guide.vi.md).
