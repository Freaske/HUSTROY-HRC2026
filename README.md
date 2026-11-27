# HUSTROY-HRC2026

ESP32 + PlatformIO với TB6612FNG, năm cảm biến AO và bộ điều khiển LINE_FOLLOW V5.
Khởi động STOPPED/DISARMED. Driver dùng GPIO 25/26/27/19/21/22/23.
Thứ tự ngõ AO trái sang phải là 33/32/34/35/36.

Đọc [profile, bằng chứng và quy trình kiểm thử LINE_FOLLOW V5](docs/line-follow-v5.vi.md).

Đọc [quy trình cho bản motor đang bật](docs/manual-motor-bench.vi.md).

Tài liệu cho đợt test của Nam gồm [chỉnh tham số và log trên điện thoại](docs/test-link.vi.md),
[kịch bản test sa bàn](docs/ke-hoach-test.vi.md) và [khảo sát encoder](docs/encoder.vi.md).

## Bàn thử xe trên laptop

Cắm USB của xe, đóng Serial Monitor, rồi chạy `python dashboard/server.py` (cần
`pyserial`, có sẵn trong Python của PlatformIO). Trình duyệt tự mở trang
`http://127.0.0.1:8770`: chọn cổng, bấm **Kết nối**.

- **Motor:** giữ nút để chạy tiến/lùi/xoay hoặc theo 2 thanh trượt; thả tay là dừng,
  firmware tự dừng sau 1 s nếu mất kết nối. Đo vùng chết motor và lưu `FF=` vào xe.
- **Cảm biến:** 5 mắt hiển thị trực tiếp trên dải sa bàn; hiệu chuẩn 6 bước giống
  `scripts/line_trial.py calibrate`.
- **Lượt chạy:** bật tự chạy, xem biểu đồ vị trí vạch và lệnh 2 bánh; mỗi lượt tự lưu
  thống kê vào `dashboard/runs/runs.csv` và log 50 mẫu/giây vào `dashboard/runs/run-*.txt`.

Nút **Dừng xe** (hoặc phím Space) gửi `!`: dừng và khóa xe ngay. Tắt trang/server trước
khi nạp firmware, vì server giữ cổng COM.

## Mô phỏng xe trên sa bàn

Chạy `python3 sim/run.py` (cần Python 3 và trình biên dịch `c++` hỗ trợ C++17),
rồi mở [sa bàn mô phỏng](http://127.0.0.1:8765). Chọn bài, chỉnh `KP`, `KD`,
PWM nền và bấm **Chạy mô phỏng**. Có thể tạm dừng, tua hoặc xem từng nhịp
điều khiển. Chạy `python3 sim/run.py --demo` để xem kết quả sáu bài trong terminal.

Sa bàn tập có đoạn thẳng 1 m, cua R100, góc nhọn, ngã tư, đoạn đứt line và ô
đích mô phỏng. Chương trình biên dịch firmware C++ với GPIO ảo và cấp tín hiệu
từ năm mắt dò. Khi xe qua mốc kiểm tra, trình mô phỏng gửi `STOP`. Firmware tự
quyết định dừng khi mất line, gặp mẫu giao nhau hoặc hết thời gian chạy.
Kích thước xe, vị trí mắt dò và hệ số PWM sang tốc độ là giả định của mô hình 2D.

## Firmware ESP32

```sh
pio run -e esp32dev
python3 tests/run_host_tests.py
```

**Mô phỏng Wokwi** (`diagram.json` + `wokwi.toml` ở gốc): build trước, rồi
VS Code → `Wokwi: Start Simulator`. Wokwi không có TB6612/motor DC nên chỉ
mô phỏng tín hiệu điều khiển: LED trên PWMA/AIN1/AIN2 (18/13/14),
PWMB/BIN1/BIN2 (19/21/22), STBY (23) và logic analyzer. Bánh "chạy" khi
STBY sáng + IN1 hoặc IN2 sáng; độ sáng LED PWM theo duty. Terminal Wokwi
không hiện chữ đang gõ; để thấy chữ, khi simulator đang chạy mở terminal khác:
`pio device monitor -p rfc2217://localhost:4000 -b 115200 --echo --eol LF`.

Lần đầu nạp board thật: đọc [hướng dẫn USB-C, Upload và Serial Monitor](docs/esp32-upload-guide.vi.md).
Hướng dẫn upload cũng mô tả bản chẩn đoán cũ: muốn trở lại bản đó, đặt
MOTOR_HARDWARE_ENABLED=false trước khi build/nạp.

Serial 115200 baud, viết HOA và Enter:

- `MODE MOTOR_TEST`, rồi `ARM`: `LEFT 30`, `RIGHT -30`, `BOTH 30`, `SET 50 30`; timeout 10 s và tự DISARM.
- `MODE SENSOR_TEST`: chỉ đọc/log cảm biến, motor dừng.
- `MODE LINE_FOLLOW`: áp dụng profile `CAL`, `ARM`, giữ vạch gần giữa rồi `GO`.
- `STOP` dừng và giữ ARM; `DISARM` dừng và khóa lại. `!` dừng + DISARM không cần Enter, sau đó Enter để xóa dòng.
- `HELP`, `STATUS` xem hướng dẫn/trạng thái. Nút mode nếu được bật chỉ chọn chế độ.

`LIMIT=` đặt thời lượng mỗi lượt LINE_FOLLOW. Mất line quá 300 ms, mẫu giao lộ
kéo dài, lỗi cảm biến hoặc vòng điều khiển quá hạn sẽ dừng và DISARM. Đổi chế độ
luôn DISARM. Khóa phần cứng trong config vẫn áp dụng khi ARM.

Đọc [hướng dẫn thử từng bánh, ARM/DISARM và ghi nhận chiều motor](docs/physical-motor-first-drive.vi.md).

Đọc [hướng dẫn tích hợp và năm bài học tiếng Việt](docs/sensor-pid-integration.vi.md)
trước khi cấu hình phần cứng. [Mã gốc đồng đội](reference/line_follower/README.md)
được giữ nguyên để đối chiếu.

[Hướng dẫn motor của pha trước](docs/tb6612-guide.vi.md) vẫn là tài liệu bổ trợ;
hướng dẫn tích hợp ở trên mô tả giao diện chế độ hiện tại.
