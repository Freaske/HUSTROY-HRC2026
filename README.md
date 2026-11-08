# HUSTROY-HRC2026

ESP32 + PlatformIO: TB6612FNG, năm cảm biến AO/DO, PID và ba chế độ Serial.

**Bản thử motor thủ công:** driver bật với GPIO 18/13/14/19/21/22/23 theo
bảng dây đã trao đổi. Khởi động vẫn STOPPED/DISARMED, cần ARM và lệnh chạy;
cảm biến còn khóa nên LINE_FOLLOW chưa chạy được. Điện áp/dòng motor, dây
và chiều bánh chưa được kiểm chứng độc lập. Ngắt VM khi nạp, kê bánh khi thử.

Đọc [quy trình cho bản motor đang bật](docs/manual-motor-bench.vi.md).

```sh
pio run -e esp32dev
python3 tests/run_host_tests.py
```

Lần đầu nạp board thật: đọc [hướng dẫn USB-C, Upload và Serial Monitor](docs/esp32-upload-guide.vi.md).
Hướng dẫn upload cũng mô tả bản chẩn đoán cũ: muốn trở lại bản đó, đặt
MOTOR_HARDWARE_ENABLED=false trước khi build/nạp.

Serial 115200 baud, viết HOA và Enter:

- `MODE MOTOR_TEST`, rồi `ARM`: `LEFT 30`, `RIGHT -30`, `BOTH 30`, `SET 50 30`; timeout 500 ms và tự DISARM.
- `MODE SENSOR_TEST`: chỉ đọc/log cảm biến, motor dừng.
- `MODE LINE_FOLLOW`: `ARM`, chờ line hợp lệ, rồi `RUN`; không tự chạy.
- `STOP` dừng và giữ ARM; `DISARM` dừng và khóa lại. `!` dừng + DISARM không cần Enter, sau đó Enter để xóa dòng.
- `HELP`, `STATUS` xem hướng dẫn/trạng thái. Nút mode nếu được bật chỉ chọn chế độ.

Mỗi `RUN` là lần thử tối đa 5 giây mặc định. LOST, INTERSECTION, dữ liệu không
hợp lệ hoặc vòng điều khiển chậm đều dừng + DISARM, không tự khởi động lại.
Đổi chế độ luôn DISARM; ARM không thể vượt khóa phần cứng trong config.

Đọc [hướng dẫn thử từng bánh, ARM/DISARM và ghi nhận chiều motor](docs/physical-motor-first-drive.vi.md).

Đọc [hướng dẫn tích hợp và năm bài học tiếng Việt](docs/sensor-pid-integration.vi.md)
trước khi cấu hình phần cứng. [Mã gốc đồng đội](reference/line_follower/README.md)
được giữ nguyên để đối chiếu.

[Hướng dẫn motor của pha trước](docs/tb6612-guide.vi.md) vẫn là tài liệu bổ trợ;
hướng dẫn tích hợp ở trên mô tả giao diện chế độ hiện tại.
