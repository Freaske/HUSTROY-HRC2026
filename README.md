# HUSTROY-HRC2026

ESP32 + PlatformIO: TB6612FNG, năm cảm biến AO/DO, PID và ba chế độ Serial.

**Bản thử motor thủ công:** driver bật với GPIO 18/13/14/19/21/22/23 theo
bảng dây đã trao đổi. Khởi động vẫn STOPPED/DISARMED, cần ARM và lệnh chạy.
Năm ngõ AO đọc thô trên GPIO 32/33/34/35/36. LINE_FOLLOW chờ hiệu chuẩn và
kiểm tra tín hiệu khi tuột dây. Điện áp/dòng motor, dây
và chiều bánh chưa được kiểm chứng độc lập. Ngắt VM khi nạp, kê bánh khi thử.

Đọc [quy trình cho bản motor đang bật](docs/manual-motor-bench.vi.md).

Tài liệu cho đợt test của Nam gồm [chỉnh tham số và log trên điện thoại](docs/test-link.vi.md),
[kịch bản test sa bàn](docs/ke-hoach-test.vi.md) và [khảo sát encoder](docs/encoder.vi.md).

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

Lần đầu nạp board thật: đọc [hướng dẫn USB-C, Upload và Serial Monitor](docs/esp32-upload-guide.vi.md).
Hướng dẫn upload cũng mô tả bản chẩn đoán cũ: muốn trở lại bản đó, đặt
MOTOR_HARDWARE_ENABLED=false trước khi build/nạp.

Serial 115200 baud, viết HOA và Enter:

- `MODE MOTOR_TEST`, rồi `ARM`: `LEFT 30`, `RIGHT -30`, `BOTH 30`, `SET 50 30`; timeout **10 giây** và tự DISARM.
- `MODE RIGHT_LINE`, `ARM`, đặt vạch ngay dưới mắt giữa, rồi `GO`. Bánh phải chạy PWM 170 tối đa 10 giây. Mất vạch giữa hoặc hết giờ sẽ dừng và DISARM. Kê bánh khi thử.
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
