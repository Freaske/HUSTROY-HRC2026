# Chỉnh tham số và xem log trên điện thoại

## Nạp và kết nối

| Thiết bị | Build | Kết nối |
| --- | --- | --- |
| Android | `pio run -e esp32dev-bt` | Ghép đôi `HUSTROY-HRC2026`, mở [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal), chọn Bluetooth Classic và đặt ký tự kết thúc là LF hoặc CRLF. |
| iPhone | `pio run -e esp32dev-wifi` | Mở USB Serial 115200 sau khi reset để đọc mật khẩu Wi‑Fi của lần bật hiện tại. Kết nối Wi‑Fi `HUSTROY-HRC2026`, giữ mạng khi iPhone báo không có internet và mở `http://192.168.4.1/` trong Safari. Mạng nhận một thiết bị tại một thời điểm. |
| USB / Wokwi | `pio run -e esp32dev` | Dùng USB Serial 115200 hoặc cửa sổ Serial của Wokwi. |

Để nạp một cấu hình, thêm `-t upload` vào lệnh PlatformIO. Dùng bản Bluetooth trên ESP32 Classic (`esp32dev`), dựa trên [BluetoothSerial SPP của Espressif](https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial).

## Lệnh

| Lệnh | Tác dụng |
| --- | --- |
| `KP=1.2`, `KD=6` | Chỉnh gain trong RAM, dùng dấu chấm thập phân. Miền 0–1000, với đạo hàm tính theo giây. Thay đổi gain đặt lại lịch sử PID để tránh bước nhảy ở thành phần D. |
| `SPD=150` | Chỉnh PWM nền (số nguyên 1–150 với `MAX_SPEED=150` hiện tại). Khi bánh ngoài chạm giới hạn, PID vẫn có thể giảm PWM bánh trong để rẽ. Đây là giá trị duty 8 bit. |
| `GET` | Xem `KP`, `KI`, `KD`, `SPD` và giới hạn PWM `MAX`. `KI` lấy từ cấu hình build. |
| `SAVE` | Lưu toàn bộ tham số đang dùng vào NVS trong flash. Gửi `STOP` trước khi `SAVE`. Khi reset, firmware đọc lại giá trị đã lưu. |
| `MODE LINE_FOLLOW`, `ARM`, `GO` | Chọn chế độ, cho phép chạy, chờ bắt được line ổn định rồi bắt đầu. `GO` dùng cùng điều kiện với `RUN` qua USB. |
| `STOP`, `DISARM` | Dừng cả hai bánh. `DISARM` đồng thời khóa chạy. Qua Bluetooth hoặc USB, gửi `!` để dừng và khóa ngay, sau đó Enter để bắt đầu dòng mới. |
| `MODE SENSOR_TEST`, `HELP` | Xem cảm biến và danh sách lệnh. Các lệnh `LEFT/RIGHT/BOTH/SET` dùng qua USB trong `MOTOR_TEST`. |

Trong `LINE_FOLLOW`, gain và PWM nền hợp lệ có hiệu lực ở nhịp điều khiển kế tiếp (chu kỳ 20 ms). Điều kiện `ARM`, hiệu chuẩn cảm biến và line hợp lệ vẫn được kiểm tra khi `GO`. Firmware đã bật đọc thô năm ngõ AO. Làm T01/T02 theo [kịch bản test](ke-hoach-test.vi.md) trước khi lái trên sân.

Dùng lệnh từ điện thoại trong các buổi thử nội bộ. Khi thi, hoàn tất chỉnh tham số trước khi đặt xe vào START, dùng nút để xuất phát theo hiệu lệnh và để xe tự hành trong lượt. [Thể lệ HRC 2026, mục 3.1 và 6](https://drive.google.com/file/d/1PVf8mOg5K7oMSuhhUea1b_E_kdrq-ePu/view)

Bluetooth phát khoảng 15 dòng log mỗi giây. Hai dòng dưới đây minh họa trạng thái đang chạy và ngay sau khi dừng.

```text
t=1820 MODE=LINE_FOLLOW RUN=1 ARM=1 LINE=LINE_DETECTED BITS=01100 e=-0.50 PWM_L=40 PWM_R=60 STOP=-
t=1887 MODE=LINE_FOLLOW RUN=0 ARM=1 LINE=LINE_DETECTED BITS=01100 e=0.00 PWM_L=0 PWM_R=0 STOP=STOP
```

`t` là mili giây từ lúc bật. `e` là sai số PID có dấu, bằng 0 khi đã dừng. `PWM_L/R` là lệnh duty trái/phải. Bluetooth gửi qua tác vụ riêng, giữ mẫu mới khi kênh nghẽn. Wi‑Fi cập nhật mẫu mới nhất trên trang điều khiển.

## Giao diện cho người tích hợp PID

Include `tuning.h` rồi đọc `const auto& params = tuningParameters();` trong vòng điều khiển để dùng `params.kp`, `params.kd` và `params.speed`. Firmware hiện gọi `tuningInit()` khi khởi động và đọc trực tiếp các trường này trong `modesUpdate()`.
