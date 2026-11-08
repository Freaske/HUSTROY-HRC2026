# Wokwi: quan sát tín hiệu số, phương án chờ duyệt

Đây không phải mô phỏng nguồn, TB6612FNG hoặc motor DC. Không dùng sơ đồ này
làm bằng chứng điện áp/dòng/RPM đúng trên phần cứng thật.

1. Build project chính bằng `pio run -e esp32dev`.
2. Mở **thư mục này** trong cửa sổ VS Code khác, giữ project chính nguyên vẹn.
3. Chạy `Wokwi: Start Simulator`. Cần Wokwi extension và quyền sử dụng của bạn.
4. Firmware hiện vẫn khóa: HELP/STATUS hoạt động, lệnh chạy báo BLOCKED,
   LED GPIO2 nháy. Không kỳ vọng PWM ở các chân motor chưa được khởi tạo.

| Analyzer | GPIO đề xuất | Tín hiệu dự định quan sát sau khi mapping được duyệt |
| --- | --- | --- |
| D0 | 25 | PWMA |
| D1 | 26 | AIN1 |
| D2 | 27 | AIN2 |
| D3 | 18 | PWMB |
| D4 | 19 | BIN1 |
| D5 | 23 | BIN2 |
| D6 | 4 | STBY; có điện trở kéo xuống 10 kΩ |
| D7 | Không nối | Dự phòng |
| GND | GND ESP32 | Mốc chung |

Ví dụ chỉ **nối đầu đo**, không sửa `include/config.h`, không đổi firmware
sang chế độ bật motor. Chưa chạy sơ đồ riêng này trong simulator; chỉ kiểm
tra cấu trúc JSON, kết nối và đường dẫn firmware. Sau khi có phê duyệt và
cấu hình đúng, analyzer có thể dùng kiểm tra tín hiệu 20 kHz, chân chiều và
STBY về LOW sau timeout. Tín hiệu không chứng minh motor đã quay.

Khi một kênh dừng còn kênh kia chạy, PWM của kênh dừng có thể HIGH liên tục:
IN1/IN2 LOW + PWM HIGH là lựa chọn coast của TB6612, không phải chạy hết tốc.
