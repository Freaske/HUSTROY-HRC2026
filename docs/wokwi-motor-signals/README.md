# Wokwi: demo GPIO/PWM của hai motor

Demo chạy **firmware ESP32 thật của project**, không có mô hình công suất
TB6612FNG hoặc motor DC. Đúng tín hiệu mô phỏng không chứng minh motor vật
lý quay, nguồn đủ dòng, mối hàn thông hoặc điện áp thực đúng.

## Hai LED đại diện hai motor

Hai LED RGB có nhãn **MOTOR TRAI / LEFT** và **MOTOR PHAI / RIGHT**.
Với cấu hình hiện tại A=trái, B=phải, INVERT=false:

| Lệnh sau ARM | LED trái | LED phải |
| --- | --- | --- |
| LEFT 50 | Xanh | Tắt |
| RIGHT 50 | Tắt | Xanh |
| BOTH 50 | Xanh | Xanh |
| LEFT -50 | Đỏ | Tắt |
| SET -50 50 | Đỏ | Xanh |
| STOP / DISARM / timeout10s | Tắt | Tắt |

Đây là đầu báo chân chiều của driver: xanh nối IN1, đỏ nối IN2, mỗi màu qua
điện trở330Ω; cathode chung xuống GND. Không dùng riêng LED mắc PWMA/PWMB
để kết luận chạy/dừng, vì PWM của một kênh coast có thể HIGH liên tục.
Độ sáng LED không thể hiện tốc độ/PWM. STBY và PWM vẫn được đo riêng bằng
logic analyzer; lỗi STBY/PWM không được phản ánh đầy đủ bởi LED chiều.
Nếu đổi kênh hoặc INVERT sau này, màu/nhãn phải đối chiếu lại.

Không đổi firmware/GPIO vật lý để thêm hai LED demo, không upload board.
LED STATUS ở phía phải là đèn nháy đôi riêng, không đại diện bánh nào.

## Chạy lại

1. Build project chính bằng `pio run -e esp32dev`.
2. Mở **thư mục này** trong cửa sổ VS Code riêng.
3. Command Palette → **Wokwi: Start Simulator**, không dùng Upload.
4. Trong **Wokwi Terminal**, gửi STATUS. Phải có
   `FW=motor-bench-led2-v2-10s`, MOTOR=READY, ARMED=0, RUN=0.
5. ARM rồi LEFT 50: timeout **10000 ms (10 giây)** rồi dừng và DISARM.
   STATUS/HELP không gia hạn timeout. Có thể STOP sớm.
6. ARM lại trước RIGHT 50, BOTH 50, SET 100 60, SET -50 50 hoặc RIGHT -50.
   Không tự chạy khi khởi động; sensor/LINE_FOLLOW vẫn khóa.
7. Stop simulator để xuất `wokwi.vcd`. Đây là bản ghi tín hiệu số, không
   phải phép đo tốc độ hoặc dòng motor.

Nếu build mới mà Wokwi vẫn in mã cũ, Stop rồi gọi lại **Wokwi: Start
Simulator từ Command Palette**. Nút Restart có thể chỉ reset firmware đã
nạp trước đó. Sơ đồ LED tại gốc project được giữ nguyên.

## Đầu đo khớp firmware

| Analyzer | GPIO | Tín hiệu |
| --- | --- | --- |
| D0 | 18 | PWMA |
| D1 | 13 | AIN1 |
| D2 | 14 | AIN2 |
| D3 | 19 | PWMB |
| D4 | 21 | BIN1 |
| D5 | 22 | BIN2 |
| D6 | 23 | STBY; điện trở kéo xuống 10 kΩ |
| D7 | 2 | LED nháy đôi |
| GND | GND ESP32 | Mốc chung |

## Đã đo trên bản 500 ms trước khi đổi timeout

Dữ liệu: [results-500ms.json](results-500ms.json), đo từ VCD Wokwi.

- LEFT 50/RIGHT 50: PWM20,000Hz, duty đo19.532%; kênh dừng có
  IN1/IN2 LOW và PWM HIGH (coast).
- BOTH 50: cả hai cùng duty. SET 100 60: duty39.062% và23.438% độc lập.
- SET -50 50, RIGHT -50, BOTH -50: chân chiều đổi đúng theo dấu từng bánh.
- Chín xung timeout (gồm hai xung LEFT 30 được nhập thêm) có STBY HIGH
  khoảng498.8–499.7ms; STOP/DISARM kết thúc xung sớm hơn hạn.
- Startup DISARMED; thiếu ARM bị chặn. Số151 bị từ chối; cuối cùng
  ARMED=0/RUN=0. Đây là quan sát simulator, không phải motor thật.

PWM8bit đo xấp xỉ duty/256 do bộ đếm256 bước; giao diện vẫn0..255.
Giá trị255 được core chuyển thành100%. Duty không phải RPM.

## Kết quả bản hiện tại: timeout 10 giây

[results-10s.json](results-10s.json) ghi đo từ VCD của bản
`motor-bench-led2-v2-10s`:

- LEFT 50: STBY HIGH **9999.4308ms**, rồi xuống LOW; PWM20,000Hz,
  duty19.532%, AIN1/2=HIGH/LOW. Kênh B ở coast.
- Trong thời gian chạy, STATUS cho ARMED=1/RUN=1; sau timeout ARMED=0/RUN=0.
- SET 100 60: hai duty39.062% và23.436%; STOP kết thúc xung trước hạn10s.
- RIGHT -50: BIN1/2=LOW/HIGH, chỉ B có PWM; DISARM kết thúc xung sớm.
- Chuyển SENSOR_TEST dừng/DISARM, ARM trong mode đó bị chặn; trở về
  MOTOR_TEST vẫn ARMED=0/RUN=0.
- GPIO2 có mẫu100ms sáng,100ms tắt,100ms sáng,700ms tắt (đã đo VCD).
- Build ESP32 SUCCESS (17.32s); 26 kịch bản host và14 cấu hình sai đạt.

Các thời gian xung STOP/DISARM trong JSON là tổng thời gian STBY HIGH,
không phải phép đo độ trễ từ lúc người dùng bấm phím. Thời gian timeout
đếm từ lúc xử lý lệnh, sớm hơn lúc chân STBY bật một phần nhỏ mili giây.

Bản ghi thô của hai lần đo được giữ tại `/tmp/hustroy-motor-demo-500ms.vcd`
và `/tmp/hustroy-motor-demo-10s.vcd`; không đưa file ghi lớn vào source tree.
JSON lưu kết quả gọn. Không upload hoặc phát lệnh quay tới ESP32 vật lý
trong phiên mô phỏng này. Không commit/push/merge.

Nguồn: [Wokwi Logic Analyzer/VCD](https://docs.wokwi.com/parts/wokwi-logic-analyzer),
[cấu hình Wokwi VS Code](https://docs.wokwi.com/vscode/project-config).
