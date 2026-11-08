# Tích hợp cảm biến, PID và ba chế độ — HUSTROY-HRC2026

**Cập nhật thử motor vật lý:** mọi chuyển động hiện cần `ARM`; xem thêm
[hướng dẫn ARM/DISARM và thử từng bánh](physical-motor-first-drive.vi.md).
Các kết quả kiểm chứng của pha tích hợp phía dưới là kết quả tại pha đó.

## 1. So sánh hai dự án và nguồn thuật toán

| Phần | HUSTROY trước tích hợp | Line_Follower.zip | Quyết định |
| --- | --- | --- | --- |
| Nền tảng | esp32dev, Arduino-ESP32 3.3.12, platform 8.0.0 | esp32dev, không ghim version | Giữ PlatformIO của HUSTROY |
| Motor | TB6612 thật, LEDC 3.x, signed duty, khóa chân, timeout | Không có | Giữ nguyên driver motor |
| Cảm biến | Chưa có | digitalRead 5 chân nhưng SENSOR_1..5 chưa định nghĩa | Module riêng, tách AO/DO, không đoán phần cứng |
| Vị trí | Chưa có | Trung bình trọng số -2..+2 | Giữ ý tưởng và phép chia float |
| PID | Chưa có | 20/0/0, cộng sai số mỗi vòng, D không chia dt | Giữ P+I+D và gains đầu; thêm giây, giới hạn, anti-windup |
| Hướng/giao lộ | Chưa có | Backup desktop LEFT/RIGHT/FORWARD/INTERSECTION/LOST | Tái dùng ý tưởng; không chép main desktop |
| Thực thi | Serial motor test, LED millis | Tính/in liên tục | Ba chế độ, tick 20 ms, log 250 ms |

Nguồn ban đầu do đồng đội cung cấp; chưa có tên tác giả. Ba tệp thật được
giải nén nguyên byte vào `reference/line_follower/`, có SHA-256 trong README.
Không nhập `.pio`, `.vscode`, `.DS_Store`, binary desktop hoặc temp runner.

Thay đổi thuật toán có chủ đích: giao lộ gồm 11111 hoặc hai cảm biến ngoài
cùng cùng active, theo backup. Nhiều cụm active tách rời, ví dụ 01010, được
coi INVALID thay vì lấy trung bình 0 rồi chạy thẳng. Với một cụm liên tục,
dấu vị trí quyết định LEFT/RIGHT/FORWARD. 01110 vẫn là FORWARD như backup.

## 2. Cấu trúc và chức năng thật

| Tệp | Vai trò / giao diện chính |
| --- | --- |
| include/config.h | GPIO, khóa phần cứng, AO/DO, ngưỡng, cực tính, gains và thời gian |
| include/hardware_config.h | Kiểm tra GPIO/ADC1/trùng motor, sensor, nút và chân dành trước lúc build |
| include/sensors.h + src/sensors.cpp | sensorsInit(), sensorsRead(), analyzeSensorPattern(), interpretSensorRaw() |
| include/pid.h + src/pid.cpp | PidController::reset/update(), forwardCorrectionLimit(), mixForwardSpeeds() |
| include/modes.h + src/modes.cpp | Chọn chế độ, Serial, RUN/STOP, lịch điều khiển và nút tùy chọn |
| include/motor.h + src/motor.cpp | Driver cũ: motorInit(), setMotorSpeeds(), motorStop(), motorIsReady() |
| src/main.cpp | setup, gọi modesUpdate() và LED millis |

Không có PID/cảm biến/Serial chen vào driver motor. Không thêm thư viện ngoài,
không delay trong control loop. Không có HC-SR04P hay Bluetooth hoạt động ở
phiên bản này; chỉ có hằng dành chân tương lai, không khởi tạo ngoại vi giả.

## 3. Cấu hình an toàn hiện tại và trình tự xác nhận

Motor giữ nguyên khóa false và bảy GPIO=-1. Cảm biến cũng khóa false,
SENSOR_INPUT_MODE=UNCONFIGURED và năm chân=-1. Nút mode tắt, chân=-1.
Không tự áp dụng bất kỳ bảng chân đề xuất cũ nào.

Các bản đề xuất chân ở pha trước chỉ dùng để đối chiếu dây. Tên mảng thật
trong firmware tích hợp là `SENSOR_PINS` kiểu std::array; không dùng tên
`LINE_SENSOR_PINS` trong bản config đề xuất cũ để thay thế module mới.

Phương án đang chờ Toàn: motor 18/13/14/19/21/22/23 theo ảnh board; phương án
16/17 trong task chỉ dùng được sau khi xác nhận biến thể và sửa chính sách
GPIO có chủ đích. Driver hiện chặn 16/17. Line dự kiến 32/33/34/35/36, HC
25/26, nút27. Đây vẫn không phải bảng đã được duyệt.

Để đọc thô sau khi xác nhận dây/điện áp, thay các hằng tương ứng, không thêm
trùng định nghĩa. Ví dụ AO với năm chân **đã được Toàn xác nhận**:

```cpp
constexpr bool SENSOR_HARDWARE_ENABLED = true;
constexpr SensorInputMode SENSOR_INPUT_MODE = SensorInputMode::ANALOG_AO;
constexpr std::array<int, SENSOR_COUNT> SENSOR_PINS = {{32,33,34,35,36}};
constexpr bool SENSOR_CALIBRATION_CONFIRMED = false; // giữ false khi đo thô
```

DO dùng `SensorInputMode::DIGITAL_DO`. AO chỉ cho phép ADC1 32/33/34/35/36/39;
không lấy ADC2 cho năm line. GPIO34..39 chỉ input, không có pull-up/pull-down
nội. Năm chân luôn xếp theo **cảm biến vật lý từ trái sang phải nhìn theo
hướng robot tiến**, không theo vị trí chân trên board.

Giá trị 2048 trong ngưỡng là placeholder giữa thang ADC 12 bit 0..4095, không
phải đã đo trên vạch đen. Ngưỡng riêng từng sensor. INVERT_LINE_SENSOR=false
tạm coi HIGH hoặc raw>=ngưỡng là line; true đảo lại. Trước xác nhận hiệu
chỉnh, log có raw/bits dự kiến nhưng VALID=0/INVALID; PID không được dùng chúng.

Sau khi đo nền/vạch và chọn đúng ngưỡng/cực tính mới đặt
SENSOR_CALIBRATION_CONFIRMED=true. Kiểm tra bias/đứt dây thực tế rồi mới đặt
SENSOR_DISCONNECT_BIAS_CONFIRMED=true. Flag này không tự kiểm tra dây điện:
cần điện trở bias phía ngõ vào ESP32 phù hợp và đo lại sau khi lắp để khi
tuột dây không tạo ngẫu nhiên một mẫu line hợp lệ. Với analog, bias có thể
làm đổi số đo nên phải hiệu chỉnh cả mạch cuối cùng.

Một giá trị tĩnh hợp lệ không chứng minh dây còn nối. Firmware dừng với
LOST/INTERSECTION/INVALID, yêu cầu frames mới và RUN có giới hạn thời gian;
nó không thể phát hiện mọi lỗi dây nếu lỗi giả dạng một mẫu hợp lệ.

Điện áp GPIO/AO/ECHO phải phù hợp 3.3 V, không đưa 5 V vào ESP32. ADC ESP32
gốc ở 11 dB đo khoảng 0.15–3.1 V; 3.3 V có thể bão hòa. Xác nhận module có
AO thay vì chỉ DO, đúng VCC và ngân sách dòng rail3V3. HC-SR04P ECHO/TRIG cần
kiểm tra riêng, chưa được đọc bằng firmware này.

VCC TB6612 dùng logic3.3V phù hợp breakout, VM dùng nguồn motor đủ dòng đã
xác nhận, GND chung. Không cấp motor từ GPIO/3V3. Xác nhận điện áp pin đầy,
dòng kẹt motor và khả năng driver/breakout/tản nhiệt, STBY kéo xuống và tụ.
PWM thấp không tự giới hạn dòng kẹt; không xem xung thử500ms là xung đỉnh10ms.
Khi chưa duyệt phần cứng, VM phải ngắt. Duy trì khả năng cắt nguồn trực tiếp.

## 4. Lệnh Serial chính xác

Viết HOA; Enter bằng CR/LF/CRLF. Số tốc độ là số nguyên, hiện -150..150.

| Lệnh | Kết quả |
| --- | --- |
| HELP | In danh sách lệnh |
| STATUS | Mode, RUN, motor/sensor ready, calibration/bias và nguyên nhân dừng |
| ARM | Dừng trước, cho phép lệnh chuyển động khi hardware READY; không quay motor |
| DISARM | Dừng và hủy quyền chuyển động |
| MODE MOTOR_TEST | Dừng trước, chuyển sang thử motor |
| LEFT 30 / LEFT -30 | Chỉ trái tiến/lùi; phải dừng |
| RIGHT 30 / RIGHT -30 | Chỉ phải tiến/lùi; trái dừng |
| BOTH 30 / BOTH -30 | Hai bánh tiến/lùi |
| SET 50 30 / SET -30 30 | Hai giá trị signed duty riêng trong MOTOR_TEST |
| MODE SENSOR_TEST | Dừng trước, chỉ đọc/log cảm biến |
| MODE LINE_FOLLOW | Dừng trước, chưa tự chạy |
| RUN | Chỉ bắt đầu LINE_FOLLOW sau ARM và đủ điều kiện; tick tiếp mới gửi tốc độ |
| STOP | Dừng, giữ mode và giữ quyền ARM hiện tại |
| ! | Dừng + DISARM ngay khi nhận ký tự; Enter trước lệnh mới |

Lệnh sai, số tràn, thừa đối số, ký tự điều khiển hoặc dòng quá63ký tự sẽ dừng + DISARM.
Lệnh motor trong SENSOR_TEST/LINE_FOLLOW bị từ chối và dừng. STOP có đối số
vẫn dừng nhưng báo lỗi. HELP/STATUS không kéo dài thời gian chạy.

MOTOR_TEST hết hạn500ms từ lệnh motor hợp lệ cuối rồi DISARM. LINE_FOLLOW tối đa5000ms
mỗi RUN; RUN lặp lại không gia hạn. Muốn lần thử mới cần STOP/lần thử trước
đã dừng rồi ARM, chờ mẫu hợp lệ, RUN. Tăng giới hạn bằng config khi thử thực tế đã được kiểm soát;
hiện giới hạn build tối đa30000ms cho pha bench, không phải firmware thi đấu
chạy vô hạn. Không có lệnh chỉnh PID/Bluetooth runtime; gains hiện chỉnh ở config.

RUN cần ARM, motor ready, sensor ready, calibration/bias confirmed, ít nhất3mẫu
LINE_DETECTED liên tiếp, mẫu mới trong100ms và khoảng điều chỉnh hợp lệ.
ARM dừng trước và xóa bộ đếm mẫu; chờ ít nhất60ms sau ARM trước RUN (thực hành chờ1giây cho dễ).
Nút nếu được bật và nối đúng sẽ debounce30ms, chỉ chọn mode và dừng + DISARM;
không arm RUN. Giữ nút lúc boot không tự đổi mode, phải thả trước.

## 5. Trạng thái và lịch điều khiển

- LINE_DETECTED: có một cụm line liên tục, cập nhật PID khi RUN đã arm.
- LINE_LOST: 00000 sau chuẩn hóa; dừng + DISARM và reset PID, không tự tìm line.
- INTERSECTION: 11111 hoặc hai sensor biên cùng active; dừng, không tự rẽ.
- INVALID: chưa cấu hình/calibrate, raw sai miền hoặc mẫu nhiều cụm mơ hồ;
  dừng và reset PID.

Mỗi20ms đọc năm sensor và có tối đa một lần điều khiển. Nếu xử lý bị trễ quá
100ms khi đang RUN thì STALE_CONTROL và dừng, không phát một loạt lệnh bù.
Khi dừng hoặc đổi mode reset PID và đếm mẫu hợp lệ lại. Mất line/giao lộ trở
lại bình thường vẫn không tự chạy, cần RUN mới. Diagnostics250ms, không in
mỗi vòngCPU. Timer dùng phép trừ unsigned chịu được millis quay vòng.

Các timeout là phần mềm, phụ thuộc CPU/loop còn chạy; không thay cho khóa
STBY phần cứng và nút/công tắc cắt VM. Không claim hard real-time hoặc bảo vệ
tất cả lỗi điện.

## Bài 1 — TCRT5000 hoạt động thế nào?

Một cảm biến gồm LED hồng ngoại chiếu xuống sàn và phần thu ánh sáng phản
xạ. Mức phản xạ phụ thuộc màu/vật liệu/khoảng cách/ánh sáng, không chỉ chữ
đen/trắng. Module AO cho điện áp thay đổi; ADC đổi điện áp thành số. DO cho
0/1 sau bộ so sánh, thường có biến trở chỉnh ngưỡng. Module khác nhau có thể
cho đen=LOW hoặc đen=HIGH, nên phải đo. Không đoán theo tên TCRT5000.

Hiệu chỉnh từng sensor: ghi raw trên nền trắng rồi vạch đen ở đúng độ cao,
chọn ngưỡng giữa hai nhóm với khoảng cách đủ rõ; ngưỡng từng kênh có thể khác.
Nếu đen thấp hơn trắng thì đặt INVERT_LINE_SENSOR=true với quy tắc hiện tại.
Điều chỉnh lại nếu thay độ cao, nguồn, điện trở bias hoặc môi trường sáng.

## Bài 2 — Trung bình trọng số

Mã đồng đội gán vị trí -2,-1,0,+1,+2 cho sensor trái→phải, cộng vị trí sensor
active rồi chia số active. Sau chuẩn hóa, bit1 luôn nghĩa là sensor thấy line.

| Bits trái→phải | Phép tính | Vị trí / chính sách |
| --- | --- | --- |
| 00100 | 0/1 | 0, FORWARD |
| 01100 | (-1+0)/2 | -0.5, LEFT |
| 00110 | (0+1)/2 | +0.5, RIGHT |
| 00000 | Không có số chia hợp lệ | LOST, dừng |
| 11111 | Trung bình0 nhưng không phải một line thường | INTERSECTION, dừng |

Ví dụ DO đọc11011 mà đen=LOW thì đảo thành00100. Chuỗi hiển thị trong BITS
đã được chuẩn hóa; đừng lấy chuỗi raw ngược cực tính để áp bảng trên.

## Bài 3 — PID chuyển sai số thành lệnh bánh

Error=position: line bên phải → error dương. P phản ứng mức lệch hiện tại.
I tích lũy lệch theo giây, D nhìn tốc độ thay đổi của lệch. Chúng tạo correction:

```text
P = Kp * error
integral += error * dtSeconds
D = Kd * (error - previousError) / dtSeconds
correction = P + Ki * integral + D
left = BASE_SPEED + correction
right = BASE_SPEED - correction
```

Error+0.5, Kp20 → correction+10. Base50 → trái60, phải40, xe có xu hướng quay
sang phải nếu hai bánh đã được định nghĩa tiến đúng. Error-0.5 → trái40,
phải60. INVERT_PID_ERROR đảo chiều điều chỉnh nếu cần; trước khi đảo hãy kiểm
tra thứ tự sensor và chiều motor. Không dùng cờ đảo để che dây cắm nhầm.

Ví dụ I: error0.5 trong0.02s tích lũy0.01 error·s, không phải cộng0.5 mỗi
vòngCPU. Ví dụ D: error tăng0.5 trong0.1s → tốc độ đổi5 đơn vị/s. Lần đầu
D=0 để tránh cú giật từ giá trị trước chưa có. Dễ nhiễu thì phải tune D thận
trọng; gains20/0/0 chỉ là điểm xuất phát của đồng đội.

Tích phân bị chặn ở±2 error·s mặc định; không lưu thêm khi đẩy đầu ra sâu hơn
vào bão hòa. Correction còn bị giới hạn bởi50 và khoảng trống hai bánh; tại
base50/max150, giới hạn50 giữ bánh không đảo chiều trong LINE_FOLLOW.
Manual MOTOR_TEST vẫn cho số âm. Ki/Kd trong bản gốc không có dt nên không
thể chuyển gains khác0 sang bản mới mà coi như đã tune.

## Bài 4 — Dòng dữ liệu thật trong mã

`main.cpp` gọi sensorsInit và modesUpdate. `sensorsRead` lấy raw theo AO/DO,
`interpretSensorRaw` đổi thành bits, `analyzeSensorPattern` tính trạng thái và
vị trí. `modes.cpp` quyết định có được chạy hay cần dừng. Chỉ khi line hợp
lệ và RUN đã arm, `PidController::update` và `mixForwardSpeeds` tạo hai lệnh,
rồi `setMotorSpeeds(left,right)` chuyển chúng thành IN/PWM/STBY trên TB6612.

Header `.h` giống danh sách thao tác; `.cpp` chứa cách thực hiện. GPIO là chân
tín hiệu, PWM là bật/tắt nhanh, cầu H trong TB6612 đổi chiều điện áp motor.
Chương trình không đo RPM hoặc dòng motor. motorStop chọn coast, bánh có thể
còn trôi. Độ lớn duty không phải tốc độ vật lý tuyến tính.

## Bài 5 — Thực hành trên macOS, VS Code, PlatformIO

1. File→Open Folder vào HUSTROY-HRC2026, mở PlatformIO Core CLI. Chạy
   `pio run -e esp32dev`, mong SUCCESS. Nếu pio không có trên PATH, dùng
   `~/.platformio/penv/bin/pio` hoặc CLI của extension. Không bỏ static_assert
   để ép chân xung đột build được.
2. Chạy `python3 tests/run_host_tests.py`. Test dùng compiler C++ trên Mac,
   API giả và config tạm; không chạm board. Nếu thiếu compiler, cần bộ công
   cụ dòng lệnh Xcode phù hợp. Đây không phải test điện.
3. Với VM tắt và board/nguồn đã kiểm tra, nối cáp USB truyền dữ liệu. Dùng
   `pio device list`, upload chỉ khi sẵn sàng bằng
   `pio run -e esp32dev -t upload` (lần tích hợp này chưa upload).
4. Monitor: `pio device monitor -b 115200 --echo --eol LF`; nhiều cổng thì
   thêm `-p /dev/cu.TEN_CONG_THUC`. HELP/STATUS trước. Chữ rác→baud; im lặng→
   cổng/cáp/Enter. Thoát Ctrl+C trước upload lại.
5. Đọc sensor trước: sau khi Toàn duyệt GPIO/điện áp/AO-DO, chỉ bật phần đọc
   sensor và để calibration false, VM vẫn tắt. MODE SENSOR_TEST in RAW mỗi
  250ms. Di chuyển từng sensor qua trắng/đen. Raw=-1 cả năm→chưa cấu hình;
   số đứng yên/loạn→kiểm tra AO/DO, GND, nguồn, khoảng cách và bias; số sát
  4095→kiểm tra mức điện áp/bão hòa. Không chữa 5V vào ADC bằng đổi threshold.
6. Chọn ngưỡng/cực tính, xác nhận calibration, rồi thử các mẫu bảng bên trên.
   Mẫu00110 phải cho+0.5 và RIGHT. Nếu ra ngược, kiểm tra sensor thứ tự và
   cực tính trước. Thử rút tín hiệu an toàn khi VM tắt để kiểm tra bias.
7. Sau khi Toàn duyệt motor/nguồn, cấu hình motor đúng, nạp khi VM tắt. Kê
   bánh và cố định thân, MODE MOTOR_TEST, cấp VM, ARM rồi thử LEFT30 riêng; ARM lại rồi RIGHT30
   riêng; mong TIMEOUT500ms và chỉ một bánh. Thử STOP/!, chờ đứng yên trước
   số âm. Không quay→kiểm tra dây/VM/STBY trước tăng duty; nóng/reset→ngắt VM.
8. Thử PID trên giá đỡ: để sensor trên mẫu line hợp lệ, MODE LINE_FOLLOW,
   ARM, chờ1s rồi RUN. Nếu BLOCKED→STATUS và các flag/mẫu; không ép bỏ khóa. Với
   line bên phải phải có L>R nếu convention đúng. Mẫu00000/11111 phải dừng;
   trả line về vẫn dừng, cần RUN. Mỗi RUN tối đa5s, STOP/! hoạt động mọi lúc.
9. Chỉ khi mọi bước trên đúng mới thử rất chậm trên sân trống, chuẩn bị cắt
   nguồn, thời gian ngắn. Theo dõi dao động và tune P trước; I/D bắt đầu0.
   Chưa có bằng chứng robot bám line tốt chỉ từ build hoặc test trên giá đỡ.

## Kiểm chứng và giới hạn

Kết quả chạy trên mã tích hợp:

| Kiểm tra | Kết quả |
| --- | --- |
| `pio run -e esp32dev` với config thật đang khóa | SUCCESS; Arduino-ESP32 3.3.12; RAM22,464 byte, Flash303,243 byte |
| Build/link ESP32 với phần cứng bật trong project tạm | AO và DO đều SUCCESS; chỉ là fixture kiểm tra, không upload và không đổi config thật |
| `python3 tests/run_host_tests.py` | 23 lượt chạy chương trình đạt, 14 cấu hình sai bị chặn biên dịch; có kiểm tra cả32pattern |
| Hash tham chiếu ZIP | Cả ba tệp được so byte và khớp bản gốc |
| Wokwi firmware tích hợp | LED tắt/sáng, HELP/STATUS, chọn cả ba mode, raw chưa cấu hình INVALID, RUN/LEFT bị khóa, BOTH151 báo lỗi, STOP hoạt động |
| AddressSanitizer bổ sung trên macOS | Không chạy được: công cụ kẹt trong khởi tạo shadow memory/malloc trước main; đã lấy stack sample và dừng tiến trình, không coi là kiểm tra bộ nhớ đã qua |
| UBSan (`undefined,float-cast-overflow`, không recover) | Ba lượt logic/system-follow/serial đạt, không có chẩn đoán lỗi |
| Phần cứng thật | Chưa upload/đo/thử motor, cảm biến, nút hoặc đường chạy |

Lỗi build ban đầu do tên enum ANALOG trùng macro Arduino đã sửa bằng tên
ANALOG_AO/DIGITAL_DO. Một lỗi fixture host đọc nhầm config gốc qua sibling
include đã sửa bằng cách sao toàn bộ header vào project tạm. Không tắt guard
để làm test qua. Không có lỗi build/test còn tồn đọng ở các lệnh trên.

Wokwi gốc giữ nguyên ESP32+LED GPIO2 và đường firmware/ELF. Nó dùng được để
kiểm tra boot/Serial/mode với hardware khóa. Không có xác nhận điện/motor/
TCRT5000 thật, không nhập chip mô phỏng tùy chế. Test máy tính gồm cả32pattern,
AO/DO/inversion/calibration, PID dt/NaN/giới hạn, clamp motor, transitions,
timeout/LOST/intersection/stale, button debounce và negative GPIO configs.

HC-SR04P và Bluetooth chưa triển khai, không có tránh vật cản hoặc tune không
dây. Nút có code nhưng tắt mặc định và chưa thử vật lý. Tuning hiện sửa config
và build lại. Tham khảo tài liệu chính thức:

- [Espressif ADC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html)
- [Espressif GPIO](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html)
- [Espressif LEDC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
