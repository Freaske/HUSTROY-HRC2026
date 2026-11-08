# Đề xuất đấu dây ESP32–TB6612FNG, chờ duyệt

## Phạm vi và giả định

Đã đọc lại `platformio.ini`, `include/config.h`, `src/motor.cpp`, `src/main.cpp`,
`diagram.json`, `wokwi.toml`. Cấu hình hiện tại: `board = esp32dev`, Arduino,
platform `espressif32@8.0.0`; core cài trên máy là Arduino-ESP32 3.3.12.
Wokwi dùng `board-esp32-devkit-c-v4`, LED ngoài qua điện trở 220 Ω ở GPIO2.

**Chưa xác định được board thật.** Phương án dưới đây giả định board phát triển
ESP32-WROOM-32/ESP32 gốc có đưa các GPIO này ra hàng chân. Không áp dụng nguyên
xi cho ESP32-S3/C3 hoặc board có ngoại vi gắn sẵn chiếm chân. GPIO25 nghĩa là
chân có nhãn IO25/25 trên board, không phải chân thứ 25 khi đếm trên hàng cắm.

Đây là phương án kỹ thuật để Toàn và người dùng duyệt. Chưa thay bất kỳ định
nghĩa GPIO/khóa nào trong firmware; chưa nối hoặc thử phần cứng. Kênh A = trái,
B = phải chỉ là quy ước tạm, cần xác nhận cùng hướng nhìn từ phía sau robot.

## A. Bảng đấu dây hoàn chỉnh

| Chân TB6612FNG | Chân ESP32 / kết nối | Chức năng |
| --- | --- | --- |
| PWMA | GPIO25 | PWM motor trái, kênh A |
| AIN1 | GPIO26 | Chọn chiều motor trái, ngõ 1 |
| AIN2 | GPIO27 | Chọn chiều motor trái, ngõ 2 |
| PWMB | GPIO18 | PWM motor phải, kênh B |
| BIN1 | GPIO19 | Chọn chiều motor phải, ngõ 1 |
| BIN2 | GPIO23 | Chọn chiều motor phải, ngõ 2 |
| STBY | GPIO4; thêm điện trở 10 kΩ từ STBY xuống GND | Cho phép driver / khóa khi reset |
| VCC | 3V3 của board ESP32 | Nguồn logic 3.3 V |
| VM | Cực dương nguồn motor đã kiểm tra, qua cầu chì và công tắc | Nguồn công suất motor |
| GND | GND chung: ESP32, driver, cực âm nguồn motor | Mốc tín hiệu và đường hồi dòng |
| AO1/AO2 | Hai dây motor trái | Đầu ra công suất kênh A |
| BO1/BO2 | Hai dây motor phải | Đầu ra công suất kênh B |

Đấu theo **nhãn in trên breakout**, không đoán thứ tự chân từ hình của một
breakout khác. Nếu dùng IC rời thay vì breakout, cần nối tất cả chân VM/GND/
PGND và các chân đầu ra trùng tên theo datasheet; bảng này dành cho breakout.

### Dành chân cho các phần còn lại

| Thiết bị / tín hiệu | GPIO dành riêng | Ghi chú |
| --- | --- | --- |
| TCRT5000 số 1, ngoài cùng bên trái, AO | 32 — ADC1_CH4 | Tín hiệu analog, không phải DO |
| TCRT5000 số 2, AO | 33 — ADC1_CH5 | Analog |
| TCRT5000 số 3, giữa, AO | 34 — ADC1_CH6 | Chỉ nhận tín hiệu |
| TCRT5000 số 4, AO | 35 — ADC1_CH7 | Chỉ nhận tín hiệu |
| TCRT5000 số 5, ngoài cùng bên phải, AO | 36 / VP — ADC1_CH0 | Chỉ nhận tín hiệu |
| HC-SR04P TRIG | 21 | ESP32 gửi xung, mức 3.3 V phải phù hợp module |
| HC-SR04P ECHO | 22 | ESP32 nhận; chỉ nối trực tiếp nếu mức điện áp đã xác nhận an toàn |
| Serial USB: TX0 / RX0 | 1 / 3 | Giữ nguyên cho nạp và Serial Monitor |
| LED hiện có | 2 | Không dùng lại cho motor/cảm biến |
| Bluetooth tích hợp ESP32 | Không cần GPIO tín hiệu riêng | Không cần module Bluetooth ngoài cho phương án này |
| Dự phòng analog | 39 / VN — ADC1_CH3 | Chỉ nhận tín hiệu |
| Dự phòng digital | 13, 14 | Có chức năng JTAG; phải xem lại nếu dùng debug JTAG |

GPIO21/22 thường dùng cho I2C, nhưng ở phương án này đã dành cho siêu âm.
Không được đồng thời nối bus I2C vào hai chân đó. Nếu sau này cần UART ngoài,
có thể xem xét remap sang 13/14 sau khi kiểm tra JTAG và các module; chưa triển
khai hoặc chiếm hai chân đó trong pha này.

### Vì sao chọn được các GPIO này?

Bảy chân motor đều xuất digital được và tương thích chính sách `isSafeMotorPin()`
trong driver hiện tại. LEDC của ESP32 có thể định tuyến PWM tới GPIO25 và GPIO18;
không cần chân riêng có nhãn PWM. Tất cả đều tách khỏi năm chân ADC1 đã dành.

Tránh GPIO0/2/5/12/15 cho dây motor vì là chân chọn chế độ lúc boot (strapping).
Giữ nguyên GPIO2 cho LED đã có; không mắc thêm tải làm thay đổi mức boot.
Tránh GPIO6–11 cho flash, để trống 16/17 để không xung đột các biến thể có
PSRAM. Không dùng GPIO34–39 làm đầu ra. Không dùng GPIO1/3 để bảo toàn UART0.

Năm đầu vào line đều ở ADC1. Điều này tránh tranh chấp ADC2 với Wi-Fi nếu sau
này dùng thêm Wi-Fi; Bluetooth tích hợp không cần cặp chân TX/RX ngoài.
GPIO34/35/36 không có điện trở kéo lên/kéo xuống nội, nên mạch cảm biến phải
tự tạo tín hiệu rõ ràng, không để đầu vào hở. GPIO37/38 không được coi là chân
có thể cắm trên board WROOM phổ biến.

Đã đối chiếu với [GPIO ESP32](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html)
và [board DevKitC V4](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html).

## B. Sơ đồ kết nối

```text
ESP32-WROOM dev board                 TB6612FNG breakout

IO25 -------------------------------- PWMA
IO26 -------------------------------- AIN1       AO1 ---- dây 1 motor TRÁI
IO27 -------------------------------- AIN2       AO2 ---- dây 2 motor TRÁI

IO18 -------------------------------- PWMB
IO19 -------------------------------- BIN1       BO1 ---- dây 1 motor PHẢI
IO23 -------------------------------- BIN2       BO2 ---- dây 2 motor PHẢI

3V3 --------------------------------- VCC

IO4 ------------------------+-------- STBY
                            |
                          [10 kΩ]     Giữ STBY LOW lúc ESP32 reset
                            |
GND ------------------------+-------- GND
                            |
Nguồn motor (-) ------------+         GND CHUNG

Nguồn motor (+) -- cầu chì -- công tắc -- VM
                   (giá trị chọn sau khi biết motor/nguồn)

USB máy tính ------ ESP32             Chỉ cấp ESP32 bằng USB khi thử bàn
```

Đây là sơ đồ chức năng, **không phải thứ tự chân vật lý trên board**.
Đường từ nguồn motor (-) nối tới cùng nút GND của ba phần.
Hai dây motor không nối riêng vào 3V3, VM hoặc GND: chúng chỉ nối vào cặp AO
hoặc BO, vì driver phải đảo được điện áp giữa hai đầu motor.

Đường motor (+), dây motor và đường hồi motor về cực (-) cần chịu dòng tải.
Cho dòng motor đi trực tiếp qua driver về cực âm nguồn bằng dây phù hợp;
không cho dòng này chạy xuyên qua đường GND mảnh của ESP32/breadboard.
Dây GND tín hiệu ESP32 nối tới điểm chung gần driver/nguồn; giữ dây tín hiệu
ngắn và tách khỏi dây motor nếu có thể. Không coi rail breadboard là đường
cấp motor đủ dòng khi chưa biết thông số.

## C. Hiểu từng kết nối

GPIO là chân gửi/nhận tín hiệu. HIGH gần 3.3 V, LOW gần 0 V. Các GPIO motor
chỉ mang lệnh; phần điện làm quay motor đến từ VM, qua driver, tới AO/BO.

PWM là bật/tắt lặp nhanh. Duty là tỷ lệ thời gian bật, không phải RPM. Các
chân IN1/IN2 chọn chiều, còn STBY giống công tắc cho phép chung hai motor.

| Dây | Loại và lý do nối như đề xuất | Nếu nối sai |
| --- | --- | --- |
| GPIO25 → PWMA | Tín hiệu PWM trái; GPIO25 xuất được LEDC, không chiếm ADC1 | Nhầm GPIO thì lệnh trái không đúng; nhầm PWMB thì đổi bánh được điều khiển |
| GPIO26 → AIN1 | Tín hiệu chiều A; chân xuất hợp lệ, ở cùng nhóm dây trái | Đổi chéo AIN1/AIN2 làm đảo chiều; nối nhầm kênh có thể khiến chạy sai bánh |
| GPIO27 → AIN2 | Tín hiệu chiều A thứ hai; không chiếm chân cảm biến | Bỏ hở/nối sai khiến không chọn đúng chiều hoặc trạng thái dừng |
| GPIO18 → PWMB | Tín hiệu PWM phải; chân xuất LEDC, không dùng flash/boot | Nhầm PWM khiến mất điều tốc hoặc điều khiển nhầm bánh |
| GPIO19 → BIN1 | Tín hiệu chiều B; chân xuất hợp lệ | Đổi chéo với BIN2 làm đảo chiều phải |
| GPIO23 → BIN2 | Tín hiệu chiều B thứ hai; không trùng module khác | Bỏ hở/nối sai làm hành vi kênh phải không đúng |
| GPIO4 → STBY | Tín hiệu cho phép; GPIO4 không nằm trong danh sách strap của ESP32 gốc | Nối GND cố định thì không chạy; nối HIGH cố định làm mất khóa chung và tăng rủi ro lúc boot |
| STBY → 10 kΩ → GND | Điện trở kéo xuống, giữ lệnh tắt khi GPIO đang thả nổi | Bỏ điện trở hoặc breakout kéo lên có thể làm mất trạng thái tắt khi reset; không thay điện trở bằng dây nối thẳng |
| ESP32 3V3 → VCC | Nguồn logic, dòng nhỏ; giúp tín hiệu ESP32 tương thích | Nối nhầm VM/battery vào VCC có thể hỏng driver; cấp VCC 5 V thì HIGH 3.3 V không còn được bảo đảm |
| Nguồn motor (+) → VM | Nguồn công suất; motor cần dòng lớn, không lấy từ GPIO | Quá áp/đảo cực có thể hỏng driver và motor; chưa biết pin thì chưa được đóng công tắc |
| ESP32 GND ↔ driver GND ↔ nguồn (-) | Mốc điện áp chung và đường hồi dòng | Thiếu chung GND làm tín hiệu không có cùng mốc, dễ chạy sai; nối nhầm cực (+) có thể gây chập |
| AO1/AO2 → motor trái | Hai dây công suất từ cầu H A | Đổi hai dây làm đảo chiều; nối chập đầu ra có thể gây quá dòng |
| BO1/BO2 → motor phải | Hai dây công suất từ cầu H B | Đổi hai dây làm đảo chiều; nối lẫn A/B làm mất điều khiển độc lập |

**GND chung không có nghĩa nối chung các cực dương.** Mỗi thiết bị đo điện áp
tín hiệu so với GND của nó. Nối GND cho chúng cùng mốc giống như dùng chung
vạch số 0 trên thước; VCC và VM vẫn có nhiệm vụ và có thể có điện áp khác nhau.

GPIO không thể cấp motor: motor có dòng khởi động/kẹt cao, tạo nhiễu và điện
áp phản kháng. Driver chịu dòng công suất; GPIO chỉ điều khiển. Nếu đụng
VM/AO/BO vào GPIO, điện áp hoặc dòng lớn có thể làm hỏng ESP32.

## D. Kiểm tra điện và trình tự cấp nguồn

### Logic 3.3 V

TB6612FNG cho VCC làm việc 2.7–5.5 V. Dùng VCC = 3.3 V thì ngưỡng HIGH tối
thiểu 0.7×3.3 = 2.31 V, phù hợp đầu ra ESP32. Nếu VCC = 5 V, ngưỡng đó thành
3.5 V, nên không bảo đảm tín hiệu HIGH 3.3 V được nhận đúng. Không cần bộ
đổi mức giữa ESP32 và TB6612 khi VCC đúng 3.3 V và breakout không có mạch
khác làm thay đổi mức. Không đưa 5 V vào GPIO ESP32.

### Nguồn motor và dòng kẹt

- Dải VM làm việc của IC: 2.5–13.5 V. Giới hạn tuyệt đối 15 V không phải
  điện áp nên cấp. Phải thỏa cả motor, breakout và nguồn, tính cả pin đầy
  và điện áp tăng do năng lượng trả về khi dừng/đảo chiều.
- Bảng operating range ghi tối đa 1.0 A/kênh khi VM ≥ 4.5 V; 0.4 A/kênh khi
  2.5 V ≤ VM < 4.5 V (điều kiện không PWM). Mức 1.2 A trong bảng absolute
  maximum không nên được coi là dòng chạy thiết kế mặc định.
- 3.2 A chỉ là đỉnh cho một xung 10 ms theo datasheet. Xung thử firmware
  500 ms **không** được dùng để hợp thức hóa dòng kẹt 3.2 A.
- Cần biết dòng kẹt mỗi motor tại điện áp định dùng và tổng tải hai kênh.
  Nếu vượt khả năng driver/breakout hoặc tản nhiệt, cần đổi driver hay có
  giới hạn dòng được thiết kế phù hợp. TB6612 không có chức năng đặt giới hạn
  dòng motor bằng phần mềm; MAX_SPEED chỉ giới hạn duty, không giới hạn ampere.
- Chưa chọn điện áp pin hoặc giá trị cầu chì khi chưa có các số trên. Ví dụ
  pin Li-ion 2S có thể đầy 8.4 V; chữ 7.4 V trên nhãn không đủ để kết luận
  an toàn cho motor 6 V. Duty thấp không loại bỏ đỉnh điện áp VM trên motor.
- ESP32 thử bàn dùng USB riêng, nguồn motor chỉ nối (+) vào VM và (-) vào
  GND chung. Không đưa battery (+) vào 3V3/5V/VIN hoặc nối nguồn 5 V ngoài
  song song USB khi chưa kiểm tra sơ đồ cấp nguồn của board.
- Kiểm tra tụ lọc trên breakout. Datasheet minh họa 10 µF song song 100 nF
  gần VM–GND và VCC–GND. Nếu chưa có, bổ sung theo tài liệu breakout; điện
  áp chịu của tụ phải lớn hơn điện áp lớn nhất dự kiến, tụ phân cực đúng chiều.

Đối chiếu [datasheet TB6612FNG](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660&prodName=TB6612FNG),
đặc biệt trang operating range, logic input và typical application.

### HC-SR04P: không đoán điện áp ECHO từ chữ P

GPIO21 dành cho TRIG (lệnh gửi đi), GPIO22 dành cho ECHO (xung nhận về).
Chưa có model/sơ đồ module cụ thể trong repo; vì vậy chưa xác nhận được nguồn
hoặc biên độ ECHO của module trên tay. Không nối ECHO 5 V trực tiếp vào ESP32.

Phương án ưu tiên: chọn đúng module được tài liệu của nhà sản xuất xác nhận
chạy 3.3 V và giao tiếp logic 3.3 V, cấp VCC module bằng rail 3.3 V đủ dòng,
GND chung; TRIG từ GPIO21, ECHO tới GPIO22. Không mặc định rail 3V3 của board
đủ cho cả ESP32, năm LED hồng ngoại và siêu âm: cần tính ngân sách dòng.

Nếu module có ECHO mức 5 V, đề xuất buffer một chiều **SN74LVC1G17**:
ECHO → A; Y → GPIO22; VCC buffer → 3V3; GND → GND chung; tụ 100 nF tại VCC–GND.
TI xác nhận đầu vào chịu 5.5 V và đầu ra theo nguồn 3.3 V. Nhờ vậy không
đưa ECHO 5 V trực tiếp vào ESP32. Toàn chọn breakout/package đúng và xác nhận
ngưỡng ECHO thực tế, không đoán vị trí chân IC. Nếu cấp module bằng 5 V,
TRIG 3.3 V cũng phải đạt ngưỡng vào của module; nếu không, cần đổi mức hướng
TRIG riêng. Không dùng buffer digital này cho đầu ra analog của cảm biến line.

Đối chiếu [TI SN74LVC1G17](https://www.ti.com/lit/ds/symlink/sn74lvc1g17.pdf).
Điện trở chia áp cũng có thể dùng cho ECHO 5 V đã xác nhận, nhưng không lắp
mù cho ECHO vốn 3.3 V vì có thể hạ HIGH xuống dưới ngưỡng ESP32.

### Năm cảm biến analog

Dùng AO của module có đầu ra analog, không dùng nhầm DO chỉ có 0/1. Cảm biến
TCRT5000 rời không phải module hoàn chỉnh: cần điện trở cho LED và mạch tải
phototransistor. Nếu dùng module, phải xác nhận sơ đồ và điện áp AO tối đa.
Đầu vào ESP32 phải ở miền an toàn 0–3.3 V; không nối AO 5 V trực tiếp.
ADC ESP32 gốc ở attenuation 11 dB đo được khoảng 0.15–3.1 V theo tài liệu,
nên tín hiệu gần 3.3 V có thể bão hòa dù chưa vượt nguồn logic. Mạch analog
và hiệu chỉnh sẽ được thiết kế ở pha cảm biến, chưa triển khai trong pha này.

Đối chiếu [ADC Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html)
và [TCRT5000 Vishay](https://www.vishay.com/docs/83760/tcrt5000.pdf).

### Checklist trước khi đóng nguồn VM

- [ ] Đúng ESP32-WROOM-32 dev board, tất cả chân xuất hiện trên board; không
  nhầm GPIO với số thứ tự chân. Đúng breakout và pinout của nó.
- [ ] Toàn duyệt bảng GPIO, kênh trái/phải, nguồn, dòng kẹt và tản nhiệt.
- [ ] Tắt mọi nguồn khi cắm/sửa dây; không chập 3V3–GND hoặc VM–GND.
- [ ] Đúng cực; VCC 3.3 V, VM theo thông số đã duyệt, tụ/cầu chì phù hợp.
- [ ] GND chung, đường hồi motor đủ dòng; không lấy dòng motor qua ESP32.
- [ ] STBY có điện trở kéo xuống 10 kΩ; kiểm tra breakout không kéo lên/tie HIGH.
- [ ] ESP32 reset/đang nạp vẫn giữ driver standby. Nguồn logic ổn định trước VM;
  tắt VM trước khi rút USB/nguồn logic.
- [ ] Firmware vẫn khóa đến khi được duyệt. Khi mở khóa sau này, boot vẫn dừng,
  chỉ quay khi có lệnh; STOP là coast, bánh có thể trôi.
- [ ] Bánh rời bàn, thân cố định, chuẩn bị ngắt VM; thử LEFT 30 một xung 500 ms,
  dừng và kiểm tra rồi mới thử bên phải. Không giữ trục để thử dòng kẹt.
- [ ] Chưa nối ECHO/AO vào ESP32 khi chưa xác nhận biên độ và mạch đổi mức.

## E. Định nghĩa đề xuất, CHƯA áp dụng

Đây là phần thay thế cho các hằng tương ứng trong `config.h` **sau khi duyệt**,
không phải đoạn dán thêm làm trùng định nghĩa. Giữ khóa false trong giai đoạn
đối chiếu; chỉ bật true ở bước thử phần cứng đã được cho phép.

```cpp
constexpr bool MOTOR_HARDWARE_ENABLED = false;
constexpr int MOTOR_PWMA_PIN = 25;
constexpr int MOTOR_AIN1_PIN = 26;
constexpr int MOTOR_AIN2_PIN = 27;
constexpr int MOTOR_PWMB_PIN = 18;
constexpr int MOTOR_BIN1_PIN = 19;
constexpr int MOTOR_BIN2_PIN = 23;
constexpr int MOTOR_STBY_PIN = 4;

constexpr bool LEFT_MOTOR_IS_CHANNEL_A = true; // tạm: A trái, B phải
constexpr bool INVERT_LEFT_MOTOR = false;     // kiểm tra chiều trên bàn sau
constexpr bool INVERT_RIGHT_MOTOR = false;

// Giữ các tham số đã có; chưa phải giá trị hiệu chỉnh motor.
constexpr uint8_t MOTOR_PWM_BITS = 8;
constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 20000;
constexpr int MAX_SPEED = 150;
constexpr int BASE_SPEED = 50;
constexpr unsigned long MOTOR_TEST_TIMEOUT_MS = 500;

// Chỉ là đề xuất dành chân, chưa có logic cảm biến.
constexpr int LINE_SENSOR_PINS[5] = {32, 33, 34, 35, 36};
constexpr int ULTRASONIC_TRIG_PIN = 21;
constexpr int ULTRASONIC_ECHO_PIN = 22;
```

GPIO của firmware đang dùng thực tế vẫn là `-1`; không đổi file config hiện tại.

Đã kiểm tra bằng công cụ: 17 GPIO đang dùng/dành trước không trùng nhau; năm
chân line đều thuộc ADC1; mapping motor đề xuất vượt qua kiểm tra biên dịch
`static_assert` của driver khi dùng một bản config tạm với khóa bật. Không
upload hoặc thay config thật. Ví dụ Wokwi đã kiểm tra JSON, tên linh kiện,
mapping đầu đo và đường dẫn firmware/ELF; chưa chạy mô phỏng riêng này.

## F. Tin nhắn gửi Toàn

> Toàn ơi, mình đề xuất cho ESP32-WROOM-32: TB6612 PWMA=25, AIN1=26, AIN2=27,
> PWMB=18, BIN1=19, BIN2=23, STBY=4 kèm kéo xuống GND 10 kΩ; VCC=3V3,
> VM dùng nguồn motor riêng và GND chung. Tạm A=trái, B=phải. Dành AO line
> 32/33/34/35/36, siêu âm TRIG=21/ECHO=22, giữ LED2 và Serial1/3. Bạn kiểm tra
> giúp model board/breakout, xung đột chân, điện áp pin khi đầy, thông số motor
> và dòng kẹt, ECHO/AO có cần đổi mức không nhé. Firmware vẫn khóa; chờ bạn
> duyệt và người dùng đồng ý rồi mới cập nhật mapping và thử bánh kê khỏi bàn.

## Ví dụ Wokwi riêng và giới hạn

Xem thư mục `docs/wokwi-motor-signals/`. Nó chỉ nối ESP32, LED và logic analyzer
theo bảng đề xuất. Không thêm chip TB6612 giả hoặc motor DC không được hỗ trợ.
Sơ đồ và cấu hình Wokwi gốc của project không đổi. Với firmware hiện đang khóa,
không có PWM motor để quan sát; điều đó là đúng. Sơ đồ riêng không tự mở khóa
hoặc áp dụng GPIO vào firmware.

Không dùng mô phỏng tín hiệu số để xác nhận dòng motor, nhiệt, điện áp nguồn,
RPM hay mức ECHO thật. [Phần cứng Wokwi](https://docs.wokwi.com/getting-started/supported-hardware)
và [logic analyzer](https://docs.wokwi.com/parts/wokwi-logic-analyzer).
