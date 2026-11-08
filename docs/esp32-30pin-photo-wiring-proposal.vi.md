# Phương án theo ảnh board 30 chân — CHƯA DUYỆT

Phương án mới này dùng ảnh `IMG_8849.HEIC` của người dùng và thay thế bảng GPIO
đề xuất trước đó cho lần xem xét hiện tại. Không trộn hai bảng đấu dây.
Không thay `include/config.h`, driver, main, PlatformIO hoặc Wokwi đang dùng.

## A. Nhận diện board và nhãn chân

Đã xem ảnh gốc chuyển sang PNG và phóng to hàng chân. Quan sát được 30 vị trí
header (15 mỗi bên), cổng USB-C bên trái ảnh, nút EN/BOOT, module có vỏ kim
loại ghi `ESP-32`, Wi-Fi/Bluetooth và hai dãy nhãn dưới đây.

**Khả năng cao là board tương thích ESP32 DevKit V1 30 chân dùng họ ESP32 gốc,
dạng module WROOM.** Không xác nhận được hãng, revision, chip USB-UART, sơ đồ
VIN hoặc mã module đầy đủ. Ảnh không cho thấy dòng `ESP-WROOM-32` rõ ràng trên
vỏ, nên chưa xác nhận được mã module WROOM cụ thể hoặc tình trạng PSRAM chỉ
từ hình dáng. Tên Wokwi DevKitC V4 không chứng minh board thật là V4.

Giữ ảnh đúng chiều: USB-C bên trái, anten ở bên phải. Số vị trí dưới đây chỉ
để chỉ chỗ trên ảnh, **không dùng làm số GPIO trong chương trình**.

| Vị trí từ trái sang phải | Hàng trên ảnh | GPIO / nguồn | Hàng dưới ảnh | GPIO / nguồn |
| --- | --- | --- | --- | --- |
| 1 | VIN (chữ ở đầu hàng khó đọc hơn) | Chân nguồn, chưa xác nhận sơ đồ | 3V3 | Nguồn 3.3 V |
| 2 | GND | Mass | GND | Mass |
| 3 | D13 | 13 | D15 | 15 |
| 4 | D12 | 12 | D2 | 2 |
| 5 | D14 | 14 | D4 | 4 |
| 6 | D27 | 27 | D16 | 16 |
| 7 | D26 | 26 | D17 | 17 |
| 8 | D25 | 25 | D5 | 5 |
| 9 | D33 | 33 | D18 | 18 |
| 10 | D32 | 32 | D19 | 19 |
| 11 | D35 | 35 | D21 | 21 |
| 12 | D34 | 34 | RX0 | 3 |
| 13 | VN | 39 | TX0 | 1 |
| 14 | VP | 36 | D22 | 22 |
| 15 | EN | Enable/reset, không phải GPIO | D23 | 23 |

Trên board này D18 tương ứng GPIO18; trong C++ dùng `18`, không dùng vị trí
thứ 9 và không mặc định tên `D18` được Arduino định nghĩa. VN ở gần VP là
GPIO39, không nhầm với VIN ở đầu hàng gần USB.

Ảnh cho thấy các pad kim loại/thiếc và có vẻ có đầu chân, nhưng không thấy
rõ hàng pin nhô phía dưới. Chưa xác nhận được có cần hàn header hoặc mối hàn
có tốt không. Nếu đã có chân hàn chắc: dùng jumper cái hoặc breadboard phù
hợp, không cần hàn thêm. Nếu chỉ có lỗ trống: cần hàn hai thanh 1×15 chân,
bước 2.54 mm, hoặc dùng đầu nối hàn phù hợp. Không cắm dây lỏng vào lỗ để thử.

## B. Bảng motor đề xuất đã chỉnh

Đề xuất gốc của người dùng có AIN1=D16 và AIN2=D17. Hai chân này **không sai
trên WROOM đã xác nhận**, nhưng có thể dành cho bộ nhớ trên biến thể khác.
Tài liệu Espressif phân biệt WROOM/SOLO-1 với WROVER ở điểm này. Driver hiện
tại cũng cố ý không cho 16/17 đi qua bộ kiểm tra GPIO. Vì ảnh chưa chốt được
module, đề xuất dùng **D13/D14** thay thế, không nới khóa an toàn phần mềm.
Đổi lại, hai chân JTAG 13/14 sẽ bị motor chiếm; không dùng đồng thời JTAG.

Các dây sau đều là kết nối theo **tên chân**, không mô tả thứ tự header TB6612.
Kênh A=trái, B=phải là tạm; trái/phải tính khi nhìn theo hướng robot tiến.

| Chân linh kiện | Nhãn ESP32 trong ảnh | GPIO / kết nối nguồn | Mục đích |
| --- | --- | --- | --- |
| TB PWMA | D18 — hàng dưới, vị trí 9 | GPIO18 | PWM motor trái A |
| TB AIN1 | D13 — hàng trên, vị trí 3 | GPIO13 | Chọn chiều A, tín hiệu 1 |
| TB AIN2 | D14 — hàng trên, vị trí 5 | GPIO14 | Chọn chiều A, tín hiệu 2 |
| TB PWMB | D19 — hàng dưới, vị trí 10 | GPIO19 | PWM motor phải B |
| TB BIN1 | D21 — hàng dưới, vị trí 11 | GPIO21 | Chọn chiều B, tín hiệu 1 |
| TB BIN2 | D22 — hàng dưới, vị trí 14 | GPIO22 | Chọn chiều B, tín hiệu 2 |
| TB STBY | D23 — hàng dưới, vị trí 15 | GPIO23; kéo xuống GND qua 10 kΩ | Khóa/cho phép cả hai cầu H |
| TB VCC | 3V3 — hàng dưới, vị trí 1 | Nguồn logic 3.3 V | Nuôi phần nhận lệnh |
| TB GND | GND — hàng dưới, vị trí 2, hoặc GND hàng trên | GND chung | Mốc logic và đường hồi dòng |
| TB VM | Không nối chân nguồn ESP32 | Nguồn motor (+), qua cầu chì/công tắc | Nuôi công suất motor |
| TB AO1/AO2 | Không nối vào ESP32 | Hai dây motor trái | Đầu ra công suất A |
| TB BO1/BO2 | Không nối vào ESP32 | Hai dây motor phải | Đầu ra công suất B |
| Nguồn motor (-) | GND | Cùng điểm GND driver/ESP32 | Đường hồi motor và mốc chung |
| Điện trở 10 kΩ | Nhánh tại STBY/D23, đầu kia GND | Không mắc nối tiếp dây D23–STBY | Giữ tắt lúc reset |

## C. Dành chân cho các phần khác

| Linh kiện / tín hiệu | Nhãn trong ảnh | GPIO / đặc tính | Mục đích |
| --- | --- | --- | --- |
| Line 1 AO | D32 — trên, vị trí 10 | 32 / ADC1_CH4 | Analog, ngoài trái |
| Line 2 AO | D33 — trên, vị trí 9 | 33 / ADC1_CH5 | Analog |
| Line 3 AO | D34 — trên, vị trí 12 | 34 / ADC1_CH6, chỉ input | Analog giữa |
| Line 4 AO | D35 — trên, vị trí 11 | 35 / ADC1_CH7, chỉ input | Analog |
| Line 5 AO | VP — trên, vị trí 14 | 36 / ADC1_CH0, chỉ input | Analog ngoài phải |
| HC-SR04P TRIG | D25 — trên, vị trí 8 | GPIO25, output | ESP32 gửi xung |
| HC-SR04P ECHO | D26 — trên, vị trí 7 | GPIO26, input; cần mức an toàn | ESP32 nhận xung |
| Nút mode, một tiếp điểm | D27 — trên, vị trí 6 | GPIO27; dự kiến INPUT_PULLUP sau này | Nhấn = LOW |
| Nút mode, tiếp điểm còn lại | GND | GND | Nút thường hở nối xuống GND |
| Serial RX0/TX0 | RX0/TX0 — dưới, vị trí 12/13 | 3/1 | Giữ cho USB serial/nạp |
| LED của firmware/Wokwi | D2 — dưới, vị trí 4 | GPIO2 | Không dùng lại |
| Bluetooth tích hợp | Không thêm dây tín hiệu | Radio trong ESP32 | Không chiếm TX/RX riêng |

Với TCRT5000: phải xác nhận module có AO, không dùng DO thay AO. TCRT5000 rời
cần mạch LED/phototransistor riêng. Ngõ AO phải nằm trong miền an toàn ESP32,
không tự nối AO 5 V. Năm chân đều ADC1; dùng input-only cho đầu vào là đúng.
GPIO34/35/36 không có pull-up/pull-down nội; mạch cảm biến phải tạo tín hiệu
xác định. ADC11 dB ESP32 gốc đo khoảng 0.15–3.1 V; mức gần 3.3 V có thể bão hòa.

HC-SR04P: chưa xác nhận nguồn và mức ECHO của module thực tế. Chỉ nối trực
tiếp ECHO→D26 khi tài liệu đúng module hoặc phép đo xung xác nhận mức phù hợp
3.3 V. Không coi kết quả đồng hồ DC là đủ để biết đỉnh của xung ngắn. Nếu
ECHO 5 V, cần đổi mức trước D26; ví dụ buffer SN74LVC1G17 cấp 3.3 V (ECHO→A,
Y→D26, GND chung, tụ 100 nF tại VCC–GND). Không dùng buffer digital này cho AO.
TRIG 3.3 V cũng phải đạt ngưỡng vào module. Không đoán từ chữ P.

Với nút mode: dây nối đúng là tiếp điểm thường hở giữa D27 và GND. Sau này
cấu hình pull-up sẽ giữ HIGH khi thả, LOW khi nhấn; chưa thêm logic nút lúc này.
Nếu nút có bốn chân, phải xác định hai nhóm tiếp điểm; không đoán theo vị trí.

Có 18 GPIO dùng/dành trước không trùng. Còn D4 dùng digital và VN/GPIO39 làm
input/ADC1 dự phòng. D16/D17 để dành đến khi xác nhận module. D5/D12/D15 là
strap, không coi như chân dự phòng có thể dùng tùy ý. 13/14 đã dùng motor,
21/22 đã dùng motor: không đồng thời dùng JTAG hoặc I2C mặc định ở đó.

## D. Nguồn và an toàn

- VCC driver = 3V3; VM = nguồn công suất motor. Hai nguồn này không đồng nghĩa.
  VIN là nhãn chân nguồn của board, không phải GPIO và không phải 3V3. Chưa
  có sơ đồ revision này nên không suy ra VIN chịu 7–12 V hoặc nối USB thế nào.
- Thử bàn: ESP32 chỉ nhận nguồn qua USB-C; không cấp thêm VIN/3V3 từ nguồn
  khác. Sau khi kiểm tra không chập, cấp USB khi VM đang ngắt và đo rail 3V3.
  Cổng USB-C không có nghĩa được cấp trực tiếp điện áp cao/nguồn PD tùy ý.
- Nhánh motor: nguồn (+) → cầu chì → công tắc → VM. Nguồn (-) → GND driver.
  ESP32 GND nối cùng điểm chung gần driver/nguồn. Không nối nguồn (+) motor
  vào VIN/3V3; không nối riêng đầu motor vào GND hoặc VM, chỉ vào cặp AO/BO.
- GND chung là cùng mốc 0 V, không phải nối chung cực dương. Dòng motor đi
  trực tiếp qua dây nguồn/driver đủ tiết diện về nguồn, không xuyên qua rail
  hoặc dây GND mảnh của ESP32. GPIO chỉ mang tín hiệu, không cấp motor.
- Cấu trúc USB cho ESP32 + nguồn motor riêng + GND chung là phương án thử
  đề xuất. Chưa phê duyệt phương án dùng cùng bộ pin cấp ESP32 qua VIN; cần
  đúng sơ đồ board, bộ ổn áp và đánh giá cấp nguồn/backfeed trước.
- TB6612 VCC làm việc 2.7–5.5 V. Ở VCC=3.3 V, ngưỡng HIGH tối thiểu 2.31 V,
  phù hợp đầu ra ESP32; VCC=5 V đòi HIGH tối thiểu 3.5 V nên không bảo đảm.
- VM làm việc IC 2.5–13.5 V, nhưng motor/breakout có thể cho phép thấp hơn.
  Phải biết điện áp pin khi đầy, độ sụt áp/tăng áp và định mức motor. 15 V
  tuyệt đối không phải điện áp thiết kế nên cấp. Không chọn điện áp pin ở đây.
- Bảng operating range: tối đa 1.0 A/kênh khi VM≥4.5 V, 0.4 A/kênh khi
  2.5≤VM<4.5 V (điều kiện không PWM); còn phụ thuộc tản nhiệt/breakout.
  3.2 A chỉ là một xung 10 ms, không áp dụng xung thử 500 ms của firmware.
  Chưa biết dòng kẹt motor nên chưa thể kết luận driver/nguồn chịu được.
  MAX_SPEED giới hạn duty, không giới hạn dòng điện. Không giữ trục để thử kẹt.
- Kiểm tra tụ lọc gần VCC/VM theo breakout; datasheet minh họa 10 µF + 100 nF
  mỗi rail. Tụ đủ điện áp chịu và đúng cực. Cầu chì/giới hạn dòng chọn sau khi
  biết nguồn, dây, driver và motor; không đoán giá trị.
- STBY đề xuất kéo xuống 10 kΩ. IC đã có kéo xuống khoảng 200 kΩ nhưng
  điện trở ngoài mạnh hơn. Nếu breakout đã kéo xuống phù hợp có thể không
  cần thêm; nếu có kéo lên/tie HIGH phải xem lại. Không nối STBY cố định HIGH.
- Giữ VM tắt khi reset/nạp/sửa dây; tắt VM trước khi rút USB. Kê hai bánh
  rời mặt bàn và cố định thân trước lệnh thử. Không đảo chiều khi còn quay.
- Rail 3V3 có thể phải nuôi nhiều module; cần tính tổng dòng năm cảm biến IR,
  siêu âm và ESP32, không mặc định ổn áp board đủ cho tất cả.

## E. So với firmware và cấu hình đề xuất

Đã đọc config, header, motor, main và PlatformIO. Driver hiện dùng LEDC core
3.x, PWM 8 bit/20 kHz; main chỉ chạy khi có lệnh Serial, timeout 500 ms, có
STOP/!. MAX_SPEED=150 chưa hiệu chỉnh. Trái/phải độc lập, có cờ đảo chiều.

GPIO16/17 bị `isSafeMotorPin()` chặn có chủ ý để tránh các biến thể có PSRAM.
Đó là chính sách thận trọng của phần mềm, không phải khẳng định rằng WROOM
không dùng được chúng. Bảng sửa D13/D14 đi qua kiểm tra hiện có mà không sửa
validator. Nếu sau này muốn giữ D16/D17, cần xác minh module và sửa chính sách
GPIO có chủ đích; không chỉ đổi config rồi xóa static_assert.

Đề xuất được lưu riêng ở `esp32-30pin-photo-config.h.txt`, không được build tự
động. Config thật vẫn khóa và bảy GPIO=-1. Chỉ thay các hằng tương ứng sau
khi duyệt; không dán thêm định nghĩa trùng tên.

Kết quả thực tế: `pio run -e esp32dev` SUCCESS (0.66 s), Arduino-ESP32 3.3.12;
RAM 22,320 byte, Flash 296,019 byte. Kiểm tra biên dịch với bản config tạm:
mapping gốc 16/17 bị chặn đúng dự kiến, mapping sửa 13/14 đạt. Không upload,
merge, push, thử motor hoặc đo GPIO/nguồn trên board thật.

## F. Sơ đồ theo tên chân, không giả định header TB6612

```text
ESP32 trong ảnh                         Chân có tên trên TB6612
D18 (dưới 9) -------------------------- PWMA
D13 (trên 3) -------------------------- AIN1       AO1 -- motor trái -- AO2
D14 (trên 5) -------------------------- AIN2
D19 (dưới 10) ------------------------- PWMB
D21 (dưới 11) ------------------------- BIN1       BO1 -- motor phải -- BO2
D22 (dưới 14) ------------------------- BIN2
3V3 (dưới 1) -------------------------- VCC

D23 (dưới 15) -----------------+------- STBY
                               |
                             [10 kΩ]
                               |
GND ---------------------------+------- GND
                               |
Nguồn motor (-) ---------------+
Nguồn motor (+) --- cầu chì --- công tắc --- VM

USB-C ---> cấp nguồn ESP32 khi thử bàn; VIN không nối trong phương án này.
```

Thứ tự chân trên breakout TB6612 chưa biết vì chưa có ảnh của nó. Sơ đồ chỉ
thể hiện quan hệ điện, không thể hiện vị trí chân bên trái/phải driver.

## G. Hiểu và thử sau khi được duyệt

ESP32 gửi HIGH/LOW (gần 3.3/0 V) tới IN1/IN2 để chọn chiều, gửi PWM qua PWMA/
PWMB để điều chỉnh duty. PWM bật/tắt nhanh, không phải một điện áp DC giảm
đều; mức 30 là 30/255, không có nghĩa 30 RPM. Tải/ma sát/pin ảnh hưởng tốc độ.

Với quy ước A=trái và các cờ INVERT=false:

- `setMotorSpeeds(30, 0)`: trái A nhận duty 30 ở D18, AIN1=D13 HIGH,
  AIN2=D14 LOW; phải B ở coast; STBY=D23 HIGH khi chạy.
- `setMotorSpeeds(0, 30)`: phải B nhận duty 30 ở D19, BIN1=D21 HIGH,
  BIN2=D22 LOW; trái A coast.
- `setMotorSpeeds(-30, 0)`: đổi trạng thái D13/D14 cho trái quay ngược.
- `motorStop()` hạ D23 LOW, cả hai coast. Bánh có thể tiếp tục trôi do quán
  tính, không phải phanh tức thời. Driver hạ STBY trong lúc cập nhật chiều/duty.
- Khi một bánh dừng còn bánh kia chạy, PWM bánh dừng có thể HIGH liên tục
  với hai IN LOW: đó là lựa chọn coast, không phải chạy hết công suất.

Sau khi phần cứng và firmware được duyệt: VM tắt → kiểm tra dây/nguồn →
USB-C cho ESP32 → kiểm tra boot STOPPED/READY → kê bánh → bật VM → gửi riêng
`LEFT 30` → chờ TIMEOUT → gửi `RIGHT 30` riêng. Không dán chuỗi lệnh. Dùng
STOP+Enter hoặc ! để dừng, và ! cần Enter trước lệnh tiếp. Chờ bánh đứng yên
rồi mới thử số âm. Nếu lệnh 30 chưa quay, kiểm tra nguồn/dây/tải trước khi tăng.
Hiện tại khóa còn false nên lệnh chạy chỉ được phép trả BLOCKED.

## H. Cần Toàn xác nhận

Mã module và revision board, hàng pin/mối hàn, ảnh/pinout breakout TB6612,
cách nối STBY và tụ, nguồn motor/điện áp khi đầy/dòng, định mức/dòng kẹt motor,
A/B trái phải, module TCRT có AO hay DO và biên độ, nguồn/logic TRIG/ECHO
HC-SR04P, tổng dòng rail 3V3. Chưa xác nhận thì chưa cấp VM.

## I. Đúng ba việc tiếp theo

1. Nhờ Toàn xem bảng sửa D13/D14, kiểm tra mặt dưới/header board và gửi ảnh
   breakout cùng thông số pin/motor/cảm biến.
2. Hoàn tất kiểm tra nguồn/mức điện áp/GND/STBY với VM đang ngắt; duyệt một
   bảng đấu dây duy nhất trước khi cắm nguồn công suất.
3. Sau khi bạn duyệt mapping, mới cập nhật firmware có kiểm soát và thử từng
   bánh bằng LEFT 30/RIGHT 30 với bánh kê cao; hiện chưa upload hay mở khóa.

## Nguồn đối chiếu

- [Espressif DevKitC: GPIO16/17 tùy module, không dùng làm sơ đồ bố trí cho board trong ảnh](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html)
- [Espressif GPIO: ADC1, input-only, strapping](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html)
- [Espressif WROOM-32 datasheet](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.pdf)
- [Espressif ADC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html)
- [Toshiba TB6612FNG](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660&prodName=TB6612FNG)
- [TI SN74LVC1G17](https://www.ti.com/product/SN74LVC1G17)
