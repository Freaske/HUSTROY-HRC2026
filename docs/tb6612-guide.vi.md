# TB6612FNG — từ chương trình đến thử động cơ an toàn

## Trạng thái của dự án

Đây là chương trình thử động cơ bằng Serial, chưa phải chương trình dò line.
Chưa có sơ đồ chân được xác nhận trong repo. Bảy chân động cơ đều là `-1`
(chưa gán) và `MOTOR_HARDWARE_ENABLED = false`. Chương trình vẫn build,
đọc lệnh và nháy LED GPIO2, nhưng không cấu hình hoặc ghi bất kỳ GPIO động cơ
hay PWM nào. Lệnh chạy trả về `BLOCKED`.

Đã giữ cách dùng tốc độ có dấu trong phần bạn đang sửa ở `motor.h` và cách
chạy không chặn trong `main.cpp`. Việc tự gửi tốc độ mỗi giây được thay bằng
lệnh chủ động để tránh tự quay bánh khi cấp điện. Bản diff trước khi sửa được
lưu cục bộ tại `/tmp/HUSTROY-HRC2026-before-tb6612.patch` (tệp tạm, không phải
bản sao lưu lâu dài).

Nhánh làm việc: `feature/tb6612-motor-driver`. Chưa commit, push, tạo PR hoặc
merge. Gợi ý commit: `feat(motor): add guarded TB6612 driver and timed serial tests`.
Các thay đổi chia theo chức năng: config + driver; giao diện Serial; kiểm tra
trên máy tính + tài liệu. Không thêm thư viện Arduino ngoài.

## Bài 1 — Motor driver là gì?

ESP32 là bộ điều khiển: nó đưa ra tín hiệu quyết định robot làm gì.
Động cơ là thiết bị tiêu thụ công suất: cần dòng điện lớn hơn nhiều so với
khả năng chân tín hiệu ESP32. Khi khởi động hoặc bị kẹt, dòng động cơ còn tăng;
động cơ cũng tạo nhiễu điện. Nối động cơ trực tiếp vào GPIO có thể làm hỏng ESP32.

Motor driver là mạch nhận lệnh nhỏ từ ESP32 và đóng/cắt đường cấp điện lớn hơn
từ nguồn động cơ. TB6612FNG chứa hai bộ điều khiển động cơ DC chổi than, gọi
là kênh A và kênh B. Mỗi kênh có một **cầu H**: có thể hình dung là bốn công
tắc điện tử bố trí quanh động cơ. Đóng các cặp công tắc thích hợp làm dòng điện
đi theo chiều này hoặc chiều kia, nên trục đổi chiều quay. IC tự điều khiển
các công tắc; bạn chỉ cần gửi tín hiệu chọn chiều và PWM.

Nguồn riêng cho động cơ giúp dòng lớn và nhiễu không đi qua đường cấp nguồn
yếu của ESP32. Có thể dùng các nhánh nguồn phù hợp từ cùng một bộ pin nếu
được thiết kế đúng; không nhất thiết phải dùng hai bộ pin độc lập. GND của
các phần vẫn phải nối chung để tín hiệu có cùng mốc điện áp.

| Chân trên driver | Vai trò | Thông tin còn cần |
| --- | --- | --- |
| VM | Nguồn công suất động cơ | Nguồn nào, điện áp đầy pin, đủ dòng không |
| VCC | Nguồn phần logic nhận lệnh | Xác nhận breakout cho phép dùng 3.3 V |
| GND | Mốc điện áp chung | Nối ESP32, driver và cực âm nguồn động cơ |
| STBY | Khóa chung hai kênh; LOW = standby | GPIO từ Toàn, kiểm tra điện trở kéo trên breakout |
| PWMA, PWMB | Hai tín hiệu PWM riêng | Hai GPIO từ Toàn |
| AIN1, AIN2 | Chọn chiều kênh A | Hai GPIO từ Toàn |
| BIN1, BIN2 | Chọn chiều kênh B | Hai GPIO từ Toàn |
| AO1/AO2 | Hai dây ra động cơ A | Không nối vào GPIO |
| BO1/BO2 | Hai dây ra động cơ B | Không nối vào GPIO |

AO1/AO2 và BO1/BO2 là đầu ra công suất, **không phải chân GPIO cần gán số**.
Quy ước hiện tại là trái = A (`LEFT_MOTOR_IS_CHANNEL_A = true`), nhưng đây chỉ
là quy ước phần mềm đang chờ xác nhận. Nếu trái thực tế là B, đổi thành `false`.

### Kiểm tra phần cứng trước khi bật khóa phần mềm

- Xác nhận đúng model ESP32, sơ đồ chân của board và đúng breakout TB6612FNG.
  Tên `esp32dev` trong PlatformIO không chứng minh board trên bàn là model đó.
- Xác nhận điện áp định mức và **dòng kẹt trục (stall current)** của từng motor.
  Motor màu vàng không đủ để xác định thông số. Dòng kẹt là dòng khi motor
  được cấp điện nhưng không quay được; không tự giữ trục để đo nếu chưa có
  quy trình giới hạn dòng phù hợp.
- Chọn VM theo motor, nguồn và breakout. Dải làm việc IC là VM 2.5–13.5 V,
  VCC 2.7–5.5 V; đây không phải lời khuyên cấp 13.5 V cho motor vàng.
  Datasheet còn giới hạn dòng theo điện áp và tản nhiệt. Không dùng số đỉnh
  3.2 A làm dòng liên tục cho mỗi motor; con số này chỉ áp dụng xung rất ngắn.
  Cần đọc thông số breakout và kiểm tra tổng dòng hai kênh.
  PWM thấp không làm VM quá cao trở nên an toàn: xung cấp motor vẫn có biên
  độ gần VM. Không dùng duty thấp để thay cho việc chọn đúng điện áp nguồn.
- Dùng VCC 3.3 V nếu breakout hỗ trợ, để nhận trực tiếp mức logic 3.3 V của
  ESP32. Nếu VCC = 5 V, ngưỡng HIGH tối thiểu 0.7×VCC là 3.5 V: tín hiệu
  3.3 V không được đảm bảo là HIGH. Không đưa mức 5 V vào GPIO ESP32.
- Cấp motor từ nhánh nguồn đủ dòng qua VM. **Không cấp motor từ GPIO,
  chân 3V3 hoặc dựa vào đường USB/5V trên board ESP32 để nuôi motor.**
- Nối chung GND. Kiểm tra đúng cực, tránh chập; kiểm tra tụ lọc gần driver
  theo tài liệu breakout, đường dây và phương án giới hạn dòng/cầu chì phù hợp.
- STBY phải do ESP32 điều khiển, không nối cố định HIGH. Kiểm tra breakout
  không kéo STBY lên VCC; cân nhắc điện trở kéo xuống khoảng 10 kΩ để giữ
  standby khi reset/đang nạp. Firmware chưa chạy lúc boot nên không thay thế
  được trạng thái khóa bằng phần cứng. Khi chưa cấu hình chân, rút nguồn VM.
- Kê robot để **hai bánh rời mặt bàn**, cố định thân, tránh tay/dây ở bánh.
  Chuẩn bị cách ngắt nguồn VM. Thử từng xung ngắn ở mức thấp; chờ bánh đứng
  yên trước khi đảo chiều. Nếu nguồn sụt, ESP32 reset hoặc driver nóng, ngắt
  nguồn và kiểm tra, không tăng PWM để cố khắc phục.

Các giới hạn điện và bảng trạng thái được đối chiếu từ
[datasheet Toshiba](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660&prodName=TB6612FNG).

## Bài 2 — ESP32 ra lệnh cho motor như thế nào?

**GPIO** là chân tín hiệu có thể được lập trình làm đầu vào hoặc đầu ra.
Trong driver này, các chân cần thiết đều là đầu ra. **LOW** gần 0 V và
**HIGH** gần 3.3 V trên ESP32; đó là tín hiệu điều khiển, không phải nguồn motor.

**PWM** là bật/tắt tín hiệu lặp lại rất nhanh. **Duty cycle** là tỷ lệ thời gian
bật trong một chu kỳ. Ví dụ, một chu kỳ dài 50 micro giây, bật 10 micro giây
và tắt 40 micro giây thì duty = 20%. Driver nhận PWM và điều khiển công suất
vào động cơ. Tần số 20,000 Hz nghĩa là 20,000 chu kỳ mỗi giây, mỗi chu kỳ 50 µs.

Độ phân giải **8 bit** cung cấp 256 giá trị lệnh, từ 0 đến 255:

| Độ lớn lệnh | Duty xấp xỉ | Ý nghĩa trong giao diện |
| --- | --- | --- |
| 0 | Trạng thái dừng riêng | Không chạy motor |
| 30 | 11.8% | Xung thử khởi đầu, có thể chưa đủ thắng ma sát |
| 50 | 19.6% | Tăng nhẹ sau khi kiểm tra |
| 128 | 50.2% | Khoảng nửa duty |
| 150 | 58.8% | Giới hạn phần mềm hiện tại, chưa hiệu chỉnh |
| 255 | 100% | Toàn thang 8 bit; hiện bị MAX_SPEED chặn |

Trong mã, `+50` nghĩa là tiến với độ lớn 50; `-50` nghĩa là lùi với cùng độ lớn.
Dấu chọn IN1/IN2; độ lớn chọn PWM. Khi motor quay, tiến dùng IN1 HIGH/IN2 LOW,
lùi dùng IN1 LOW/IN2 HIGH, trước khi áp dụng cấu hình đảo chiều.

PWM **không phải RPM** (số vòng quay mỗi phút). `50` không có nghĩa 50 RPM,
và duty 50% không bảo đảm tốc độ bằng nửa tối đa. Pin yếu, tải, ma sát, hộp số,
đặc tính motor và sụt áp đều ảnh hưởng. Hai bánh cùng lệnh cũng có thể quay lệch.

### Dừng thả trôi và phanh điện khác nhau

`motorStop()` hạ STBY LOW: hai cầu H về trạng thái trở kháng cao (hiểu đơn giản
là ngắt đường kéo motor), bánh giảm tốc theo quán tính. Đây là **coast**.
**Phanh điện** dùng mạch tạo lực cản điện để giảm tốc nhanh hơn. Pha này chưa
cung cấp hàm phanh chủ động.

Khi một bánh chạy còn bánh kia dừng, STBY phải HIGH cho kênh đang chạy.
Kênh đứng yên đặt IN1=LOW, IN2=LOW, PWM=HIGH để chọn coast theo datasheet.
Vì thế PWM đo ở kênh đứng yên có thể là 255: đó là mã chọn trạng thái dừng,
không phải yêu cầu motor chạy hết công suất. MAX_SPEED giới hạn **lệnh chạy**.
Chỉ đặt duty về 0 trong lúc driver hoạt động có thể chọn phanh, nên không
thể hiểu chung rằng PWM=0 luôn là thả trôi.

## Bài 3 — Hiểu từng tệp mã nguồn

### `include/config.h`: bảng cấu hình chung

```cpp
constexpr bool MOTOR_HARDWARE_ENABLED = false;
constexpr int MOTOR_PWMA_PIN = -1; // chờ Toàn
constexpr uint8_t MOTOR_PWM_BITS = 8;
constexpr unsigned long MOTOR_TEST_TIMEOUT_MS = 500;
```

`constexpr` là giá trị cố định khi biên dịch. Chỉ thay GPIO khi đã có bảng chân
được xác nhận. Điền đủ bảy GPIO, xác nhận kênh trái/phải rồi mới bật khóa và build
lại. `MAX_SPEED=150`, `BASE_SPEED=50` được giữ từ dự án; không coi là đã cân
chỉnh. `BASE_SPEED` không được dùng để tự chạy motor trong pha thử này.

`INVERT_LEFT_MOTOR` và `INVERT_RIGHT_MOTOR` đảo dấu lệnh từng bánh. Hai motor
gắn đối xứng có thể cần quay trục khác chiều để cùng đưa xe tiến. Thử từng
bánh, STOP, chờ đứng yên, tắt VM rồi đổi cấu hình và nạp lại nếu cần.

### `include/motor.h`: danh sách thao tác mà phần khác được gọi

```cpp
void motorInit();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void motorForward(int speed);
void motorStop();
bool motorIsReady();
```

Header giống tờ danh sách nút bấm: cho biết tên hàm và dữ liệu cần đưa vào.
Chưa cần biết cách tạo tín hiệu điện khi viết phần PID sau này.

### `src/motor.cpp`: thực hiện các thao tác đó

`motorInit()` kiểm tra khóa và GPIO, giữ STBY LOW, thiết lập các chân và gắn
hai bộ phát PWM. Không gọi chuyển động khi khởi động. Gọi lại hàm này chỉ
dừng motor, không cấp phát thêm kênh PWM. Gọi các hàm trước init cũng không
được phép chạm chân motor.

```cpp
ledcAttach(MOTOR_PWMA_PIN, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_BITS);
ledcWrite(MOTOR_PWMA_PIN, duty);
```

**LEDC** là phần cứng ESP32 tạo PWM tự động; tên có chữ LED nhưng vẫn dùng cho
driver motor. Đã đọc trực tiếp core cài trên máy: Arduino-ESP32 **3.3.12**.
Hàm ghi nhận số GPIO, không phải số channel kiểu core 2.x. `platformio.ini`
ghim nền tảng đang dùng là `espressif32@8.0.0`; đổi sang core 2.x sẽ có lỗi
biên dịch rõ ràng thay vì âm thầm gọi API khác nghĩa. Tham khảo
[API Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html).

Để đặt LOW trước khi chuyển chân thành đầu ra, phần khởi tạo dùng
`gpio_set_level()` của ESP32. Core 3.x không cho dùng `digitalWrite()` như
cách nạp sẵn mức trước `pinMode()` trong một số ví dụ Arduino khác.

`setMotorSpeeds()` kẹp từng lệnh vào `[-MAX_SPEED, MAX_SPEED]`, áp dụng đảo chiều,
chọn kênh A/B, hạ STBY trong lúc đổi tín hiệu rồi mới bật lại. Cách kẹp trước
khi đổi dấu cũng tránh lỗi với giá trị số nguyên âm nhỏ nhất. Kẹp là giới hạn
bảo vệ ở cấp driver; giao diện Serial sẽ từ chối lệnh vượt giới hạn để bạn
biết đã nhập sai.

`motorForward()` dùng độ lớn đối số cho cả hai bánh tiến. Muốn lùi hãy dùng
`setMotorSpeeds(-30, -30)`, không dùng `motorForward(-30)` vì hàm đó vẫn tiến.
`motorStop()` ngắt cầu H bằng STBY trước khi xóa duty và chân chiều.
`motorIsReady()` báo khóa đã mở, chân hợp lệ và PWM khởi tạo thành công.

`static_assert` là kiểm tra lúc build: khi khóa bật, chân `-1`, trùng nhau,
chân LED, UART, flash/PSRAM, chân boot hoặc chân chỉ đọc bị chặn. Danh sách
được phép là chính sách thận trọng cho ESP32 gốc, **không phải sơ đồ đấu dây**.
Board ESP32 khác phải được kiểm tra lại. Hệ thống không thể phát hiện bạn cắm
nhầm dây, nguồn sai hay module hỏng chỉ bằng kiểm tra phần mềm.

Nếu `ledcAttach()` hoặc `ledcWrite()` thất bại, giữ STBY LOW, báo lỗi, khóa driver
đến khi reset sau khi sửa nguyên nhân. Không tự chạy lại. Khóa phần mềm lúc
chưa cấu hình không bảo đảm trạng thái điện của breakout đang nối sai;
nguồn VM và trạng thái STBY phần cứng vẫn phải được kiểm tra.

### `src/main.cpp`: giao tiếp với người thử

`setup()` chạy một lần lúc bật/reset: mở Serial 115200, khởi tạo driver và LED,
in trạng thái dừng. `loop()` lặp liên tục: kiểm tra timeout, nhận từng ký tự,
xử lý dòng lệnh, kiểm tra timeout lần nữa và nháy LED mỗi 500 ms.

```cpp
if (testRunning && millis() - testStartedAt >= MOTOR_TEST_TIMEOUT_MS) {
    stopTest();
}
```

`millis()` là số mili giây từ khi khởi động. Phép trừ số không dấu tiếp tục
làm việc khi bộ đếm quay vòng. Không dùng `delay()` hay hàm đọc Serial chờ đủ
dòng, vì khi chờ thì STOP và các cảm biến sau này sẽ phản ứng chậm.

Bộ đệm lệnh cố định tối đa 47 ký tự và chỉ đọc tối đa 64 byte mỗi vòng để
luồng dữ liệu dài không chiếm toàn bộ thời gian. Lệnh sai, số quá lớn, ký tự
điều khiển hoặc dòng quá dài đều dừng motor; dòng lỗi bị bỏ đến Enter nếu cần.
HELP/STATUS không gia hạn thời gian chạy. Chỉ một lệnh chuyển động hợp lệ mới
bắt đầu xung 500 ms mới. Timeout nằm trong ứng dụng thử `main.cpp`; bản thân
driver không áp timeout cho mọi lệnh tương lai của PID.

Đây là timeout phần mềm, phụ thuộc `loop()` còn hoạt động, không phải bộ cắt
nguồn độc lập. Có sai số nhỏ do lịch chạy và thời gian xuất Serial. Không
gửi lệnh chạy liên tục tự động trong lần thử đầu.

## Bài 4 — Thử trên macOS + VS Code + PlatformIO

### Cú pháp chính xác

Viết HOA, cách nhau bằng dấu cách, nhấn Enter (CR, LF hoặc CRLF đều nhận).

| Lệnh | Kết quả khi driver đã được mở khóa hợp lệ |
| --- | --- |
| `HELP` | In hướng dẫn |
| `STATUS` | In READY hoặc DISABLED/FAULT, trạng thái thử, giới hạn và timeout |
| `LEFT 30` | Trái tiến 30, phải dừng |
| `LEFT -30` | Trái lùi 30, phải dừng |
| `RIGHT 30` | Phải tiến 30, trái dừng |
| `RIGHT -30` | Phải lùi 30, trái dừng |
| `BOTH 30` | Cả hai tiến 30 |
| `BOTH -30` | Cả hai lùi 30 |
| `BOTH 0` | Cả hai dừng; LEFT 0/RIGHT 0 cũng dừng cả hai |
| `STOP` | Dừng ngay khi nhận hết dòng; không chờ hết xung |
| `!` | Dừng ngay khi nhận ký tự, không cần Enter; nhấn Enter trước lệnh tiếp |

Khoảng số hợp lệ hiện là -150..150. `REVERSE 50` không có trong giao diện;
dùng `BOTH -50`. `BOTH 151`, `LEFT abc`, `LEFT 3.5`, lệnh thường hoặc thừa đối
số sẽ bị từ chối và dừng. `STOP thua` vẫn dừng nhưng báo sai cú pháp.

### Các bước thực hành

1. **Mở dự án:** VS Code → File → Open Folder → `HUSTROY-HRC2026`, chọn thư mục
   chứa `platformio.ini`. Không tạo project mới. Nếu không thấy PlatformIO,
   kiểm tra extension PlatformIO IDE và chờ nó khởi tạo.
2. **Build trước, chưa cấp VM:** bấm dấu ✓ PlatformIO hoặc mở PlatformIO Core CLI
   và chạy `pio run -e esp32dev`. Mong đợi `[SUCCESS]`. Nếu shell báo
   `command not found: pio`, mở PlatformIO Core CLI bằng Command Palette hoặc
   dùng `~/.platformio/penv/bin/pio run -e esp32dev`. Nếu báo chân không hợp lệ,
   sửa bảng chân cùng Toàn; đừng xóa kiểm tra an toàn để ép build.
3. **Kết nối ESP32 sau khi kiểm tra dây:** tắt VM, dùng cáp USB có truyền dữ liệu.
   Chạy `pio device list` để tìm cổng `/dev/cu.…`. Nếu không thấy board, thử cáp
   dữ liệu/cổng USB và kiểm tra chip USB-serial đúng model; không cài driver ngẫu nhiên.
4. **Upload:** bấm mũi tên PlatformIO, hoặc `pio run -e esp32dev -t upload`.
   Nếu nhiều cổng, thêm `--upload-port /dev/cu.TEN_CONG_THUC_TE` với tên vừa tìm.
   Mong đợi upload thành công; board vẫn dừng. Nếu không kết nối được, đóng
   monitor khác, kiểm tra board/cổng; nút BOOT chỉ dùng theo hướng dẫn board.
5. **Mở Serial Monitor:** dùng biểu tượng phích cắm PlatformIO hoặc
   `pio device monitor -b 115200 --echo --eol LF`. Với nhiều cổng, thêm
   `-p /dev/cu.TEN_CONG_THUC_TE`. Gõ `HELP` rồi Enter. Nếu chữ rác, kiểm tra
   baud 115200; nếu im lặng, kiểm tra đúng cổng, Enter và nhấn EN/reset khi VM
   đang tắt. `MOTOR DISABLED` là kết quả đúng trước khi có mapping. `STATUS`
   vẫn dùng được, lệnh chạy báo `BLOCKED`.
6. **Thử bánh trái:** chỉ sau khi Toàn xác nhận toàn bộ checklist, cập nhật đủ
   GPIO/kênh/khóa, build và upload lại với VM tắt. Kê bánh, kiểm tra READY,
   rồi cấp VM đúng thông số. Gửi một lệnh `LEFT 30`. Mong đợi chỉ bánh trái
   nhận lệnh, log `TEST: left=30 right=0 ...` và sau khoảng 0.5 giây có
   `TIMEOUT: motors stopped (coast).` Nếu chưa quay, 30 có thể chưa thắng ma
   sát: STOP, kiểm tra VM/GND/STBY và dây trước khi cân nhắc `LEFT 40`/`LEFT 50`.
   Không mặc định rằng tăng tới 150 sẽ giải quyết được lỗi.
7. **Thử bánh phải:** gửi `RIGHT 30` riêng, chờ timeout. Chỉ phải nhận lệnh;
   nếu trái chạy thì kiểm tra A/B và `LEFT_MOTOR_IS_CHANNEL_A`. Nếu chiều sai,
   STOP, chờ đứng yên, tắt VM rồi sửa cờ INVERT tương ứng và nạp lại.
8. **Thử tiến/lùi:** sau khi từng bánh đúng chiều, lần lượt `BOTH 30`, chờ dừng
   hoàn toàn, rồi `BOTH -30`. Đây là hai lần thử riêng, không dán cả danh sách
   lệnh cùng lúc. Đảo chiều ngay khi đang quay có thể gây dòng lớn.
9. **Thử STOP và timeout:** gửi `BOTH 30`, rồi gửi `!` sớm trước 500 ms nếu có
   thể; nó không cần Enter. Nhấn Enter để xóa dòng; hoặc dùng `STOP` + Enter.
   Với pulse ngắn, thao tác tay có thể chậm hơn timeout: xem log STOP/TIMEOUT
   để phân biệt. Thử lại một xung và không gửi gì; phải có TIMEOUT. Bánh có
   thể còn trôi sau khi tín hiệu đã dừng. Thử lệnh `BOTH 151`: phải ERROR và
   không chạy. Nếu có lỗi PWM, tắt VM, sửa nguyên nhân, reset; không tự bỏ khóa.

Thoát monitor bằng Ctrl+C trước khi upload lại. Monitor dùng gửi từng ký tự
cho phép `!` tới ngay; nếu bật bộ lọc `send_on_enter` hoặc giao diện gom cả
dòng, ký tự chỉ tới khi nhấn Enter. Chi tiết tùy chọn ở
[PlatformIO Monitor](https://docs.platformio.org/en/latest/core/userguide/device/cmd_monitor.html).

### Wokwi mô phỏng được gì?

Giữ nguyên `diagram.json`: ESP32-DevKitC V4, điện trở 220 Ω và LED GPIO2.
Giữ nguyên `wokwi.toml`: trỏ firmware/ELF trong `.pio/build/esp32dev/`.
Sau build, Command Palette → `Wokwi: Start Simulator`; LED là dấu hiệu vòng
lặp hoạt động, không phải bằng chứng motor hoạt động.

[Danh sách phần cứng Wokwi](https://docs.wokwi.com/getting-started/supported-hardware)
hiện không liệt kê TB6612FNG và motor DC chổi than tích hợp. Servo và stepper
là loại khác, không thay thế để chứng minh driver này đúng. Vì vậy không có
kiểm chứng mô phỏng dòng điện, mô-men, quán tính, kẹt trục, nhiệt hay RPM.

Wokwi hỗ trợ [logic analyzer](https://docs.wokwi.com/parts/wokwi-logic-analyzer)
để đo tín hiệu số. Sau khi có mapping được xác nhận, có thể tạo **bản sao
chỉ dành cho mô phỏng** với 7 tín hiệu và GND vào analyzer: STBY LOW lúc boot,
chân chiều đúng dấu, PWM chạy khoảng 20 kHz, duty khoảng 30/255 khi lệnh 30,
STBY trở LOW sau timeout. Repo này chưa thêm sơ đồ analyzer hoặc mapping
mô phỏng; không có bản đo PWM/VCD nào được khẳng định đã thực hiện.

## Bài 5 — Sau này nối với dò line thế nào?

Cảm biến đọc vị trí vạch; PID là phép tính đưa ra mức điều chỉnh. PID sẽ gọi
giao diện driver, không cần biết tên chân PWMA hay AIN1:

```cpp
setMotorSpeeds(40, 60);  // trái chậm hơn phải
```

Với robot hai bánh chủ động, bánh phải đi nhanh hơn đẩy phía phải tiến nhiều
hơn, nên xe có xu hướng vòng sang trái khi cả hai đang tiến. Hai bánh có
cùng vận tốc thực thì xe đi thẳng; hai bánh khác tốc độ thì xe rẽ. Ví dụ
chỉ là lệnh duty: thực tế còn phải cân chỉnh do hai motor không hoàn toàn giống nhau.

Các mô-đun cảm biến TCRT5000/HC-SR04P, PID, chỉnh tham số Serial/Bluetooth và
các chế độ hoạt động có thể dùng lại driver. Khi thêm chúng, cần quản lý ai
được gửi lệnh ở mỗi chế độ, thời gian an toàn và cấp phát GPIO/LEDC; không
để chương trình thử và PID cùng điều khiển motor. Pha này chưa triển khai chúng.

## Kiểm chứng và thông tin đang thiếu

Kết quả thực hiện ngày 08/11/2026:

| Kiểm tra | Kết quả thực tế |
| --- | --- |
| `pio run -e esp32dev`, cấu hình mặc định khóa motor | SUCCESS; Arduino-ESP32 3.3.12, platform 8.0.0 |
| Build ESP32 với đường điều khiển được bật, trong project tạm | SUCCESS; dùng GPIO giả định chỉ để kiểm tra biên dịch/link, không upload, không thay config gốc; project tạm đã được xóa |
| Dung lượng bản mặc định | RAM 22,320 / 327,680 byte; Flash 296,019 / 1,310,720 byte |
| `python3 tests/run_host_tests.py` | 20 lượt đạt: 13 lượt chạy chương trình và 7 cấu hình sai bị chặn biên dịch |
| Wokwi, firmware mặc định và sơ đồ gốc | Khởi động DISABLED/STOPPED; HELP và STATUS trả lời; LEFT 30, RIGHT -30, BOTH 50 đều BLOCKED; BOTH 151 báo ERROR; STOP có phản hồi |
| PWM/GPIO motor qua logic analyzer Wokwi | Chưa đo; cấu hình motor vẫn khóa |
| Động cơ/driver trên phần cứng thật | Chưa thử; chưa upload board thật hoặc cấp nguồn motor |

Trong Wokwi đã quan sát trực tiếp log `Driver: DISABLED/FAULT | Test: STOPPED`,
ba phản hồi `BLOCKED`, phản hồi sai lệnh và `STOP: motors stopped (coast).`
Timeout khi driver được bật được kiểm tra bằng API giả trên máy Mac, chưa
được đo với GPIO thật hoặc trong Wokwi ở chế độ bật motor.

Đọc `tests/run_host_tests.py` để chạy lại kiểm tra trên máy tính bằng compiler
C++ có sẵn. Mã thật `src/main.cpp` và `src/motor.cpp` được biên dịch với API
giả ghi lại thao tác chân và UART. Các số GPIO trong fixture chỉ tồn tại
trong bản config tạm của bài test, **không phải sơ đồ chân thật hoặc Wokwi**.

Kiểm tra gồm: trước init/khóa tắt không chạm chân motor; init dừng; trái/phải,
tiến/lùi, clamp cả INT_MIN/INT_MAX, đổi kênh và cờ đảo; lệnh sai/số tràn/dòng
dài/NUL; STOP/!; timeout và phép trừ qua vòng đếm; HELP/STATUS không gia hạn;
lỗi attach/write/stop luôn giữ STBY LOW; 7 cấu hình GPIO sai phải không build.
Các test máy tính không chứng minh hoạt động điện của ESP32 hoặc driver thật.

Còn cần Toàn xác nhận:

- Bảy GPIO chính xác: PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY; tránh xung đột
  với LED hiện có và các cảm biến dự kiến.
- Model board ESP32 và module cụ thể, có PSRAM không; model/sơ đồ breakout
  TB6612FNG, đặc biệt VCC và điện trở trên STBY.
- Nguồn VM, điện áp khi đầy/tải, khả năng cấp dòng; định mức motor và dòng kẹt.
- Bánh trái/phải gắn vào A/B nào; chiều tiến thực tế của từng bánh.

Chỉ ba việc tiếp theo: xin bảng thông tin này; thử HELP/STATUS với khóa còn
tắt; sau khi xác nhận phần cứng mới mở khóa và thử một xung LEFT 30.
