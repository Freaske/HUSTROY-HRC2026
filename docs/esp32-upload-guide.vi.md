# Nạp ESP32 qua USB-C và kiểm tra Serial an toàn

> Cập nhật: theo yêu cầu thử motor tiếp theo, `include/config.h` hiện đã bật
> driver với GPIO 18/13/14/19/21/22/23. Xem [bản thử motor hiện tại](manual-motor-bench.vi.md).
> Những phần dưới nói “bản đang khóa” mô tả bản USB chẩn đoán trước đó;
> để dùng bản đó, đặt MOTOR_HARDWARE_ENABLED=false rồi build/upload lại.
> Không dùng kỳ vọng ARM bị BLOCKED cho bản motor đã bật. Sensor vẫn khóa.

## 1. Bản firmware nào đang chuẩn bị nạp?

Đây là bản **khóa phần cứng motor và cảm biến**. Có thể nạp và dùng HELP,
STATUS, chuyển chế độ khi chưa hoàn thành dây motor. Không cần bật motor để
kiểm tra USB hoặc Serial. Trong `include/config.h`, cấu hình thật vẫn là:

```cpp
constexpr bool MOTOR_HARDWARE_ENABLED = false;
constexpr int MOTOR_PWMA_PIN = -1;
constexpr int MOTOR_AIN1_PIN = -1;
constexpr int MOTOR_AIN2_PIN = -1;
constexpr int MOTOR_PWMB_PIN = -1;
constexpr int MOTOR_BIN1_PIN = -1;
constexpr int MOTOR_BIN2_PIN = -1;
constexpr int MOTOR_STBY_PIN = -1;
```

`-1` nghĩa là chưa gán chân. Cảm biến vẫn disabled, mode UNCONFIGURED và
năm chân -1; calibration/bias và nút mode chưa bật. `motorInit()` thoát trước
khi ghi GPIO/LEDC motor. LED trạng thái trên GPIO2 vẫn hoạt động theo code.
`modesInit()` chọn MOTOR_TEST, dừng và DISARM; không tự LINE_FOLLOW.

**Giới hạn quan trọng:** không ghi chân motor cũng có nghĩa firmware chưa
được phép chủ động kéo STBY vật lý xuống. Phải giữ nguồn motor ngắt trong
lần nạp này; khóa phần mềm không sửa được dây sai hoặc driver đang nối tắt.

## 2. Build, Upload và Serial Monitor khác nhau thế nào?

| Việc | Hiểu đơn giản | Có cần ESP32 cắm USB? |
| --- | --- | --- |
| Build | Dịch mã C++ thành tệp firmware để ESP32 hiểu | Không |
| Upload | Ghi firmware từ Mac vào bộ nhớ flash của ESP32 | Có |
| Serial Monitor | Cửa sổ gửi lệnh và đọc câu trả lời từ firmware đang chạy | Có trong cách thử USB này |

Flash là bộ nhớ giữ chương trình kể cả khi tắt điện. Nạp thành công thường
làm board reset và chạy chương trình mới. Sửa code trên Mac chưa thay đổi
chương trình trong board: phải Build rồi Upload lại. Build SUCCESS không
chứng minh đã Upload hay motor hoạt động.

Khi có nguồn nuôi ESP32 đúng yêu cầu của board, chương trình có thể chạy
không cần MacBook. Không tự nối pin robot vào VIN/3V3: nguồn, bộ hạ áp và
board phải được kiểm tra riêng. Serial qua USB chỉ dùng được khi có kết nối.

## 3. Chuẩn bị phần cứng cho lần USB đầu tiên

1. Ngắt nguồn motor tại VM; không để nguồn pin chưa rõ nối vào VIN hoặc 3V3.
   Nếu dây đang rời hoặc chưa kiểm tra, ngắt nguồn rồi tách board ESP32 khỏi
   mạch chưa xác minh. Đặt board trên mặt không dẫn điện.
2. Dùng cáp USB-C **có truyền dữ liệu**, nối cổng USB-C của ESP32 vào Mac hoặc
   adapter USB phù hợp. Cáp chỉ sạc có thể làm LED sáng nhưng không có cổng Serial.
3. Không chạm/chập các chân nguồn. Chưa cấp điện motor để thử upload.

ESP32 cổ điển trên DevKit thường dùng chip USB–UART làm cầu nối: cáp USB
đưa chương trình và dữ liệu Serial qua chip này. Có đầu USB-C chưa đủ để
xác định chip cầu nối hoặc khẳng định cáp có dữ liệu.

## 4. Mở project và tìm đúng cổng trên macOS

VS Code → File → Open Folder → chọn thư mục HUSTROY-HRC2026 đang có
`platformio.ini`. Bấm biểu tượng PlatformIO → mở PlatformIO Core CLI.
Hoặc dùng Terminal thường:

```bash
cd /Users/harrieum/Downloads/HUSTROY-HRC2026
pio device list
```

Nếu báo `command not found: pio`, dùng PlatformIO Core CLI hoặc thay `pio`
trong mỗi lệnh bằng `/Users/harrieum/.platformio/penv/bin/pio`; không cần
cài lại PlatformIO. Cách tìm cổng:

- Chạy `pio device list` khi chưa cắm, rồi cắm USB và chạy lại.
- Tìm cổng mới xuất hiện, thường như `/dev/cu.usbserial-...`,
  `/dev/cu.SLAB_USBtoUART` hoặc `/dev/cu.wchusbserial...`. Tên thực tế có thể khác.
- Dùng mô tả/USB ID và việc cổng biến mất khi rút board để xác định đúng thiết bị.
- Các tên Bluetooth, tai nghe hoặc `/dev/cu.debug-console` không phải bằng
  chứng có ESP32. Không chọn bừa chúng để upload.

Nếu không có cổng mới, xử lý cáp/cổng/adapter trước; chưa chạy upload.
[PlatformIO: liệt kê thiết bị](https://docs.platformio.org/en/latest/core/userguide/device/cmd_list.html)

## 5. Build và Upload

Trước hết build bản đang khóa:

```bash
pio run -e esp32dev
```

Mong đợi cuối log có `[SUCCESS]`. Board cấu hình là `esp32dev`, platform
`espressif32@8.0.0`, core đang cài Arduino-ESP32 3.3.12. Không cần đổi các
thiết lập bootloader hoặc cài lại dependencies cho lần này.

Đóng Serial Monitor đang dùng board bằng Ctrl+C, rồi upload:

```bash
pio run -e esp32dev -t upload
```

Nếu chỉ có một cổng ESP32 xác định rõ, PlatformIO thường tự tìm được. Khi
có nhiều thiết bị hoặc chọn sai, chỉ định cổng. Thay phần TEN_CONG_THUC
bằng tên vừa tìm được; dòng dưới là mẫu, không phải cổng đã phát hiện:

```bash
pio run -e esp32dev -t upload --upload-port /dev/cu.TEN_CONG_THUC
```

Trên giao diện VS Code: PlatformIO → PROJECT TASKS → esp32dev → General →
Build, rồi Upload. Có thể dùng nút dấu tích Build và mũi tên Upload ở thanh
trạng thái, kiểm tra tên khi rê chuột; giao diện có thể khác theo phiên bản.
Không nhầm nút chạy Wokwi với Upload board thật.

Một lần upload thành công cần log kết nối chip, ghi flash, kiểm tra dữ liệu
và kết thúc SUCCESS. Tùy phiên bản có các dòng như `Writing at ...`,
`Hash of data verified`, `Hard resetting via RTS pin...`; câu chữ có thể khác.
Chỉ thấy Build SUCCESS trước phần upload chưa đủ. Sau đó phải đọc đúng banner
và thử HELP/STATUS để xác nhận ứng dụng mới thực sự chạy.

Không rút cáp giữa lúc ghi. Không dùng erase flash hoặc sửa partition/bootloader
để thử khắc phục khi chưa xác định nguyên nhân lỗi.

## 6. Mở Serial Monitor, kiểm tra chương trình

```bash
pio device monitor -b 115200 --echo --eol LF
```

Nếu cần chọn cổng:

```bash
pio device monitor -p /dev/cu.TEN_CONG_THUC -b 115200 --echo --eol LF
```

Trong VS Code có thể mở PROJECT TASKS → esp32dev → Monitor. Lệnh CLI trên
đặt rõ baud 115200, hiển thị ký tự mình gõ và gửi xuống dòng LF khi Enter.
`--echo` chỉ giúp thấy mình gõ gì, không phải board đã phản hồi.
Ctrl+C đóng monitor, không xóa firmware.
[PlatformIO: Serial Monitor](https://docs.platformio.org/en/latest/core/userguide/device/cmd_monitor.html)

Sau reset, phần ứng dụng phải in:

```text
MOTOR DISABLED: confirm 7 GPIOs and wiring in config.h with Toan.
SENSORS DISABLED: confirm AO/DO, GPIOs and safe signal voltages in config.h.
MODE=MOTOR_TEST STOPPED and DISARMED. ARM required before motion.
HUSTROY integrated firmware: startup STOPPED and DISARMED. Type HELP.
```

Có thể có log boot ESP32 trước những dòng này. Nếu mở monitor muộn, banner
đã chạy qua: gửi STATUS hoặc bấm EN một lần khi VM vẫn ngắt để đọc lại.

Gửi từng dòng, viết HOA và nhấn Enter:

| Lệnh | Kết quả mong đợi với bản đang khóa |
| --- | --- |
| HELP | Danh sách lệnh, khoảng duty, timeout và ba chế độ |
| STATUS | MODE=MOTOR_TEST, ARMED=0, RUN=0, MOTOR=DISABLED/FAULT, SENSORS=DISABLED |
| MODE MOTOR_TEST | Dừng, DISARM và chọn chế độ |
| ARM | BLOCKED vì phần cứng chưa được cho phép |
| LEFT 50 | BLOCKED; không có tín hiệu motor |
| RIGHT 50 | BLOCKED |
| BOTH 50 | BLOCKED |
| SET 50 30 | BLOCKED |
| STOP | Báo dừng cả hai |
| DISARM | Báo dừng và hủy quyền |
| STATUS | ARMED=0 và RUN=0 |

`STOP=` trong STATUS là nguyên nhân dừng gần nhất, có thể thay đổi theo lệnh.
`DISABLED/FAULT` gộp hai tình huống; trong bản mặc định này DISABLED là có chủ
ý, chưa phải bằng chứng motor hỏng. ARM không vượt được khóa compile/config.
RUN cũng không hoạt động khi hardware/sensor/calibration chưa đáp ứng.

## 7. Xử lý lỗi upload hoặc Serial

| Hiện tượng | Kiểm tra theo thứ tự |
| --- | --- |
| LED sáng nhưng không thấy cổng | Đổi cáp dữ liệu đã biết tốt, thử cổng/adapter khác, kiểm tra lại danh sách thiết bị |
| Vẫn không thấy cổng với cáp tốt | Xác định đúng chip USB–UART; chỉ cài driver chính hãng nếu chip đó thực sự cần, không cài driver ngẫu nhiên |
| Cổng bận | Đóng monitor, Arduino Serial Monitor và chương trình khác đang giữ đúng cổng; chạy lại |
| Cổng biến mất | Kiểm tra tiếp xúc/cáp/nguồn USB, liệt kê lại vì tên cổng có thể đổi |
| Failed to connect / Connecting mãi | Kiểm tra đúng cổng, giữ VM ngắt, rồi thử BOOT/EN theo bên dưới |
| Ghi flash thất bại giữa chừng | Kiểm tra cáp/nguồn/cổng; đọc log lỗi, không tự xóa flash |
| Monitor chữ rác | Chọn 115200 baud |
| Monitor im lặng | Gửi STATUS + Enter, kiểm tra đúng cổng và reset EN; LED nguồn không chứng minh firmware đã nạp |
| Gõ lệnh không phản hồi | Viết HOA, gửi LF/CRLF, đúng cổng; đóng và mở lại monitor nếu cần |
| Đổi code mà thấy hành vi cũ | Lưu tệp, Build và Upload lại; kiểm tra log upload thành công rồi reset |

BOOT thường đưa GPIO0 xuống LOW; EN reset chip. Thông thường upload tự reset
được và không cần bấm nút. Nếu Failed to connect:

1. Giữ BOOT, nhấn rồi thả EN trong khi vẫn giữ BOOT.
2. Bắt đầu hoặc thử lại Upload; giữ BOOT qua lúc reset/kết nối.
3. Khi bắt đầu ghi flash, thả BOOT. Sau thành công, nếu vẫn ở chế độ nạp,
   thả BOOT hoàn toàn và nhấn EN một lần để chạy chương trình.

Không giữ EN liên tục hoặc nhấn reset khi flash đang ghi. Nếu vẫn lỗi, ghi
lại lỗi cụ thể thay vì sửa bootloader/eFuse.
[Espressif: chế độ BOOT và reset](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html),
[Espressif: lỗi nạp](https://docs.espressif.com/projects/esptool/en/latest/esp32/troubleshooting.html).

## 8. Cấu hình chỉ dùng SAU KHI Toàn và bạn duyệt phần cứng

Đây là **đề xuất**, không phải cấu hình đang nạp và không được tự động include.
Trước khi bật, cần xác nhận từng dây GPIO trên breakout thật; board/module;
nguồn pin (loại, điện áp khi đầy, cực tính); motor (điện áp và dòng kẹt);
khả năng dòng/nhiệt driver; common GND; mạch STBY giữ tắt khi reset;
tụ nguồn; kênh A/B nối bánh nào. Không đủ thông tin thì giữ disabled.

Chỉ sau xác nhận, thay đúng các định nghĩa đang có trong `include/config.h`
bằng khối tương ứng dưới đây; **không dán thêm định nghĩa trùng**:

```cpp
// AFTER HARDWARE APPROVAL ONLY — not applied to the safe USB build.
constexpr bool MOTOR_HARDWARE_ENABLED = true;
constexpr int MOTOR_PWMA_PIN = 18;  // D18
constexpr int MOTOR_AIN1_PIN = 13;  // D13
constexpr int MOTOR_AIN2_PIN = 14;  // D14
constexpr int MOTOR_PWMB_PIN = 19;  // D19
constexpr int MOTOR_BIN1_PIN = 21;  // D21
constexpr int MOTOR_BIN2_PIN = 22;  // D22
constexpr int MOTOR_STBY_PIN = 23;  // D23
```

Khối này giả định chính bảng đề xuất được duyệt; nếu dây được duyệt khác,
phải dùng dây thật và rà soát xung đột lại. Các tùy chọn tương ứng đã có:

```cpp
constexpr bool LEFT_MOTOR_IS_CHANNEL_A = true; // Chỉ đúng nếu A=trái, B=phải.
constexpr bool INVERT_LEFT_MOTOR = false;      // Cần kiểm tra chiều bánh thật.
constexpr bool INVERT_RIGHT_MOTOR = false;
```

Nếu A=phải/B=trái thì cờ đầu phải false. Hai cờ INVERT chỉ đổi sau khi kiểm
tra chiều đúng bánh. Không đổi đồng thời nhiều cờ để đoán lỗi.

Giữ nguyên timeout 500 ms, PWM 8 bit/20 kHz, MAX_SPEED=150 và BASE_SPEED=50;
chúng chưa được hiệu chỉnh và không phải giới hạn dòng điện. Giữ sensor,
calibration, nút mode disabled; không bật LINE_FOLLOW trong lần thử motor đầu.
Driver kiểm tra GPIO chưa gán/trùng/không phù hợp khi build và lỗi PWM khi chạy;
những kiểm tra này không thể biết dây thật đang cắm sai hoặc VM đúng hay không.

VCC driver dự kiến nối 3V3, VM dùng nguồn motor đã duyệt, GND dùng chung.
Motor cần nguồn riêng phù hợp vì tiêu thụ dòng lớn hơn GPIO có thể cấp;
không cấp motor từ GPIO/3V3. Không nối pin chưa biết vào VIN. Xem kiểm tra
điện và sơ đồ đầy đủ ở [hướng dẫn thử motor](physical-motor-first-drive.vi.md).

Sau khi duyệt: ngắt VM, sửa config, chạy lại host tests và Build, Upload bản
mới với VM vẫn ngắt, kiểm tra banner/STATUS rồi mới chuẩn bị thử công suất.
MOTOR=READY vẫn phải kèm ARMED=0/RUN=0; READY chỉ xác nhận khởi tạo GPIO/PWM.

## 9. Thử motor sau duyệt — chưa thực hiện trong phiên này

Kê và cố định xe để cả hai bánh rời mặt bàn, tránh tay/dây cuốn bánh. Chuẩn
bị ngắt VM vật lý rồi mới cấp nguồn motor đã duyệt. Gửi từng lệnh riêng:

1. MODE MOTOR_TEST, ARM: bánh vẫn dừng, ARMED=1.
2. LEFT 30: chỉ trái nhận lệnh; ghi bánh thực tế. Chờ timeout khoảng 500 ms,
   xem STATUS: RUN=0/ARMED=0. Nếu sai bánh, ngắt VM rồi sửa ánh xạ A/B.
3. ARM rồi RIGHT 30: xác nhận bánh còn lại. Mức thấp có thể chưa thắng ma sát;
   không tăng liên tục để ép quay, cần kiểm tra nguồn/dây và cơ khí trước.
4. ARM rồi BOTH 30: cả hai tiến logic. Cùng PWM không bảo đảm cùng RPM.
5. Chờ trục đứng yên, ARM lại cho từng lệnh LEFT -30, RIGHT -30, BOTH -30.
   Nếu sai chiều, STOP/DISARM, ngắt VM trước khi sửa INVERT/nạp lại.
6. Thử STOP trước timeout nếu kịp: RUN=0, giữ ARM. DISARM hủy ARM; lệnh
   LEFT 30 sau đó phải BLOCKED. Timeout cũng hủy ARM. Có thể dùng ! rồi Enter;
   ký tự chỉ tác dụng khi đã được gửi tới firmware, không phải nút cắt điện.

LEFT đặt trái=n/phải=0; RIGHT đặt trái=0/phải=n; BOTH đặt hai bên=n;
SET cho hai số độc lập. ARM là cấp quyền, không phải lệnh quay. DISARM là
dừng và thu hồi quyền. Đổi mode luôn DISARM. STOP dùng coast (thả trôi),
không khóa cứng bánh; không thay thế công tắc nguồn khi có sự cố.

Chưa thử trên mặt đất hoặc PID trước khi từng bánh, chiều, dừng và timeout
được xác nhận. Quy trình chi tiết và bảng ghi nhận nằm trong hướng dẫn motor.

## 10. Ghi nhận kiểm chứng

- `pio run -e esp32dev`: SUCCESS, 0.58 giây (build tăng dần với môi trường đã
  cài); RAM 22,464/327,680 byte, flash 304,355/1,310,720 byte. Core 3.3.12.
- `python3 tests/run_host_tests.py`: toàn bộ 24 kịch bản chạy và 14 cấu hình
  sai bị từ chối đúng dự kiến đã đạt. Đây là test trên máy tính với GPIO,
  Serial và thời gian giả, không đo điện hay bánh xe.
- Đã bổ sung và chạy lại riêng kịch bản disabled: kiểm tra banner DISARMED,
  nội dung HELP/STATUS, không running, không cấp LEDC sau các lệnh chẩn đoán
  hoặc lệnh motor bị chặn. Kết quả PASS; không đổi hành vi firmware.
- `pio device list` và pyserial chỉ thấy cổng hệ thống/Bluetooth, không có
  cổng USB ESP32 được nhận diện. Bạn xác nhận chưa cắm ESP32 và muốn nạp sau.
  Vì vậy **không chạy upload, không đọc Serial từ board thật**; các khối log
  ở mục 6 là kết quả mong đợi theo mã nguồn, không phải log nạp thật.
- Giữ `include/config.h`, các module firmware, `platformio.ini` và Wokwi
  nguyên trạng. Không reinstall dependencies, xóa flash, sửa bootloader,
  commit, push hoặc merge. Nhánh vẫn `feature/physical-motor-integration`.

Các tệp thay đổi: hướng dẫn này, liên kết README, bản đề xuất config dạng
`.h.txt` (sửa chú thích/định danh cũ và bỏ khẳng định ECHO đã xác minh), cùng
kiểm tra host disabled. Tài liệu `.h.txt` không được PlatformIO biên dịch.

Không có ảnh chụp màn hình minh họa được tạo giả; vị trí UI ở trên là chỉ dẫn
bằng chữ. Upload, hoạt động Serial trên board thật và motor vật lý chưa được
kiểm chứng trong lần chuẩn bị này.
