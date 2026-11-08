# Thử motor vật lý: hai bánh độc lập, ARM và DISARM

> Cập nhật cấu hình: theo yêu cầu người dùng, driver đã bật cho thử motor
> thủ công, vẫn chờ ARM. Quy trình hiện tại ở [manual-motor-bench.vi.md](manual-motor-bench.vi.md).
> Các kết quả “GPIO=-1/hardware khóa” bên dưới ghi lại pha trước;
> chúng không còn mô tả cấu hình mặc định hiện tại. Chưa có kiểm chứng motor thật.

Tài liệu này cập nhật giao diện thử motor của pha trước. Firmware thật vẫn
khóa phần cứng, bảy GPIO=-1. Không upload hoặc cấp nguồn motor trong lần làm
việc này. Board ESP32 có ảnh; chưa truy cập được ảnh khung xe/breakout mới.

## 1. Trả lời câu hỏi về mã trước khi sửa

Mã trước lần sửa này đã có driver **hai kênh độc lập**, không chỉ điều khiển
một bánh. `setup()` gọi motorInit/sensorsInit/modesInit; `loop()` gọi modesUpdate.
Không có motorForward(BASE_SPEED) tự chạy trong main hiện tại.

| Câu hỏi | Kết quả kiểm tra mã |
| --- | --- |
| Hàm điều khiển trái/phải | Cùng setMotorSpeeds(left,right), hai đối số riêng |
| motorForward(BASE_SPEED) | Gọi setMotorSpeeds(speed,speed), cả hai tiến logic; không chỉ một bánh |
| Chạy một bánh, bánh kia dừng | Có: (50,0) hoặc (0,50) |
| Chạy hai tốc độ khác nhau | API đã có: (100,60); trước sửa chưa có Serial hai đối số |
| Lùi độc lập | Có: (-50,0), (0,-50), (-50,50) |
| Tín hiệu thật hay chỉ Serial | Driver có digitalWrite/ledcAttach/ledcWrite; nhánh hardware khóa không ghi chân motor |
| A/B là bánh nào | Config tạm A=trái, B=phải; chưa xác nhận dây thật |
| ARM/DISARM | Chưa có trước lần sửa này; đã bổ sung ở modes |

Mã đối chiếu: src/main.cpp (setup/loop), src/motor.cpp (motorInit,
setMotorSpeeds, motorForward, motorStop), include/config.h (khóa và đảo chiều),
src/modes.cpp (manual test và PID gọi cùng giao diện motor).

## 2. Đã thay đổi gì?

- RobotStatus có `armed`, khởi động false; khác với hardware READY.
- `modesArm()` dừng trước, kiểm tra mode/hardware rồi cấp quyền; không quay bánh.
- `modesDisarm()` dừng và hủy quyền. Mọi nhánh gây chuyển động trong modes
  đều phải qua ARM, bao gồm LINE_FOLLOW trước RUN.
- Manual có `SET left right` để thử hai tốc độ hoặc hai dấu khác nhau.
- STOP thường dừng nhưng giữ ARM. DISARM, !, timeout, lỗi và chuyển mode
  dừng + hủy ARM. STOP sai cú pháp cũng dừng + hủy ARM.
- Giữ driver motor, thuật toán sensor, PID theo dt, anti-windup và ba mode.
  Không triển khai thêm Bluetooth/siêu âm, không nới guard GPIO16/17.

ARM là khóa ở bộ điều phối ứng dụng; driver motor là lớp xuất tín hiệu mức
thấp. Không thêm lời gọi motorForward vào setup/loop ngoài bộ điều phối.
READY chỉ báo GPIO/LEDC khởi tạo được, không đo được dây, VM hay motor thật.

## 3. Cú pháp Serial mới

Viết HOA, Enter sau mỗi lệnh (CR/LF/CRLF). Thang PWM8bit 0..255; lệnh chạy
hiện giới hạn -150..150, chưa phải giá trị hiệu chỉnh motor. API kẹp giới
hạn; Serial từ chối số ngoài khoảng để người thử biết nhập sai.

| Lệnh | Kết quả |
| --- | --- |
| HELP / STATUS | Hướng dẫn; MODE, ARMED, RUN, hardware/sensor và nguyên nhân dừng |
| MODE MOTOR_TEST | Dừng + DISARM, chọn thử motor |
| ARM | Dừng trước, cấp quyền khi hardware READY; không quay motor |
| LEFT 50 | Trái50, phải0 |
| RIGHT 50 | Trái0, phải50 |
| BOTH 50 | Trái50, phải50 |
| SET 100 60 | Trái100, phải60; chỉ dùng sau khi thử mức thấp thành công |
| LEFT -50 / RIGHT -50 | Chỉ bánh tương ứng lùi |
| BOTH -50 | Cả hai lùi |
| SET -50 50 | Trái lùi, phải tiến; không dùng cho lần thử mặt đất đầu |
| STOP | Dừng cả hai, giữ ARM; không tự chạy lại |
| DISARM | Dừng và chặn lệnh chạy đến khi ARM lại |
| ! | Dừng + DISARM ngay khi nhận ký tự; Enter trước lệnh tiếp |
| MODE SENSOR_TEST | Dừng + DISARM; không cho ARM trong mode này |
| MODE LINE_FOLLOW | Dừng + DISARM; cần ARM, chờ mẫu hợp lệ rồi RUN |

Mỗi test manual hết hạn theo MOTOR_TEST_TIMEOUT_MS=500ms từ lệnh chuyển động
hợp lệ cuối, rồi DISARM. Cần ARM lại trước xung tiếp theo. Mỗi RUN line tối
đa5s và không tự gia hạn khi gửi RUN lặp. Lệnh sai/dòng quá dài/số tràn đều
dừng + DISARM. Không thêm delay, dùng millis và lượng đọc Serial có giới hạn.

ARM không vượt được MOTOR_HARDWARE_ENABLED=false, chân chưa gán hoặc lỗi PWM.
Trước khi phần cứng được duyệt, ARM phải báo BLOCKED; đó là kết quả đúng.

## 4. Chân dây và kênh trái/phải

Các thông số dưới đây **đề xuất, chưa áp dụng**. Bảng 16/17 trong yêu cầu chỉ
phù hợp sau khi xác minh module WROOM cụ thể và xem lại guard. Tài liệu mới
theo ảnh và driver đang dùng ưu tiên13/14 để tránh phụ thuộc PSRAM/module.

| TB6612 | ESP32 theo phương án hiện có | Nhiệm vụ / loại dây |
| --- | --- | --- |
| PWMA | D18/GPIO18 | Tín hiệu PWM kênh A |
| AIN1 | D13/GPIO13 (thay D16 đang chờ xác nhận) | Tín hiệu chiều A1 |
| AIN2 | D14/GPIO14 (thay D17 đang chờ xác nhận) | Tín hiệu chiều A2 |
| PWMB | D19/GPIO19 | Tín hiệu PWM kênh B |
| BIN1 | D21/GPIO21 | Tín hiệu chiều B1 |
| BIN2 | D22/GPIO22 | Tín hiệu chiều B2 |
| STBY | D23/GPIO23; nhánh10kΩ xuống GND | Tín hiệu cho phép chung |
| VCC | 3V3 | Nguồn logic |
| VM | Cực+ nguồn motor đã duyệt qua cầu chì/công tắc | Nguồn công suất |
| GND | ESP32 GND + driver GND + nguồn motor(-) | Mốc0V và đường hồi dòng |
| AO1/AO2 | Hai dây motor A | Đầu ra công suất, tạm trái |
| BO1/BO2 | Hai dây motor B | Đầu ra công suất, tạm phải |

Không vẽ vị trí header TB6612 vì chưa có ảnh breakout. Đấu theo tên in trên
module đã xác nhận, không theo thứ tự trên một ảnh module khác.

```text
ESP32 D18/D13/D14 ---- PWMA/AIN1/AIN2   TB6612   AO1/AO2 ---- motor A
ESP32 D19/D21/D22 ---- PWMB/BIN1/BIN2            BO1/BO2 ---- motor B
ESP32 D23 ------------ STBY
                        |
                      [10kΩ]
                        |
ESP32 GND -------------+---- driver GND ---- nguồn motor(-)
ESP32 3V3 ------------------ VCC
Nguồn motor(+) ---- cầu chì ---- công tắc ---- VM
USB-C cấp ESP32; không nối nguồn motor(+) vào VIN hoặc3V3.
```

Nếu LEFT làm quay bánh phải: STOP/DISARM, ngắt VM, kiểm tra dây. Sau xác nhận,
đổi LEFT_MOTOR_IS_CHANNEL_A=false để trái logic chuyển sang kênh B. Không cần
đổi hàng loạt hằng chân. Nếu đúng bánh nhưng chiều ngược: đổi cờ
INVERT_LEFT_MOTOR hoặc INVERT_RIGHT_MOTOR tương ứng. Không sửa cờ khi VM còn
cấp điện hoặc nạp lại giữa lúc bánh đang quay.

## 5. Điều nhìn được và điều chưa xác minh

Ảnh truy cập được IMG_8849.HEIC cho thấy board30chân USB-C, vỏ module ghi
ESP-32 và nhãn D18/D19/D21/D22/D23... Không đọc đủ mã module/revision.
Không thấy ảnh khung xe/TB6612 trong dữ liệu truy cập được của lượt này.
Khung acrylic, hai motor vàng, hai bánh, caster, hộp pin và dây rời là mô
tả của người dùng, không ghi thành quan sát ảnh đã xác nhận.

Chưa biết chân A/B nối bánh nào, cực nguồn, điện áp pin đầy, số cell/loại pin,
định mức motor/dòng kẹt, breakout chính xác, mối hàn, GND chung hoặc thông mạch.
Không thể xác nhận những thứ đó chỉ từ màu dây hoặc hình dáng motor.

## 6. Kiểm tra điện trước cấp nguồn công suất

1. Rút USB và nguồn motor trước khi sửa dây/đo thông mạch. Kiểm tra từng dây
   theo tên chân bằng đồng hồ khi mọi nguồn đã ngắt; không dùng chế độ ohm/
   thông mạch trên mạch đang cấp điện. Kiểm tra mối hàn và không chập rail.
2. Xác nhận board/module và breakout. GPIO16/17 dùng được trên WROOM phù
   hợp nhưng dành nội bộ trên WROVER; chưa có bằng chứng thì không nới guard.
   Tránh chân boot0/2/5/12/15 và flash; không dùng chân input-only cho motor.
3. VCC TB6612 đề xuất3.3V. Dải logic IC2.7–5.5V, ngưỡng HIGH=0.7×VCC:
   VCC3.3V cần ít nhất2.31V, phù hợp ESP32; VCC5V đòi3.5V, không bảo đảm.
4. VM IC làm việc2.5–13.5V nhưng motor/breakout có thể cho phép thấp hơn.
   Kiểm tra điện áp pin khi đầy và điều kiện tăng/sụt áp. Không tự chọn pin
   hoặc coi15V tuyệt đối là điện áp vận hành nên dùng.
5. Cần dòng kẹt mỗi motor tại điện áp định dùng. Bảng operating range ghi
  1.0A/kênh khi VM≥4.5V, 0.4A khi2.5≤VM<4.5V, điều kiện khôngPWM; còn phải
   xét tản nhiệt/breakout. Đỉnh3.2A chỉ10ms, không áp dụng xung thử500ms.
   Chưa có thông số motor thì chưa kết luận đạt. Không giữ trục để thử kẹt.
6. GPIO không cấp motor. VM là nguồn công suất, VCC là logic. Nuôi ESP32 qua
   USB-C khi thử bàn; không nối pin chưa biết vào VIN/3V3/VM. Chỉ sau khi nguồn
   motor được duyệt mới nối VM đúng cực, qua dây/cầu chì/công tắc phù hợp.
7. GND chung không phải nối chung cực dương. Cho dòng motor về nguồn qua dây
   công suất phù hợp, không chạy xuyên GND mảnh ESP32/rail breadboard chưa rõ
   khả năng dòng. Hai đầu motor chỉ vào AO/BO, không nối riêng xuống GND.
8. Kiểm tra tụ lọc, STBY kéo xuống và mọi điện trở kéo lên trên breakout.
   IC có pull-down nội200kΩ; điện trở ngoài10kΩ giúp giữ tắt khi reset nếu
   breakout không có đấu nối xung đột. Không nối STBY cố định HIGH.
9. Giữ VM ngắt khi nạp/reset/rút USB. Chuẩn bị ngắt nguồn vật lý: STOP/! là
   lệnh phần mềm, không bảo vệ được khi ESP32 treo, dây sai hoặc driver hỏng.

## 7. Hiểu API bằng ví dụ

TB6612 có hai cầu H riêng: mỗi cầu giống nhóm công tắc điện tử đổi điện áp
giữa hai đầu motor. A có PWMA/AIN1/AIN2, B có PWMB/BIN1/BIN2. STBY là khóa
chung nhưng PWM/chiều từng kênh độc lập.

motorForward(50) kẹp độ lớn50 rồi gọi setMotorSpeeds(50,50). setMotorSpeeds
áp cờ đảo từng bánh, chuyển trái/phải sang A/B theo config rồi ghi từng kênh.
(50,0) chỉ trái kéo; (50,50) cả hai kéo cùng duty; (100,60) trái được lệnh lớn
hơn phải. PWM là bật/tắt nhanh, không phải RPM; cùng duty chưa chắc cùng tốc
độ thật. Mức30/50 thấp có thể chưa thắng ma sát, không chứng minh driver hỏng.

Dương/âm là chiều tiến/lùi **logic**. Chiều thật phụ thuộc dây và cách gắn
motor đối xứng; phải xác nhận trên giá đỡ trước khi thử đất. Đảo chiều bằng
số âm chỉ sau khi STOP/timeout và chờ trục đứng yên.

STOP cả hai: STBY LOW, standby và coast (thả trôi). Chỉ một bánh0: IN1/IN2
LOW, PWM HIGH chọn coast của kênh đó trong khi STBY HIGH cho bánh còn lại.
Vì thế PWM bánh dừng có thể255, không phải chạy hết công suất. Không cung
cấp lệnh phanh điện trong pha này. Bánh còn trôi sau STOP là có thể xảy ra.

PID vẫn lấy trọng số sensor trái→phải -2..+2, cập nhật20ms theo dt thực,
giới hạn tích phân/correction, tạo left=base+correction, right=base-correction
rồi gọi setMotorSpeeds thật. Error dương (line phải) làm trái nhanh hơn phải
nếu convention đúng; có INVERT_PID_ERROR. Gains20/0/0 và base50 chưa tune.

## 8. Thử từng bước sau khi phần cứng được duyệt

Không dán cả chuỗi lệnh chuyển động một lúc. Mỗi dòng dưới là một lần thao
tác, quan sát và chờ dừng trước lần tiếp theo. Ở config hiện tại chỉ thử được
HELP/STATUS/chuyển mode/ARM bị BLOCKED; không được bỏ guard để ép quay.

1. **Build:** VS Code mở đúng thư mục, PlatformIO Core CLI chạy
   `pio run -e esp32dev`, mong SUCCESS. Thiếu pio thì dùng
   `~/.platformio/penv/bin/pio`; lỗi GPIO thì sửa bảng chân đã duyệt.
2. **Upload:** VM rút, USB dữ liệu nối board sau kiểm tra điện. `pio device list`,
   rồi `pio run -e esp32dev -t upload` khi sẵn sàng. Nhiều cổng thì chọn đúng
   `--upload-port /dev/cu.TEN_CONG_THUC`. Đóng monitor khác nếu cổng bận.
3. **Monitor:** `pio device monitor -b 115200 --echo --eol LF`, thêm `-p` nếu
   cần. HELP rồi STATUS. Chữ rác→baud, im lặng→cáp/cổng/Enter.
4. **Xác nhận tắt:** phải có ARMED=0, RUN=0. MOTOR=DISABLED/FAULT là đúng khi
   chưa cấu hình. MOTOR=READY chỉ mong đợi sau khi GPIO/khóa được duyệt và
   cập nhật firmware, không chứng minh VM/dây đúng. Không có tự quay khi boot.
5. **Kê bánh:** cố định thân, hai bánh chủ động rời mặt bàn, dây/tay tránh
   bánh. Chuẩn bị công tắc rút VM. Chỉ cấp VM sau toàn bộ kiểm tra mục6 đạt.
6. **Arm:** MODE MOTOR_TEST, rồi ARM; STATUS phải ARMED=1/RUN=0, bánh vẫn dừng.
   BLOCKED thì kiểm tra config/READY; không nối tắt STBY để chữa.
7. **Trái:** gửi `LEFT 30`. Mong chỉ một bánh nhận lệnh;
   sau khoảng500ms có TIMEOUT và ARMED=0. Nếu không quay, STOP/DISARM và kiểm
   tra nguồn/dây/ma sát trước khi cân nhắc50; không tăng liên tục để ép motor.
8. **Xác định bánh:** ghi bánh thật nào quay và hướng trục. Nếu phải quay khi
   LEFT, ngắt VM và xác nhận A/B; đổi LEFT_MOTOR_IS_CHANNEL_A sau xác nhận.
9. **Phải:** ARM lại, gửi `RIGHT 30`, quan sát bánh còn lại, chờ timeout.
   Nếu cùng một bánh quay cho hai lệnh, kiểm tra dây kênh, không chỉ đổi INVERT.
10. **Cả hai:** sau khi từng bánh đúng, ARM rồi gửi `BOTH 30`. Hai bánh
    phải nhận lệnh tiến logic; chờ dừng. Chưa thử `SET 100 60` ngay; mức thấp trước.
11. **Lùi:** chờ trục đứng yên, ARM rồi gửi `LEFT -30`; lặp riêng `RIGHT -30`
    và `BOTH -30` với ARM mỗi lần. Nếu sai chiều, VM tắt rồi sửa cờ INVERT đúng
    bánh và nạp lại, không sửa khi đang quay.
12. **Dừng/khóa:** thử STOP hoặc ! trước timeout nếu kịp. STOP giữ ARM, DISARM
    và ! hủy ARM. Sau DISARM, `LEFT 30` phải BLOCKED. Sau timeout cũng phải ARM
    lại. Khi ! dùng xong Enter trước lệnh tiếp. Với pulse500ms, tay có thể
    chậm hơn timeout; xem nguyên nhân dừng trong STATUS để phân biệt. Nếu
    monitor gom dòng, ! chỉ tới khi bấm gửi. Đừng coi STOP là cắt nguồn khẩn.
13. **Ghi nhận:** hoàn thành bảng dưới rồi mới thử đất. Lần chạy đất đầu dùng
    lệnh BOTH mức thấp/thời gian ngắn trên khoảng trống, đứng sẵn cạnh công
    tắc VM. Không bật LINE_FOLLOW cho đến khi sensor/calibration/bias và dấu
    correction được xác nhận; mode đó vẫn cần ARM, chờ mẫu rồi RUN.

| Mục ghi nhận | Trái | Phải |
| --- | --- | --- |
| Nối kênh A/B | Chưa xác nhận | Chưa xác nhận |
| Bánh quay đúng khi LEFT/RIGHT | Chưa thử | Chưa thử |
| Dấu dương làm bánh đẩy xe tiến | Chưa thử | Chưa thử |
| Cờ INVERT cần đặt | Chưa xác nhận | Chưa xác nhận |
| Duty thấp nhất khởi động ổn định, VM đo được | Chưa đo | Chưa đo |
| STOP/DISARM/timeout | Chưa thử thật | Chưa thử thật |

## 9. Kết quả kiểm chứng trong lần cập nhật này

- `pio run -e esp32dev`: **SUCCESS**, 9.46 giây; RAM 22,464/327,680 byte,
  flash 304,355/1,310,720 byte. Framework cài thực tế Arduino-ESP32 3.3.12;
  driver dùng `ledcAttach(pin, frequency, bits)` và `ledcWrite(pin, duty)`.
- `python3 tests/run_host_tests.py`: **38 trường hợp đạt**, gồm 24 kịch bản
  chạy và 14 cấu hình sai bị từ chối biên dịch đúng dự kiến. Test chạy mã
  thật với GPIO/Serial/thời gian giả trên máy tính: ARM không tự chạy,
  thiếu ARM bị chặn, LEFT/RIGHT/BOTH/SET, lùi độc lập, clamp, cú pháp sai,
  timeout, reset/chuyển mode, PID và cảm biến hợp lệ nhưng RUN chưa ARM.
  Đây không phải đo tín hiệu điện thật.
- Một bản sao tạm có bật motor và gán chân 18/13/14/19/21/22/23 cũng build
  ESP32 **SUCCESS** (15.19 giây). Chỉ kiểm tra nhánh biên dịch; không upload,
  không xác nhận dây, không đổi config thật; bản sao tạm đã dọn.
- Wokwi với firmware mới: LED hiện có vẫn hoạt động; startup báo STOPPED
  và DISARMED. STATUS cho ARMED=0/RUN=0, ARM bị BLOCKED vì hardware khóa;
  LEFT 50/SET 50 30 bị chặn; DISARM hoạt động; BOTH 151 báo lỗi; STOP rồi
  STATUS vẫn ARMED=0/RUN=0. Không thêm mô hình TB6612/motor hoặc xác nhận
  tốc độ, chiều quay, dòng điện qua mô phỏng này.
- Không có upload, đo điện hoặc quay bánh thật trong phiên này. Điện áp,
  dòng kẹt, A/B, chiều quay, STOP và timeout trên robot vẫn phải thử vật lý.

Nhánh: `feature/physical-motor-integration`. Giữ nguyên thay đổi chưa commit
có sẵn của pha sensor/PID; không commit, push hoặc merge. Commit gợi ý:
`feat(motor): require arming and add independent wheel test commands`.

## Nguồn đối chiếu

- [Toshiba TB6612FNG: nguồn, dòng, logic và coast/standby](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660&prodName=TB6612FNG)
- [Espressif: GPIO16/17 tùy module](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html)
- [Arduino-ESP32 LEDC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
