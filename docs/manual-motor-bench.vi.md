# Bản thử motor thủ công đang bật

Timeout manual hiện là **10 giây** theo yêu cầu mới. Đã chạy demo Wokwi và
đo GPIO/PWM; xem [kết quả mô phỏng](wokwi-motor-signals/README.md).
Các số liệu build/timeout500ms ở phần ghi nhận lịch sử không phải bản mới.

## Nhận diện bản LED mới

Bản hiện tại `motor-bench-led2-v2-10s` phát mẫu trên GPIO2: sáng100ms, tắt100ms,
sáng100ms, tắt700ms rồi lặp. Không dùng delay; timeout motor và xử lý Serial
vẫn chạy. GPIO không đổi sang chân khác: chưa có bằng chứng LED tích hợp
board thật nối ở đâu. LED nguồn sáng liên tục không phải LED lập trình.

Sau nạp/reset, banner và STATUS có `FW=motor-bench-led2-v2-10s`. STATUS thêm
PWM, bảng chân và L_REQ/R_REQ. Đây là cấu hình/lệnh phần mềm, không phải
phép đo điện hoặc xác nhận bánh quay; timeout làm L_REQ/R_REQ về0.

Nếu có `TEST: left=50 right=0` rồi TIMEOUT mà bánh không quay, lệnh đã được
chấp nhận và xử lý, nhưng chưa biết VM/VCC thực, thông mạch, điện áp đầu ra,
ma sát hay tình trạng motor. PWM50 chỉ khoảng20% duty. Kiểm tra nguồn/dây/
cơ khí trước khi tăng; không suy ra lắp đúng chỉ từ READY hoặc LED nháy.

Không có đồng hồ thì cần ảnh rõ chân TB6612, mối hàn, dây nguồn và đường
GND để kiểm tra bằng mắt. Không dùng chế độ sửa code/đổi GPIO ngẫu nhiên
để thay phép đo nguồn. Ngắt mọi nguồn trước khi chỉnh dây.

Ghi nhận lịch sử bản LED v1: `pio run -e esp32dev` SUCCESS (8.92 giây), RAM
22,512 byte/flash 314,643 byte; toàn bộ 26 kịch bản host và 14 cấu hình sai
đạt. Chưa upload, chưa quan sát LED hoặc đo GPIO/motor thật của bản này.

Người dùng yêu cầu bật thử sau khi báo đã đấu dây, dùng AIN1=D13/AIN2=D14 và
hộp 4 pin AA ghi 1.5V/viên. Cấu hình này được áp dụng theo yêu cầu đó; không
có nghĩa định mức điện, cực nguồn, GND hay dòng kẹt đã được đo/xác minh.

## Cấu hình thật

`include/config.h`: MOTOR_HARDWARE_ENABLED=true. PWMA=18, AIN1=13, AIN2=14,
PWMB=19, BIN1=21, BIN2=22, STBY=23. VCC driver dùng 3V3; GND driver/ESP32/
âm nguồn motor dùng chung; VM là nguồn motor riêng. Không nối hộp AA vào
ESP32 3V3. Mức 6V là danh định nếu các cell nối tiếp, không phải số đo.

Kênh A tạm được coi là trái, B phải. Hai cờ INVERT vẫn false. Chỉ kiểm tra
được bánh/chiều thật khi thử trên giá đỡ. Sensor/calibration vẫn tắt;
RUN line-follow bị chặn. Không thêm tự chạy lúc cấp điện.

Độ phân giải PWM 8bit (0..255), lệnh giới hạn ±150; đây không phải RPM hay
giới hạn dòng điện. Theo yêu cầu mới, timeout manual **10000ms (10 giây)**;
LINE_FOLLOW vẫn có giới hạn riêng 5 giây. Timeout, lỗi lệnh và đổi mode đều
dừng + DISARM. ARM không quay bánh; STOP dừng/coast nhưng giữ ARM, DISARM
dừng và hủy quyền. Bánh có thể còn trôi sau khi bỏ công suất.

## Nạp và thử

1. Ngắt VM, nối USB, kiểm tra từng dây theo bảng trên; VCC/GND không đảo.
   Không cấp điện khi mối nối chập/lỏng hoặc cực nguồn chưa rõ.
2. Build và nạp bằng PlatformIO, dùng cổng ESP32 thật từ `pio device list`:

   ```bash
   pio run -e esp32dev
   pio run -e esp32dev -t upload --upload-port /dev/cu.TEN_CONG_THUC
   pio device monitor -p /dev/cu.TEN_CONG_THUC -b 115200 --echo --eol LF
   ```

   TEN_CONG_THUC là chỗ thay tên, không phải cổng có thật. Khi cần vào chế
   độ nạp: giữ BOOT, nhấn/thả EN, upload; thả BOOT lúc bắt đầu ghi flash.
3. Sau reset, mong MOTOR READY và startup STOPPED/DISARMED. STATUS phải
   ARMED=0/RUN=0. READY chỉ xác nhận GPIO/LEDC, không xác minh dây/nguồn.
4. Kê, cố định hai bánh khỏi mặt bàn, tránh tay/dây vào bánh; chuẩn bị ngắt
   nguồn VM bằng công tắc/rút nguồn vật lý. Thông số motor/dòng kẹt còn chưa
   biết: không coi PWM thấp là bảo vệ dòng; cần đối chiếu định mức phần cứng.
5. Khi phần điện phù hợp để cấp VM, gửi từng lệnh, không dán cả chuỗi:
   MODE MOTOR_TEST → ARM → LEFT 30. Chờ timeout khoảng 10 giây rồi STATUS;
   phải RUN=0/ARMED=0. Ghi bánh thực tế nhận lệnh.
6. ARM lại → RIGHT 30; chờ dừng. Sau khi từng bánh đúng mới ARM → BOTH 30.
   Nếu 30 chưa quay, dừng kiểm tra nguồn/dây/ma sát trước khi thử 50; không
   tăng liên tục hoặc giữ trục để ép motor.
7. Chờ trục đứng yên trước khi thử số âm: ARM → LEFT -30, rồi thử riêng
   RIGHT -30. ARM lại mỗi xung. STOP/DISARM trước khi đổi hướng/cấu hình.
   Có thể STOP sớm; không cần chờ đủ 10 giây, nhất là khi motor không quay.
8. Thử STOP, DISARM và timeout. Sau DISARM hoặc timeout, LEFT 30 phải bị
   chặn đến khi ARM lại. ! dừng và DISARM khi ký tự tới ESP32, Enter để xóa dòng.

Nếu LEFT làm quay bánh phải, ngắt VM rồi xác nhận lại A/B trước khi đổi
LEFT_MOTOR_IS_CHANNEL_A=false. Đúng bánh nhưng ngược chiều thì đổi riêng cờ
INVERT tương ứng khi VM đã ngắt, build/nạp lại. Không bật LINE_FOLLOW lần này.
Đứt cáp Serial không thay thế cắt VM; STOP phần mềm không phải công tắc khẩn.

## Trở lại bản chỉ chẩn đoán USB

Đặt MOTOR_HARDWARE_ENABLED=false, build rồi upload với VM ngắt. Có thể giữ
bảng GPIO; khóa false khiến motorInit không cấp GPIO/PWM motor. Khi đó ARM
bị BLOCKED là đúng, HELP và STATUS vẫn hoạt động.

Chưa upload bản motor-enabled và chưa xác nhận bánh quay trong lần chỉnh này.

## Kết quả phần mềm cho cấu hình này

- `pio run -e esp32dev`: SUCCESS, 8.54 giây; Arduino-ESP32 3.3.12;
  RAM 22,512/327,680 byte, flash 314,275/1,310,720 byte.
- `python3 tests/run_host_tests.py`: 26 kịch bản chạy đạt, 14 cấu hình sai
  bị từ chối đúng dự kiến. Hai kịch bản bổ sung kiểm tra chính bảng chân
  đang dùng (driver và arming); vẫn kiểm tra riêng bản motor disabled.
- Không bypass kiểm tra GPIO, ARM, timeout, mode hoặc điều kiện RUN.
  Test host dùng GPIO/Serial giả, không chứng minh điện áp, dòng hay bánh quay.
- Kiểm tra cổng trong phiên này chưa thấy USB ESP32. Không chạy upload,
  không gửi ARM hoặc lệnh chuyển động tới phần cứng. Không push/merge.

Commit gợi ý: `feat(motor): configure user-requested manual bench wiring`.
