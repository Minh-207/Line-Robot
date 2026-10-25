# Line-Robot[README (1).md](https://github.com/user-attachments/files/33922636/README.1.md)
# Hiểu code robot dò line và cách chỉnh PD/PID

Robot dùng 8 cảm biến quang học để tìm vị trí line, sau đó điều chỉnh chênh lệch lệnh hai động cơ để quay về giữa đường. Hãy mở **`So_do_chuc_nang_robot.pdf`** khi đọc: trang 1 nối phần cứng với các hàm, trang 2 thể hiện trạng thái chạy và tác dụng của bộ điều khiển. **`So_do_tong_quan_robot.svg`** là bản vector của trang tổng quan, có thể sửa bằng Inkscape hoặc phần mềm hỗ trợ SVG.

> **Điểm cần biết trước:** bản gốc là **PD**, chưa có thành phần I. `lastPos` chưa được cập nhật nên phần D chưa hoạt động đúng; quá trình học màu có thể chia cho 0. Vì vậy, nên đọc mục 10 trước khi chạy để chỉnh thông số. Các đoạn sửa trong tài liệu là đề xuất để bạn áp dụng, không có thay đổi nào được thực hiện vào file `.ino` gốc.

## Mục lục

1. [Cách robot hoạt động](#1-cách-robot-hoạt-dộng)
2. [Phần cứng và chân kết nối](#2-phần-cứng-và-chân-kết-nối)
3. [Bản đồ các khối code](#3-bản-đồ-các-khối-code)
4. [Khởi động và học màu](#4-khởi-động-và-học-màu)
5. [Từ 8 cảm biến đến vị trí line](#5-từ-8-cảm-biến-đến-vị-trí-line)
6. [Hiểu P, I, D trên chính robot này](#6-hiểu-p-i-d-trên-chính-robot-này)
7. [Từ giá trị điều khiển đến hai bánh xe](#7-từ-giá-trị-điều-khiển-đến-hai-bánh-xe)
8. [Các trạng thái và cách xử lý mất line](#8-các-trạng-thái-và-cách-xử-lý-mất-line)
9. [Các biến và hàm phụ](#9-các-biến-và-hàm-phụ)
10. [Những điểm nên sửa trước khi tinh chỉnh](#10-những-điểm-nên-sửa-trước-khi-tinh-chỉnh)
11. [Quy trình chỉnh PD từng bước](#11-quy-trình-chỉnh-pd-từng-bước)
12. [Nhìn hiện tượng để tìm nguyên nhân](#12-nhìn-hiện-tượng-để-tìm-nguyên-nhân)
13. [Muốn phát triển tiếp thì sửa ở đâu?](#13-muốn-phát-triển-tiếp-thì-sửa-ở-đâu)
14. [Phạm vi kiểm tra và tài liệu tham khảo](#14-phạm-vi-kiểm-tra-và-tài-liệu-tham-khảo)

## 1. Cách robot hoạt động

Mỗi chu kỳ, robot thực hiện những công việc sau:

1. **Nhìn đường:** đọc điện áp của 8 cảm biến A0 đến A7.
2. **Hiểu đường:** tạo hai kiểu dữ liệu: mức tín hiệu đã cân bằng `sensorPID[]` và mẫu 8 bit `sensor`.
3. **Tính độ lệch:** ước lượng tâm line đang nằm ở đâu so với tâm dãy cảm biến.
4. **Tính mức đánh lái:** bộ PD tạo giá trị `servoPwm`.
5. **Chọn cách chạy:** `runforwardline()` dùng mẫu bit để chọn bám line, tìm lại line hoặc dừng theo từng trường hợp.
6. **Điều khiển bánh:** cộng/trừ mức đánh lái vào tốc độ nền, rồi xuất PWM và tín hiệu chiều quay.
7. **Nhìn lại:** robot đã di chuyển nên tín hiệu cảm biến đổi; chu trình lặp lại.

Đây là **điều khiển phản hồi theo vị trí line**. Code không đọc encoder và không đo tốc độ thực của bánh xe. Giá trị 200 là lệnh điều khiển, không phải 200 vòng/phút hay 200 cm/s.

`read_sensor()` được gọi trong ngắt Timer2; `loop()` dùng kết quả để ra lệnh động cơ. Hai phần này không phải hai bộ xử lý chạy song song: ngắt tạm dừng `loop()`, xử lý rồi quay lại.

## 2. Phần cứng và chân kết nối

### 2.1. Những gì biết được từ code

| Bộ phận | Tên trong code | Chân |
| --- | --- | --- |
| 8 cảm biến quang analog | `sensorValue[0..7]` | A0, A1, A2, A3, A4, A5, A6, A7 |
| Điều khiển chiều động cơ trái | `LDIR` | D9 |
| Điều khiển PWM động cơ trái | `LPWM` | D5 |
| Điều khiển chiều động cơ phải | `RDIR` | D11 |
| Điều khiển PWM động cơ phải | `RPWM` | D6 |
| Nút SW1 | `BUTTON1` | D7 |
| Nút SW2 | `BUTTON2` | D8 |
| Nút SW3 | `BUTTON3` | D12 |
| Còi | `BUZZER` | D4 |
| LED đỏ / xanh lá / xanh dương | `RED` / `GREEN` / `BLUE` | D3 / D2 / D13 |
| Chân mang tên servo | `SERVO` | D10, **chưa được sử dụng** |

Code dùng thanh ghi AVR và Timer2, đồng thời cần A6/A7. Điều này gợi ý một board AVR có 8 kênh analog, chẳng hạn Nano classic; **file không xác nhận chính xác board**. Không thể nạp nguyên code này cho ESP32 hoặc Nano Every chỉ bằng cách đổi tên board. Thông số ADC 0..1023 và ví dụ chu kỳ 16 MHz dưới đây giả định nền tảng AVR tương ứng [1][3].

Không có sơ đồ driver nên chưa thể kết luận chính xác chế độ phanh, thả trôi hoặc cực tính PWM của mạch công suất. Sơ đồ đi kèm thể hiện **kết nối chức năng**, không phải sơ đồ đấu dây nguồn.

### 2.2. Quy ước trái/phải cần kiểm tra

Trong hình minh họa, **nhìn từ trên xuống, đầu robot hướng lên**, A0 ở trái và A7 ở phải. Đây là giả định để giải thích; phải kiểm tra vị trí lắp thực tế.

| Kênh ADC | Chỉ số `j` | Bit trong `sensor` | Trọng số vị trí | Độ lệch nếu chỉ kênh này thấy line |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0 | bit 7 | 0 | -3500 |
| A1 | 1 | bit 6 | 1000 | -2500 |
| A2 | 2 | bit 5 | 2000 | -1500 |
| A3 | 3 | bit 4 | 3000 | -500 |
| A4 | 4 | bit 3 | 4000 | +500 |
| A5 | 5 | bit 2 | 5000 | +1500 |
| A6 | 6 | bit 1 | 6000 | +2500 |
| A7 | 7 | bit 0 | 7000 | +3500 |

Thứ tự bit trên là chắc chắn từ phép dịch bit trong code. Còn A0 có thật sự ở phía trái hay không thì phải kiểm tra trên robot. Nếu lắp ngược thứ tự, hướng điều khiển có thể ngược.

## 3. Bản đồ các khối code

| Khối / hàm | Dòng bản gốc | Công việc | Dữ liệu quan trọng |
| --- | --- | --- | --- |
| Khai báo chân và biến | 1-31 | Đặt tên chân, tạo bộ nhớ và thông số | `sensor`, `servoPwm`, `kp`, `kd` |
| `setup()` | 32-65 | Khởi tạo chân, dừng bánh, đọc EEPROM, phát âm, bật lấy mẫu | `start=0`, `isCalib=1` ở cuối |
| `timer_init()` | 66-76 | Cấu hình Timer2 tạo ngắt định kỳ | `TCNT2=0xB2`, bộ chia 1024 |
| `ISR(TIMER2_OVF_vect)` | 77-81 | Đọc cảm biến, tính điều khiển, tăng bộ đếm | Gọi `read_sensor()`, tăng `cnt` |
| `loop()` | 82-134 | Học màu, nhận nút và chạy máy trạng thái | `start`, `pattern`, `cnt` |
| `runforwardline()` | 136-221 | Giải mẫu bit và chọn lệnh chạy | `sensor`, `line`, `servoPwm` |
| `updateLine()` | 223-245 | Thu min/max ADC và tính ngưỡng từng mắt | `black_value`, `white_value`, `compare_value` |
| `RGB()` | 246-269 | Bật/tắt các chân LED | 0: đỏ; 1: xanh lá; 2: xanh dương |
| `readEeprom()` | 270-274 | Đọc 8 ngưỡng đã lưu trước đó | `EEPROM.read(i)*4` |
| `handleAndSpeed()` | 278-290 | tốc độ và mức đánh lái | `speedLeft`, `speedRight` |
| `speed_run()` | 292-311 | phát xung tín hiệu GPIO/PWM | D5/D9 và D6/D11 |
| `read_sensor()` | 313-359 | Đọc ADC, cân bằng, đóng gói bit, tính vị trí và PD | `sensorPID[]`, `sensor`, `servoPwm` |
| `beep()` | 360-364 | Còi báo | Có `delay(timer)` |
| `sensorMask()` | 365-367 | bit cảm biến | `sensor & mask`; chưa có chỗ gọi |

## 4. Khởi động và học màu

### 4.1. Trình tự khi cấp nguồn

`setup()` đặt lệnh hai bánh bằng 0, đọc ngưỡng cũ từ EEPROM, bật Serial ở **9600 baud**, phát 3 âm khởi động, cấu hình Timer2 rồi chuyển sang học màu. Khi `start==0`, `loop()` lặp lại `updateLine()` và kiểm tra ba nút.

Trong lúc học màu, di chuyển ngang robot qua line và nền để **từng mắt đều nhìn thấy cả hai vùng**, giữ độ cao cảm biến gần như lúc chạy. Nếu một mắt chỉ nhìn một màu, khoảng hiệu chuẩn của mắt đó có thể bằng 0 hoặc quá nhỏ.

### 4.2. `updateLine()` 

Với từng mắt `j`, code lấy:

```cpp
black_value[j]   // giá trị ADC nhỏ nhất đã quan sát
white_value[j]   // giá trị ADC lớn nhất đã quan sát
compare_value[j] = (black_value[j] + white_value[j]) / 2;
```

Tên `black_value` và `white_value` dễ gây hiểu nhầm: theo phép so sánh, chúng thực chất là **min và max ADC**. Chưa đo thì không thể khẳng định min luôn là đen và max luôn là trắng.

Ví dụ một mắt có min=200, max=800 thì ngưỡng là 500. Trong code gốc, ADC >500 tạo bit 1; ADC <=500 tạo bit 0.

### 4.3.Chức năng SW

| Nút | Giá trị được gán trong bản gốc | Ý nghĩa |
| --- | ---: | --- |
| SW1 | `speed_run_forward=0` | Chuyển sang chạy với tốc độ nền 0; PD vẫn có thể tạo lệnh quay |
| SW2 | `speed_run_forward=200` | Chuyển sang chạy, tăng dần tốc độ nền lên 200 |
| SW3 | `speed_run_forward=255` | Chuyển sang chạy, tăng dần tốc độ nền lên 255 |

Chú thích “50” của SW2 và “70” của SW3 không khớp phép gán 200/255. Thay mỗi dòng `speed_run_forward=200` trong `setup()` **không đủ**, vì khi bấm nút, giá trị bị ghi đè ở dòng 99 hoặc 106.

SW1 **không phải nút dừng khẩn cấp**. Với tốc độ nền 0, công thức trộn vẫn có thể cho một bánh tiến và một bánh lùi. Đặc biệt, bản gốc còn có nhánh gọi thẳng `speed_run(255,255)` khi mất line và `line==0`.

Sau khi chạy, code không còn kiểm tra nút trong khối `while(start==0)`. Chưa có chức năng bấm lại để dừng hoặc học màu lại. Ba `if` độc lập cũng có nghĩa: bấm nhiều nút cùng lúc thì phép gán ở phía sau có thể ghi đè phép gán trước.

### 4.4.lưu học màu vào EEPROM 

**Hiện tại không.** Đoạn `EEPROM.write(...)` ở dòng 235-243 nằm trong comment `/* ... */`. Chú thích “bấm SW1 để lưu” không phản ánh hành vi hiện tại.

`readEeprom()` chỉ đọc 8 ngưỡng, không đọc min/max cần cho chuẩn hóa. Vì vậy, chỉ mở lại đoạn lưu ngưỡng cũng chưa đủ để bỏ học màu ở lần bật máy sau. Muốn làm vậy phải lưu/đọc cả bộ min, max và kiểm tra dữ liệu hợp lệ.

## 5. Từ 8 cảm biến đến vị trí line

### 5.1. Chu kỳ lấy mẫu

`TCNT2=0xB2=178`, nên cần `256-178=78` nhịp để tràn. Với cấu hình AVR phù hợp, bộ chia 1024 và clock 16 MHz:

```text
T_tick = 1024 / 16.000.000 = 64 micro giây
T_mẫu danh định = 78 * 64 micro giây = 4,992 mili giây
f_mẫu danh định ≈ 200,32 Hz
```

Đây là tính toán theo cấu hình [3], không phải kết quả đo trên robot. Có độ trễ vào ngắt và thời gian xử lý. Timer được nạp lại **ở đầu** ISR, vì vậy không cộng đơn giản toàn bộ thời gian đọc ADC vào 4,992 ms. Nếu ISR quá lâu, nhịp xử lý có thể bị ảnh hưởng.

Trên AVR tương ứng, một lần `analogRead()` mất khoảng 100 micro giây [1], nên 8 lần đọc đã chiếm khoảng 0,8 ms, chưa kể chuẩn hóa và số thực. Không đặt `Serial.print()` hoặc `delay()` vào ISR. Khi cần tối ưu, có thể để ISR đặt cờ lấy mẫu và xử lý ở `loop()`, nhưng phải bảo đảm mỗi mẫu chỉ cập nhật PD một lần và đo lại chu kỳ thực.

### 5.2. Nhánh analog: `sensorPID[]`

Sau học màu, code kẹp giá trị ADC vào khoảng min/max rồi chuẩn hóa:

```cpp
sensorPID[j] = map(sensorValue[j], black_value[j], white_value[j], 0, 1000);
```

Ví dụ min=200, max=800:

| ADC | Giá trị chuẩn hóa |
| ---: | ---: |
| 200 | 0 |
| 500 | 500 |
| 800 | 1000 |

Mục đích là giảm ảnh hưởng việc mỗi mắt có mức điện áp khác nhau dù cùng nhìn một màu.

**Cực tính phải đồng nhất:** bản gốc gán trọng số lớn cho ADC cao, đồng thời đặt bit 1 khi ADC cao hơn ngưỡng. Do đó, code chỉ đúng với mục tiêu dò line khi **line cần bám tạo ADC cao hơn nền**. Chú thích nhắc line trắng chưa đủ để xác nhận phần cứng thực tế.

Nếu line cần bám tạo ADC thấp hơn nền, cần đảo **cả hai nhánh**:

```cpp
// Nhánh analog dùng để tính tâm line:
sensorPID[j] = 1000 - map(rawClamped, minADC, maxADC, 0, 1000);

// Nhánh bit dùng để nhận mẫu đường:
bool onLine = rawADC < compare_value[j];
```

Đây là minh họa với tên biến giải thích; mục 10.2 có hàm thay thế dùng tên biến thực. Nếu chỉ đảo bit mà không đảo `sensorPID[]`, phần nhận mẫu đường và phần tính đánh lái sẽ hiểu hai vùng khác nhau.

### 5.3. Nhánh nhị phân: `sensor`

Mỗi bit tương ứng một mắt. Phép `temp=temp<<1` làm A0 đi vào bit 7 và A7 đi vào bit 0.

| Mẫu bit | Cảm biến đang có bit 1 | Ý nghĩa theo quy ước A0 trái, A7 phải |
| --- | --- | --- |
| `00011000` | A3, A4 | Line giữa dãy |
| `00000100` | A5 | Line phía phải |
| `00100000` | A2 | Line phía trái |
| `00000000` | Không có mắt nào vượt ngưỡng | Code gọi là mất line |
| `11111111` | Tất cả mắt đều vượt ngưỡng | Vùng rộng/giao nhau hoặc cực tính không đúng; không đủ để xác định loại đường |

`sensor` dùng để phân loại tình huống. `sensorPID[]` dùng để tính vị trí liên tục hơn. Chúng có vai trò khác nhau.

### 5.4. Tính tâm line và độ lệch

Code tính trung bình có trọng số:

```text
sum      = tổng sensorPID[j]
avg      = tổng (sensorPID[j] * j * 1000)
position = avg / sum
error    = position - 3500
```

`position` nằm trong 0..7000 khi có tín hiệu hợp lệ. Tâm dãy là `(0+7000)/2=3500`. Vì vậy `error` nằm trong -3500..+3500. Trong bản gốc, biến `i` chính là `error`, không phải thành phần tích phân I.

Ví dụ A3=A4=1000, các mắt khác bằng 0:

```text
position = (1000*3000 + 1000*4000) / 2000 = 3500
error = 3500 - 3500 = 0
```

Nếu chỉ A5=1000 thì position=5000, error=+1500: line ở phía A5. Nếu chỉ A2=1000 thì position=2000, error=-1500.

Điều kiện bắt buộc là `sum>0`. Nếu tổng bằng 0 thì vị trí không xác định; cần xử lý mất line, không thực hiện phép chia.

## 6. Hiểu P, I, D trên chính robot này

### 6.1. Công thức mong muốn

PID đầy đủ có ba thành phần:

```text
P: phản ứng với độ lệch hiện tại
I: phản ứng với độ lệch tích lũy qua thời gian
D: phản ứng với mức thay đổi của độ lệch
```

Code này chỉ khai báo `kp`, `kd`; không có `ki` hay biến cộng dồn sai số. Sau khi sửa lịch sử sai số, công thức tương ứng là:

```text
e[k]  = position[k] - 3500
de[k] = e[k] - e[k-1]
u[k]  = (kp * e[k] + kd * de[k]) / 60
```

`u` được đưa vào `servoPwm`. Đây là **mức hiệu chỉnh chênh lệch lệnh hai bánh**, không phải góc servo theo độ. Không có lệnh điều khiển servo D10 trong file.

### 6.2. P tác động ra sao?

`kp` càng lớn thì cùng một độ lệch, robot sửa hướng càng mạnh. Ví dụ `kp=2.5`, `kd=0`, error=+600:

```text
u = 2.5 * 600 / 60 = +25
```

Với tốc độ nền 100, trước giới hạn đầu ra, bánh trái nhận 125 và bánh phải nhận 75. Theo quy ước lắp bánh tiến bình thường, robot quay sang phải để đuổi theo line nằm phía phải.

P nhỏ quá: phản ứng chậm, dễ cắt ra ngoài cua. P lớn quá: dễ lắc qua lại, nhất là khi tốc độ nền cao.

### 6.3. D tác động ra sao?

D nhìn xem sai số đang tăng hay giảm. Ví dụ `kp=2.5`, `kd=1.0`, sai số trước là +900, hiện tại +600:

```text
de = 600 - 900 = -300
u = (2.5*600 + 1.0*(-300)) / 60 = +20
```

So với P đơn thuần cho +25, D làm giảm mức quay khi robot đang tiến gần tâm line. Nếu sai số còn tăng nhanh, D sẽ làm mức sửa mạnh hơn. D quá lớn hoặc tín hiệu nhiễu có thể làm bánh giật.

Code dùng chênh lệch **theo mẫu**, không chia cho chu kỳ `Ts`. Vì vậy `kd` không thể chép nguyên từ thư viện PID dùng đạo hàm theo giây. Đổi chu kỳ lấy mẫu thì phải chỉnh lại D. Trong cách viết này, hệ số đạo hàm theo giây tương đương là `(kd/60)*Ts`; hệ số P tương đương là `kp/60`.

### 6.4. Vì sao D trong bản gốc chưa đúng?

Đoạn gốc:

```cpp
iP = kp * i;
iD = kd * (lastPos - i);
iRet = iP - iD;
```

Hai dấu trừ không phải lỗi: đại số cho `kp*i + kd*(i-lastPos)`. Vấn đề là **không có chỗ nào cập nhật `lastPos=i`**.

`lastPos` là biến toàn cục nên khởi đầu bằng 0, rồi giữ nguyên. Với `kp=2.5`, `kd=0.1`, đầu ra trước giới hạn gần bằng `2.6*error`, sai khác nhỏ do ép về số nguyên. Như vậy tăng `kd` trong bản gốc chủ yếu làm tăng độ mạnh giống P, chưa tạo tác dụng dựa trên sai số mẫu trước.

### 6.5. Hệ số `/60` và chặn lệch một phía

`servoPwm=iRet/60` làm nhỏ đầu ra về thang lệnh động cơ. Nếu giảm 60 xuống 30 thì cả tác động P và D đều tăng gấp đôi. Khi mới chỉnh, giữ nguyên thang này để dễ so sánh.

Đoạn sau trong bản gốc **không phải giới hạn đối xứng**:

```cpp
if (iRet < -4000) iRet = 0;
```

Ví dụ với `lastPos=0` và bộ số gốc:

| Error | `iRet` trước điều kiện | `servoPwm` sau điều kiện và chia 60 |
| ---: | ---: | ---: |
| -2000 | -5200 | 0 |
| -1000 | -2600 | -43 |
| +1000 | +2600 | +43 |
| +2000 | +5200 | +86 |

Ở một phía, robot có thể mất hẳn lệnh sửa khi lệch nhiều. Cần dùng giới hạn hai phía, ví dụ `-MAX_TURN..+MAX_TURN`, thay cho việc đưa riêng phía âm về 0.

### 6.6. Khi nào mới cần I?

Chưa cần thêm I khi cảm biến, hướng quay hoặc PD còn lỗi. Nếu sau khi sửa cơ khí và hiệu chuẩn, robot vẫn lệch cố định về một phía, có thể xem xét bù hai bánh hoặc thêm I nhỏ. Nếu thêm I, cần giới hạn tích lũy và xóa tích lũy khi học màu, dừng hoặc mất line để tránh tích phân tăng quá mức. Thêm một biến `ki` mà không thêm phép cộng dồn sai số thì chưa thành PID.

## 7. Từ giá trị điều khiển đến hai bánh xe

### 7.1. `handleAndSpeed(angle, speed1)`

Trong hàm này, `angle` chính là `servoPwm`, `speed1` là tốc độ nền:

```cpp
speedLeft  = speed1 + angle;
speedRight = speed1 - angle;
```

| Mức sửa `angle` | Bánh trái | Bánh phải | Hướng quay dự kiến |
| ---: | --- | --- | --- |
| 0 | Bằng tốc độ nền | Bằng tốc độ nền | Đi thẳng |
| Dương | Tăng | Giảm | Sang phải |
| Âm | Giảm | Tăng | Sang trái |

Các hướng này giả định lệnh dương làm cả hai bánh đẩy robot tiến và A0 ở trái. Nếu kiểm tra thực tế sai, cần sửa thứ tự cảm biến hoặc quy ước động cơ trước khi tăng gain.

Trước khi cộng/trừ, code giảm tốc độ nền nếu một bánh sẽ vượt +255. Ví dụ speed1=240, angle=40:

```text
240+40 > 255 nên speed1 được hạ thành 255-40=215
speedLeft=255; speedRight=175
```

Vì vậy lệnh nền 255 không có nghĩa hai bánh luôn giữ 255 khi vào cua. Hàm gốc chỉ xử lý giới hạn phía trên theo cách này, chưa có bảo vệ đầy đủ phía -255 khi cho `angle` quá lớn. Khi đề xuất `MAX_TURN=80` và tốc độ nền trong 0..255, lệnh trộn nằm trong miền -255..255; vẫn cần kiểm tra driver trước khi dùng lệnh âm.

### 7.2. `speed_run(left, right)`

Lệnh không âm dùng `DIR=LOW`, PWM bằng chính giá trị lệnh. Lệnh âm dùng `DIR=HIGH`, PWM bằng `255+lệnh`.

| Lệnh một bánh | DIR được xuất | Đối số truyền cho `analogWrite()` |
| ---: | --- | ---: |
| +100 | LOW | 100 |
| 0 | LOW | 0 |
| -10 | HIGH | 245 |
| -100 | HIGH | 155 |
| -255 | HIGH | 0 |

Đây là cách biến đổi trong file, **chưa thể kết luận độ mạnh chạy lùi từ con số duty** khi thiếu sơ đồ driver. Một số mạch dùng PWM đảo ở nhánh lùi, một số mạch dùng PWM thông thường. Không tự đổi thành `abs(speed)` chỉ vì thấy công thức khác quen thuộc.

`analogWrite()` trên AVR đặt chân PWM thành OUTPUT bên trong hàm [2], nên việc bản gốc không gọi `pinMode(LPWM,OUTPUT)` riêng không tự động là lỗi. Tín hiệu này đi đến driver; chân vi điều khiển không cấp công suất trực tiếp cho động cơ.

## 8. Các trạng thái và cách xử lý mất line

### 8.1. `pattern` là trạng thái làm việc

| Trạng thái | Hành động trong bản gốc | Chuyển tiếp |
| --- | --- | --- |
| Chưa chạy: `start=0` | Học màu, đợi nút | Nút bất kỳ đặt `start=1`, `pattern=10` |
| `pattern=10` | Chạy bám line với tốc độ nền `cnt` tăng dần | Đủ tốc độ thì sang 11 |
| `pattern=11` | Bám line bằng `runforwardline(speed_run_forward)` | Lặp lại; chưa có giải map tự động |
| `pattern=12` | Chưa có hành động | Không có chuyển tiếp trong case này; lệnh bánh cũ có thể vẫn được giữ |
| `pattern=100` | Gọi lệnh dừng và đổi LED | Chưa có đoạn nào trong luồng hiện tại chuyển đến 100 |
| Giá trị khác | Gán lại `pattern=11` | Vòng sau quay về bám line |

`cnt` tăng 1 mỗi ngắt, nên tăng tốc từ 0 lên 200 mất danh định khoảng `200*4,992 ms = 0,9984 s`; lên 255 khoảng `1,273 s`. Đây là tăng lệnh PWM theo thời gian, không phải đo gia tốc thực. Các nhánh gọi thẳng 255 trong `runforwardline()` có thể bỏ qua quá trình tăng mềm này.

### 8.2. `line` khác với `error`

`line` chỉ là **nhãn hướng rời rạc**: -3,-2,-1,0,+1,+2,+3. Nó được cập nhật khi mẫu bit khớp các `case`, chủ yếu để quyết định tìm lại line về phía nào. Nó không tham gia công thức PD.

Ví dụ: mẫu `00000100` gán `line=+1`, mẫu `00100000` gán `line=-1`. Một số mẫu rộng lệch về A7 gán +3; lệch về A0 gán -3. Khi gặp mẫu ở giữa, code gán `line=0`, tức là **không giữ lại mãi hướng lệch cuối cùng trước đó**.

### 8.3. Các trường hợp đặc biệt phải hiểu đúng

| Tình huống | Hành vi thực tế của bản gốc |
| --- | --- |
| `sensor=00000000`, `line>0` | Gọi `speed_run(tocdo,-10)` để tìm về phía được ghi nhớ |
| `sensor=00000000`, `line<0` | Gọi `speed_run(-10,tocdo)` |
| `sensor=00000000`, `line==0` | Gọi thẳng `speed_run(255,255)` |
| `sensor=11111111` | Gọi 255/255, nhưng **thiếu `break`**, nên chạy tiếp nhóm giữa và ghi đè bằng lệnh PD |
| Mẫu không có trong `switch` | `handleAndSpeed(0,0)`, đặt lệnh hai bánh bằng 0 |

Đường giao nhau, vùng line rộng hoặc nhiều vạch có thể sinh mẫu chưa được liệt kê. Không nên chỉ tăng Kp nếu robot dừng ở đúng một kiểu giao lộ; cần xem mẫu `sensor` và bổ sung logic trạng thái.

## 9. Các biến và hàm phụ

| Tên | Nên hiểu là gì? |
| --- | --- |
| `sensorValue[8]` | ADC từng mắt; trong bản gốc còn bị kẹp min/max sau khi chạy, nên không luôn là ADC thô |
| `sensorPID[8]` | Tín hiệu đã chuẩn hóa 0..1000 dùng để tính vị trí |
| `black_value`, `white_value` | Min/max đo khi hiệu chuẩn, không chắc tương ứng màu theo tên |
| `compare_value` | Ngưỡng để tạo bit 0/1 cho từng mắt |
| `sensor` | Mẫu 8 bit của cả dãy |
| `lastPos` | Được đặt tên như vị trí cũ, nhưng công thức cần **sai số mẫu trước**; bản gốc chưa cập nhật |
| `servoPwm` | Giá trị sửa hướng hai bánh |
| `isCalib` | 1: đang học màu; 0: cho phép chuẩn hóa |
| `speed_run_forward` | Lệnh tốc độ nền |
| `cnt` | Bộ đếm nhịp ngắt dùng tăng tốc ban đầu |
| `start` | Cờ đã rời chế độ học màu hay chưa |
| `pattern` | Trạng thái chương trình |
| `addr` | Địa chỉ đầu cho đoạn ghi EEPROM đang bị comment |

`RGB(0/1/2)` lần lượt xuất mức bật đỏ/xanh lá/xanh dương theo quy ước mức LOW kích hoạt của code. `RGB(4)` rơi vào `default` và xuất tắt cả ba, không phải màu thứ tư.

`beep(100)` có `delay(100)`. Do `updateLine()` gọi nó khi `sensor==0xff`, vòng học màu có thể bị chặn 100 ms dù comment của `loop()` nói không dùng delay. Việc in liên tục 8 số ở 9600 baud cũng làm vòng học màu chậm hơn; ngắt vẫn tiếp tục nếu không bị tắt.

`sensorMask(mask)` trả về `sensor & mask`, ví dụ mask `0b11110000` chỉ giữ nhóm bit của A0..A3. Hàm chưa được dùng trong luồng hiện tại.

## 10. Những điểm nên sửa trước khi tinh chỉnh

### 10.1. Danh sách theo mức ảnh hưởng

| Vấn đề trong bản gốc | Ảnh hưởng | Hướng sửa |
| --- | --- | --- |
| Tính `avg/sum` cả khi học màu | `sensorPID[]` ban đầu bằng 0, gây chia cho 0 | Khi học màu chỉ đọc ADC/tạo bit, chưa tính PD |
| Chuẩn hóa khi min=max | `map()` chia cho 0 | Kiểm tra khoảng hiệu chuẩn từng mắt trước khi chạy |
| Không cập nhật `lastPos` | D không dùng lịch sử sai số | Lưu sai số hiện tại sau mỗi mẫu hợp lệ; bỏ D ở mẫu đầu |
| `kp`,`kd` bị gán trong mỗi lần đọc | Chỉnh ở nơi khác sẽ bị ghi đè | Đưa thông số về một chỗ; xóa phép gán trong ISR |
| `iRet<-4000` thì về 0 | Mất khả năng sửa mạnh một phía | Chặn đối xứng đầu ra |
| Mất line với `line==0` chạy 255/255 | Bỏ qua tốc độ đã chọn | Khi thử PD, thay nhánh mất line bằng dừng |
| `11111111` thiếu `break` | Lệnh đầu bị ghi đè | Chọn hành vi rõ ràng rồi thêm `break` |
| `cnt` chia sẻ với ISR nhưng không `volatile` | Có thể đọc không đúng kỳ vọng khi tối ưu | Khai báo `volatile`; chụp dữ liệu nguyên tử |
| `servoPwm` là `int` nhiều byte trên AVR 8 bit | `volatile` không bảo đảm đọc đủ cùng một lần cập nhật | Dùng `ATOMIC_BLOCK` cho bản chụp trong `loop()` |
| Cực tính line chưa xác nhận | Dò nhầm nền hoặc điều khiển sai | Đo ADC và đảo đồng thời nhánh bit/analog khi cần |

Phép trừ `3500` trong biểu thức gốc thực hiện với số không dấu trước khi ép kiểu. Trên AVR thường cho kết quả mong đợi theo cách chuyển kiểu của công cụ, nhưng nên viết rõ `int error=(int)(avg/sum)-3500` để tránh phụ thuộc cách chuyển số không dấu sang số có dấu. Khi tăng gain, các biến `int iP/iD/iRet` 16 bit cũng có thể vượt miền trước khi chia 60; giữ phép toán dạng `float` đến sau giới hạn giúp tránh vấn đề đó.

### 10.2. Đề xuất sửa phần đo và PD

Các bước dưới đây là **một nhóm chỉnh sửa cần áp dụng cùng nhau**. Chúng giữ cấu trúc hiện tại, thêm kiểm tra hiệu chuẩn và sửa PD. Giá trị khởi đầu chỉ phục vụ thử chậm; không phải bộ thông số tối ưu đã đo trên robot.

**A. Thay hai khai báo `float kp; float kd;` bằng khối thông số này, và thêm include nguyên tử:**

```cpp
#include <util/atomic.h>

float kp = 1.0f;
float kd = 0.0f;                  // ban đầu chỉnh P trước
const float PD_SCALE = 60.0f;
const int MAX_TURN = 80;          // giới hạn độ sửa hai phía
const int MIN_CALIB_SPAN = 30;    // ví dụ; điều chỉnh theo tín hiệu đo
const bool LINE_IS_HIGH = true;  // false nếu line có ADC thấp hơn nền
volatile bool calibrationValid = false;
bool pdPrimed = false;           // chỉ được dùng trong read_sensor()
```

Nếu độ tương phản ADC của mắt nào nhỏ hơn 30, bản đề xuất sẽ không cho chạy. Không hạ ngưỡng này để che cảm biến hỏng; hãy nhìn giá trị thực và độ nhiễu trước.

**B. Trong `setup()`, sau âm khởi động và trước `timer_init()`, khởi tạo học màu:**

```cpp
noTone(BUZZER);   // kết thúc tone trước khi dùng Timer2 cho lấy mẫu
for (int j = 0; j < 8; ++j) {
  black_value[j] = 1023;
  white_value[j] = 0;
}
isCalib = 1;
timer_init();
```

Khối này **thay** cặp `timer_init(); isCalib=1;` cũ, không thêm một lần gọi Timer2 thứ hai. Trong `updateLine()`, xóa dòng `if (black_value[i] == 0) black_value[i] = 1100;` vì đã khởi tạo min rõ ràng; ADC=0 có thể là giá trị hợp lệ.

Trên Arduino AVR tương ứng, `tone()` sử dụng Timer2 [4]. Sau khi khởi tạo timer lấy mẫu, không gọi lại `tone()` nếu chưa tổ chức lại việc dùng timer. `beep()` hiện tại chỉ điều khiển GPIO nhưng vẫn có delay.

**C. Thay toàn bộ hàm `read_sensor()` gốc bằng hàm dưới đây.** Không giữ lại `kp=2.5; kd=0.1;` hay điều kiện `iRet<-4000` cũ.

```cpp
void read_sensor() {
  const uint8_t pins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
  uint8_t bits = 0;

  for (int j = 0; j < 8; ++j) {
    sensorValue[j] = analogRead(pins[j]);
    bool onLine = LINE_IS_HIGH
      ? sensorValue[j] > compare_value[j]
      : sensorValue[j] < compare_value[j];
    bits = (uint8_t)((bits << 1) | (onLine ? 1 : 0));
  }
  sensor = bits;

  if (isCalib) {
    calibrationValid = false;
    servoPwm = 0;
    lastPos = 0;
    pdPrimed = false;
    return;                       // chưa chuẩn hóa, chưa chia avg/sum
  }

  unsigned long avg = 0;
  unsigned int sum = 0;
  for (int j = 0; j < 8; ++j) {
    int lo = (int)black_value[j];
    int hi = (int)white_value[j];
    if (hi - lo < MIN_CALIB_SPAN) {
      calibrationValid = false;
      servoPwm = 0;
      lastPos = 0;
      pdPrimed = false;
      return;
    }
    int raw = constrain((int)sensorValue[j], lo, hi);
    long q = map(raw, lo, hi, 0, 1000);
    sensorPID[j] = (unsigned int)(LINE_IS_HIGH ? q : 1000 - q);
    sum += sensorPID[j];
    avg += (unsigned long)sensorPID[j] * (j * 1000UL);
  }
  calibrationValid = true;

  // Mẫu không có mắt vượt ngưỡng không được đưa vào PD.
  if (sum == 0 || bits == 0) {
    sensor = 0;
    servoPwm = 0;
    lastPos = 0;
    pdPrimed = false;
    return;                       // xử lý dừng/tìm line ở runforwardline()
  }

  int error = (int)(avg / sum) - 3500;
  int deltaError = pdPrimed ? error - lastPos : 0;
  float u = (kp * error + kd * deltaError) / PD_SCALE;
  lastPos = error;
  pdPrimed = true;
  u = constrain(u, -(float)MAX_TURN, (float)MAX_TURN);
  servoPwm = (int)u;
}
```

Sau sửa, `sensorValue[]` được giữ là ADC thô; không kẹp trực tiếp lên mảng đó nữa. `calibrationValid` chỉ cho biết khoảng min/max hợp lệ, không có nghĩa đang thấy line.

**D. Thêm bản chụp và chặn hiệu chuẩn lỗi ở đầu `runforwardline()`:**

```cpp
uint8_t sensorNow;
int correctionNow;
bool calOK;
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  sensorNow = sensor;
  correctionNow = servoPwm;
  calOK = calibrationValid;
}
if (!calOK) {
  speed_run(0, 0);
  return;
}
```

Trong chính hàm đó, đổi `switch(sensor)` thành `switch(sensorNow)` và thay **mọi** đối số `servoPwm` trong `handleAndSpeed(servoPwm,tocdo)` thành `correctionNow`. Như vậy mẫu bit và mức sửa được lấy từ cùng một lần chụp, không bị ngắt chen giữa lúc đọc.

**E. Trong giai đoạn chỉnh PD, thay toàn bộ hai nhánh đặc biệt bằng các case dừng rõ ràng:**

```cpp
case 0b00000000:       // thay cả khối tìm line cũ
  speed_run(0, 0);
  RGB(2);
  break;

case 0b11111111:       // thay case 11111111 cũ, đặt trước nhóm giữa
  speed_run(0, 0);     // dừng để xem mẫu đường rộng/giao nhau
  line = 0;
  RGB(0);
  break;
```

Đây là **thay đổi hành vi được đề xuất để thử PD**. Sơ đồ trạng thái và mục 8 mô tả bản gốc. Khi cần giải map, thiết kế lại hành vi giao lộ/mất line rồi mới thay các case dừng này bằng tìm line có giới hạn thời gian.

**F. Sửa bộ đếm tăng tốc và đọc giá trị nhiều byte:**

Thay `int cnt=0;` bằng `volatile unsigned int cnt=0;`. Thay ISR bằng:

```cpp
ISR(TIMER2_OVF_vect) {
  TCNT2 = 0xB2;
  read_sensor();
  if (cnt < 255) ++cnt;  // dừng đếm ở 255, tránh tăng/tràn mãi
}
```

Trong `case 10`, dùng bản chụp thay cho việc đọc `cnt` nhiều lần:

```cpp
case 10: {
  unsigned int rampNow;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { rampNow = cnt; }
  if (rampNow < (unsigned int)speed_run_forward) {
    runforwardline((int)rampNow);
  } else {
    runforwardline(speed_run_forward);
    pattern = 11;
  }
  break;
}
```

Thay mỗi phép đặt lại `cnt=0` khi bấm nút bằng `ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { cnt=0; }`. Khi học màu, cũng nên chụp đủ 8 giá trị ADC trong một khối nguyên tử ngắn rồi mới cập nhật min/max và in Serial **ở ngoài** khối đó. Không bao cả `updateLine()` hoặc toàn bộ phần in Serial trong đoạn tắt ngắt.

**G. Đổi tốc độ thử ở đúng chỗ.** Với thử ban đầu, có thể đặt giá trị 100 trong nhánh SW2, thay cho 200. Chưa dùng SW3=255 để tìm gain. 100 chỉ là mức ví dụ; cần chọn đủ để hai bánh quay đều theo phần cứng thực tế.

### 10.3. Những điểm bản sửa minh họa chưa thay thế được

Nhóm sửa trên chưa nhận diện giao lộ, chưa đo tốc độ bánh và chưa bổ sung nút dừng trong lúc chạy. Chưa có ảnh/sơ đồ phần cứng để xác nhận hướng A0/A7, cực tính quang hoặc cách driver nhận PWM đảo. Kiểm tra các điểm này trên robot là điều kiện để diễn giải đúng Kp/Kd.

## 11. Quy trình chỉnh PD từng bước

### Bước 1. Kiểm tra hướng cảm biến và bánh xe

Trước lần chạy dưới sàn đầu tiên, thử lệnh động cơ ngắn khi bánh không chạm đất để xác nhận lệnh dương thực sự tạo hướng tiến ở cả hai bên và có cách ngắt nguồn thuận tiện. Không dùng SW1 của bản gốc như nút bảo đảm đứng yên.

Mở Serial ở 9600 baud lúc học màu, đặt từng mắt lần lượt lên line và nền. Ghi lại ADC, xác định `LINE_IS_HIGH` trong bản đề xuất. Kiểm tra A0..A7 theo thứ tự lắp thực tế và khoảng min/max của cả 8 mắt.

### Bước 2. Học màu đúng điều kiện chạy

Giữ độ cao cảm biến ổn định, quét dãy qua cả line lẫn nền vài lần. Thực hiện lại khi thay mặt đường, độ cao cảm biến hoặc ánh sáng làm tín hiệu thay đổi đáng kể. Không cố chữa dữ liệu hiệu chuẩn sai bằng gain cao.

### Bước 3. Bắt đầu với P, tạm để D=0

Sau khi áp dụng nhóm sửa mục 10.2, bộ số **minh họa** để bắt đầu là:

| Thông số | Ví dụ ban đầu | Sửa ở đâu? |
| --- | ---: | --- |
| Tốc độ nền thử | 100 | Phép gán trong nhánh SW2 |
| `kp` | 1.0 | Khối thông số toàn cục mới |
| `kd` | 0.0 | Khối thông số toàn cục mới |
| `PD_SCALE` | 60.0 | Giữ cố định khi tìm Kp/Kd |
| `MAX_TURN` | 80 | Giới hạn đầu ra mới |

Nếu bánh không quay đều ở 100, trước hết xác định miền lệnh vận hành phù hợp. Nếu robot quay ra xa line ngay lập tức, dừng và sửa dấu điều khiển/thứ tự cảm biến; tăng Kp sẽ không giải quyết việc sai hướng.

Chạy cùng một đoạn thẳng có cua nhẹ. Tăng Kp từng bước nhỏ, ví dụ 0.2 hoặc 10-20% mỗi lần. Nếu sửa quá chậm thì tăng; khi bắt đầu lắc rõ thì hạ khoảng 20-30% để lấy mốc thử D. Mỗi lần chỉ đổi một thông số.

### Bước 4. Thêm D để giảm lắc

Giữ Kp và tốc độ nền. Thử Kd từ nhỏ, chẳng hạn 0.2, 0.5, 1.0, rồi điều chỉnh theo kết quả; đây không phải dãy bắt buộc. Vì code lấy chênh lệch sai số theo mẫu và chia 60, Kd quá nhỏ có thể gần như không thể hiện sau khi ép lệnh về `int`.

Quan sát robot vượt tâm và quay qua lại bao nhiêu lần. Tăng D đến khi bớt lắc nhưng vẫn vào cua được. Nếu bánh giật dù đường thẳng, xem nhiễu cảm biến và giảm D. Không đổi chu kỳ Timer2 đồng thời với gain.

### Bước 5. Tăng tốc độ nền sau khi đã bám ổn

Tăng 10-20 đơn vị PWM mỗi lần và chạy lại đúng đoạn thử. Tốc độ cao hơn thường cần tinh chỉnh lại Kp/Kd và cách giảm tốc ở cua. Không thể bảo đảm bộ số chạy tốt ở 100 sẽ chạy tốt ở 255.

Nếu `servoPwm` liên tục bằng `+MAX_TURN` hoặc `-MAX_TURN`, đầu ra đang bão hòa. Tăng gain thêm có thể không tăng mức sửa thực tế; hãy giảm tốc độ hoặc kiểm tra giới hạn/cơ khí. Khi tăng `MAX_TURN`, phải kiểm tra lại miền lệnh hai bánh và hành vi driver khi lệnh âm.

### Bước 6. Thử các tình huống riêng

Sau đoạn cơ bản, thử lần lượt cua trái, cua phải, cua gắt, mất line và giao lộ. Trong bản đề xuất, mất line và toàn bit 1 đang dừng có chủ ý. Logic giải map cần được kiểm thử riêng với PD.

Ghi một bảng thử để tránh chỉnh theo cảm giác:

| Lần thử | Tốc độ nền | Kp | Kd | Mặt đường / pin | Hiện tượng |
| --- | ---: | ---: | ---: | --- | --- |
| 1 | 100 | 1.0 | 0.0 | Điền thực tế | Điền kết quả |
| 2 | 100 | 1.2 | 0.0 | Giữ điều kiện gần giống | Điền kết quả |
| 3 | 100 | Giữ Kp đã chọn | 0.5 | Giữ điều kiện gần giống | Điền kết quả |

### Log gì khi cần xem sâu hơn?

Nên xem `sensor`, sai số và mức sửa. Sau bản sửa, `lastPos` chứa sai số mẫu hợp lệ gần nhất. Đọc bản chụp bằng `ATOMIC_BLOCK`, sau đó in ngoài khối này, khoảng 10 lần/giây trong `loop()` bằng `millis()` thay cho delay. Đừng in trong ISR. Khi chỉ cần học màu, 8 cột ADC sẵn có trong `updateLine()` đã hữu ích.

## 12. Nhìn hiện tượng để tìm nguyên nhân

| Hiện tượng | Kiểm tra trước | Sau khi dữ liệu đúng mới chỉnh |
| --- | --- | --- |
| Robot chạy ra xa line | Thứ tự A0/A7, hướng dương hai bánh, cực tính line | Sửa dấu/quy ước trước |
| Chỉ cua một bên tốt | Điều kiện `iRet<-4000`, hai động cơ, min/max từng mắt | Sau sửa mới cân chỉnh gain |
| Lắc qua lại trên đường thẳng | Hiệu chuẩn, độ rơ, `lastPos` có cập nhật không | Giảm Kp hoặc tăng D vừa phải |
| Sửa chậm, trôi ra cua | Có bão hòa không, cảm biến còn thấy line không | Tăng Kp vừa phải hoặc giảm tốc |
| Bánh giật nhanh | Nhiễu ADC, dây, ánh sáng, chu kỳ lấy mẫu | Giảm D; cân nhắc lọc sau khi đo |
| Mất line rồi lao thẳng | Nhánh `sensor==0 && line==0` gốc | Sửa chính sách mất line, không chỉ gain |
| Đứng ở giao lộ | Mẫu bit có rơi vào `default` không | Thêm logic trạng thái/map |
| Bấm SW1 mà bánh vẫn quay | Tốc độ nền 0 không đồng nghĩa mức sửa 0 | Tạo chế độ dừng riêng |
| Thay `kp` trong setup mà không hiệu quả | `read_sensor()` gán lại 2.5 mỗi mẫu | Chỉ giữ một nơi đặt thông số |
| Chạy lỗi ngay lúc học màu | `sum==0`, min=max, ISR tính PD quá sớm | Bổ sung các điều kiện chặn |
| Chỉnh tốt lúc đầu rồi thay pin lại khác | Nguồn thay đổi, code chưa đóng vòng tốc độ bánh | Thử cùng mức nguồn; cân nhắc điều khiển tốc độ có encoder |

## 13. Muốn phát triển tiếp thì sửa ở đâu?

| Mục tiêu | Vị trí chính | Điều kiện đi kèm |
| --- | --- | --- |
| Đổi tốc độ bằng nút | Khối BUTTON2/BUTTON3 trong `loop()` | Không chỉ sửa mặc định trong setup |
| Chỉnh Kp/Kd | Khối thông số mới hoặc dòng 350-351 ở bản gốc | Ưu tiên sửa lỗi D trước |
| Dò màu có ADC thấp | Tạo bit và chuẩn hóa trong `read_sensor()` | Đảo cả hai nhánh |
| Rẽ ở ngã ba | `pattern=11`, thêm trạng thái và dùng mẫu `sensor` | Quy định điều kiện vào/ra, thời gian tối đa |
| Dừng khi gặp vạch đích | Logic mẫu bit + chuyển `pattern=100` | Phân biệt vạch đích với giao lộ bằng trình tự/timing |
| Học màu lại bằng nút | Bổ sung xử lý nút khi đang chạy | Dừng bánh, đặt lại min/max và lịch sử PD |
| Bỏ học màu mỗi lần bật | `readEeprom()` và phần ghi mới | Lưu đủ min/max, kiểm tra phiên bản/dữ liệu hợp lệ |
| Thêm I | Khối tính điều khiển | Giới hạn tích phân, reset khi mất line/dừng |
| Đổi board/chu kỳ lấy mẫu | Timer, ADC, ánh xạ chân | Đối chiếu phần cứng, đo lại chu kỳ và chỉnh lại D |

## 14. Phạm vi kiểm tra và tài liệu tham khảo

Tài liệu và sơ đồ được đối chiếu với nội dung `Code_Frame.ino` đã cung cấp. Ví dụ số học thể hiện công thức và tác dụng của giới hạn; các đoạn sửa là hướng dẫn áp dụng, chưa phải firmware đã chạy thử trên robot. Chưa có model driver, sơ đồ đấu dây, ảnh dãy cảm biến hoặc log thực nghiệm nên chưa thể xác định bộ Kp/Kd tối ưu. Các giả định phần cứng được ghi rõ tại chỗ sử dụng.

Các nguồn chính thức dùng để kiểm tra hành vi nền tảng, đối chiếu ngày 25/10/2026:

1. [Arduino - analogRead](https://github.com/arduino/reference-en/blob/master/Language/Functions/Analog%20IO/analogRead.adoc): miền ADC và thời gian đọc tham khảo.
2. [Arduino AVR core - wiring_analog.c](https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring_analog.c): cách đọc ADC và xuất PWM của core AVR.
3. [Microchip - ATmega328P datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf): Timer/Counter2, chế độ Normal và chọn bộ chia clock. [Arduino Nano](https://docs.arduino.cc/hardware/nano): ví dụ board AVR phù hợp, không phải xác nhận board robot đang dùng.
4. [Arduino AVR core - Tone.cpp](https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/Tone.cpp): việc dùng timer của `tone()` trên AVR tương ứng.

**Thứ tự nên làm:** xác nhận cảm biến và động cơ → sửa phép tính/giới hạn → học màu → chọn tốc độ thấp phù hợp → tìm Kp → thêm Kd → tăng tốc → phát triển giải map.
