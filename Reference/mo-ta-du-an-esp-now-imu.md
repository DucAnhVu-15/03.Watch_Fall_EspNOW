# Mô tả dự án: Truyền & Đồng bộ dữ liệu IMU qua ESP-NOW

## 1. Ý tưởng tổng quan

Hệ thống gồm 2 thiết bị đeo trên cơ thể (ví dụ ở hai cổ tay), giao tiếp trực tiếp với nhau qua **ESP-NOW**:

- **Slave**: đọc dữ liệu IMU (accelerometer), gửi định kỳ sang Master.
- **Master**: nhận dữ liệu từ Slave, kết hợp với dữ liệu IMU của chính nó, lưu xuống Flash dưới dạng CSV, và cho phép trích xuất dữ liệu ra PC (qua USB).

**Mục tiêu chính:**
- Truyền và đồng bộ dữ liệu giữa 2 thiết bị.
- Sai lệch thời gian giữa 2 thiết bị không quá 2 samples (~2ms).
- Dùng tính năng đồng bộ thời gian có sẵn của ESP-NOW để tự động căn chỉnh lệch thời gian, không cần làm thủ công.

---

## 2. Sơ đồ ý tưởng

```
[Cổ tay trái]              [Cổ tay phải]                  [Lưu trữ / xử lý]
   Slave     <---ESP-NOW--->   Master
(Accelerometer)         (RTC + Accelerometer)   --->    ESP32 --> Flash
                               |
                          cmd: start/stop
```

- Cả hai cổ tay đều đeo thiết bị: **tay trái là Slave** (chỉ đọc Accelerometer), **tay phải là Master** (có RTC + Accelerometer).
- Hai thiết bị trao đổi trực tiếp qua ESP-NOW: dữ liệu RTC (đồng bộ thời gian), dữ liệu Accelerometer, và lệnh điều khiển (start/stop).
- Master đóng vai trò trung tâm, sau khi nhận dữ liệu từ Slave sẽ đẩy xuống ESP32 → Flash để lưu trữ.

---

## 3. Mô tả chi tiết

### 3.1. Kết nối ESP-NOW
- Hai thiết bị kết nối trực tiếp qua ESP-NOW.
- Một thiết bị là **Master** (trung tâm), một thiết bị là **Slave** (cung cấp dữ liệu).
- Bật tính năng đồng bộ thời gian của ESP-NOW để tự động căn chỉnh lệch thời gian giữa 2 bên (không cần can thiệp thủ công).

### 3.2. Thiết bị Slave
- Đọc dữ liệu từ IMU (ghi vào RAM), tần số lấy mẫu **100Hz**.
- Định kỳ gửi dữ liệu sang Master qua ESP-NOW, chu kỳ **1s/2s/5s** tùy theo dung lượng RAM và độ rộng khung truyền ESP-NOW.
- Nếu khung truyền là 100 mẫu thì gửi dư thêm **10 mẫu cũ** (overlap) để hỗ trợ căn chỉnh lệch thời gian nếu cần.
- Dữ liệu gửi đi gồm: **Time-Stamp bắt đầu khung thu** + **dữ liệu IMU**.
- Chưa cần tối ưu năng lượng triệt để, nhưng nên khai thác **IRQ của IMU** để giảm dòng tiêu thụ.
- Có thể nhận lệnh **start/stop** từ Master để điều khiển quá trình thu thập.

### 3.3. Thiết bị Master
- Đọc dữ liệu tương tự Slave, lưu vào RAM.
- Không ngủ — luôn sẵn sàng chờ nhận dữ liệu từ Slave (sẽ tối ưu năng lượng sau).
- Khi nhận dữ liệu từ Slave: lấy dữ liệu tương ứng từ RAM của chính mình (kèm thêm 10 mẫu overlap tương tự).
- Lưu dữ liệu vào Flash dưới dạng **CSV**, dùng **FlashFS** (cần tìm thêm thư viện phù hợp cho ESP32).
- Có thể lưu thành 1 file chung hoặc 2 file riêng (Master/Slave), mỗi file có cột Time-stamp của từng thiết bị kèm dữ liệu.
- Cần có cơ chế kết nối để lấy dữ liệu từ Flash sang PC, có thể qua USB.
- Giai đoạn đầu: chỉ lưu dữ liệu **raw**, chưa cần xử lý/đồng bộ gộp dữ liệu ngay. Sau khi thuật toán trên PC hoạt động tốt mới đưa xuống MCU.

### 3.4. Cấu trúc file CSV (tham khảo)

**Master:**
```
timestamp_ms,sample_idx,acc_x_raw,acc_y_raw,acc_z_raw,is_overlap
171800000000,0,1024,-45,16384,1
171800000010,1,1020,-48,16390,1
...
171800000100,10,1035,-50,16370,0
171800000110,11,1040,-42,16375,0
```

**Slave:**
```
timestamp_ms,frame_idx,sample_idx,acc_x_raw,acc_y_raw,acc_z_raw,is_overlap
171800000000,12,0,1012,-60,16200,1
171800000010,12,1,1015,-58,16210,1
...
171800000100,12,10,1030,-62,16190,0
171800000110,12,11,1028,-65,16195,0
```

---

## 4. Danh sách đầu việc (Task list)

### 4.1. Thiết lập ESP-NOW (nền tảng chung)
- [ ] Thiết lập giao tiếp ESP-NOW giữa Master và Slave
- [ ] Bật/cấu hình tính năng đồng bộ thời gian (time sync) của ESP-NOW
- [ ] Kiểm tra sai lệch thời gian giữa 2 thiết bị ≤ 2 samples (~2ms)
- [ ] Định nghĩa cấu trúc gói tin (timestamp, số mẫu, dữ liệu IMU...)

### 4.2. Thiết bị Slave
- [ ] Viết driver đọc dữ liệu IMU (accelerometer), 100Hz
- [ ] Ghi dữ liệu vào RAM (buffer)
- [ ] Gửi định kỳ (1s/2s/5s) dữ liệu sang Master qua ESP-NOW
- [ ] Gửi dư 10 mẫu cũ (overlap) mỗi khung
- [ ] Gắn Time-Stamp bắt đầu khung vào gói gửi đi
- [ ] Khai thác IRQ của IMU để giảm tiêu thụ dòng
- [ ] Cài đặt cơ chế nhận lệnh start/stop từ Master

### 4.3. Thiết bị Master
- [ ] Đọc dữ liệu IMU tương tự Slave, lưu vào RAM
- [ ] Giữ thiết bị luôn sẵn sàng nhận dữ liệu (chưa ngủ)
- [ ] Khi nhận gói từ Slave: lấy dữ liệu tương ứng từ RAM Master (kèm 10 mẫu overlap)
- [ ] Tích hợp thư viện FlashFS phù hợp cho ESP32
- [ ] Ghi dữ liệu ra Flash dạng CSV theo cấu trúc tham khảo
- [ ] Quyết định lưu chung 1 file hay tách 2 file
- [ ] Xây dựng cơ chế truyền dữ liệu từ Flash ra PC (USB MSC/CDC/UART — cần chọn giao thức)

### 4.4. Giai đoạn sau
- [ ] Test đồng bộ timestamp giữa 2 nguồn dữ liệu khi ghép file trên PC
- [ ] Xử lý/đồng bộ dữ liệu raw (khi thuật toán PC ổn định mới đưa xuống MCU)
- [ ] Tối ưu năng lượng cho Master (cho phép ngủ giữa các chu kỳ nhận)
