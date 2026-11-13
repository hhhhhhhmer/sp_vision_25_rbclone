// test_serial.cpp - 简化版本
#include <iostream>
#include <iomanip>
#include <vector>
#include <serial/serial.h>
#include <unistd.h>

int main() {
    serial::Serial ser;
    
    try {
        ser.setPort("/dev/ttyACM1");
        ser.setBaudrate(460800);
        
        serial::Timeout timeout = serial::Timeout::simpleTimeout(2);
        ser.setTimeout(timeout);
        ser.open();
        
        std::cout << "Serial port opened successfully" << std::endl;
        
        std::vector<uint8_t> buffer;
        int frame_count = 0;
        const size_t FRAME_SIZE = 29;  // 数据长度为42字节
        
        while (true) {
            if (ser.available()) {
                std::vector<uint8_t> new_data;
                size_t available = ser.available();
                ser.read(new_data, available);
                
                // 将新数据添加到缓冲区
                buffer.insert(buffer.end(), new_data.begin(), new_data.end());
                
                // 处理缓冲区中的数据
                bool found_frame = false;
                for (size_t i = 0; i < buffer.size(); ++i) {
                    // 查找0x53帧头
                    if (buffer[i] == 0x5a && (buffer.size() - i) >= FRAME_SIZE && buffer[i+1] == 0x53) {
                        found_frame = true;
                        std::cout << "\n[Frame " << ++frame_count << "] ";
                        
                        // 输出帧内容
                        for (size_t j = 0; j < FRAME_SIZE; ++j) {
                            std::cout << std::hex << std::setw(2) << std::setfill('0')
                                      << static_cast<int>(buffer[i + j]) << " ";
                            if ((j + 1) % 16 == 0 && j < FRAME_SIZE - 1) {
                                std::cout << std::endl << "            ";
                            }
                        }
                        std::cout << std::dec << std::endl;
                        
                        // 移除已处理的数据
                        buffer.erase(buffer.begin(), buffer.begin() + i + FRAME_SIZE);
                        break;  // 退出循环，重新开始查找
                    }
                }
                
                // 如果没有找到完整帧，但缓冲区太大，清除部分数据
                if (!found_frame && buffer.size() > FRAME_SIZE * 2) {
                    // 保留最后一部分数据（避免一直累积）
                    size_t keep_size = std::min(buffer.size(), FRAME_SIZE * 2);
                    buffer.erase(buffer.begin(), buffer.end() - keep_size);
                    std::cout << "Buffer cleared, kept " << buffer.size() << " bytes" << std::endl;
                }
                
            }
            
            usleep(5000);  // 5ms
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    return 0;
}