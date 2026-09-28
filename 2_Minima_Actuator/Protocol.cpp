/**********************************************************************
 * Protocol.cpp
 *
 * 这个文件实现 Minima 侧固定长度协议帧的收发和校验。
 *********************************************************************/

// 引入协议头文件。
#include "Protocol.h"

// 实现协议接收器构造函数。
ProtocolReceiver::ProtocolReceiver()
    // 初始化接收索引为 0。
    : rxIndex(0) {}

// 实现 CRC8 计算函数。
uint8_t calculateCRC8(const uint8_t* data, uint8_t len) {
    // 初始化 CRC 寄存器。
    uint8_t crc = 0x00;
    // 逐字节处理输入数据。
    for (uint8_t i = 0; i < len; i++) {
        // 把当前字节异或进 CRC。
        crc ^= data[i];
        // 对当前字节执行 8 次移位迭代。
        for (uint8_t j = 0; j < 8; j++) {
            // 如果最高位为 1。
            if (crc & 0x80) {
                // 就左移并与多项式 0x07 异或。
                crc = (crc << 1) ^ 0x07;
            } else {
                // 否则只执行左移。
                crc <<= 1;
            }
        }
    }
    // 返回最终 CRC 值。
    return crc;
}

// 实现协议帧发送函数。
void sendFrame(Stream& port, uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2) {
    // 创建固定长度帧缓冲区。
    uint8_t frame[FRAME_LENGTH];

    // 写入帧头。
    frame[0] = FRAME_HEADER;
    // 写入帧长度。
    frame[1] = FRAME_LENGTH;
    // 写入命令字节。
    frame[2] = cmd;
    // 写入第一个数据字节。
    frame[3] = data0;
    // 写入第二个数据字节。
    frame[4] = data1;
    // 写入第三个数据字节。
    frame[5] = data2;
    // 计算并写入 CRC8。
    frame[6] = calculateCRC8(&frame[2], 4);
    // 写入帧尾。
    frame[7] = FRAME_TAIL;

    // 把整帧发送到指定串口。
    port.write(frame, FRAME_LENGTH);
}

// 实现协议接收轮询函数。
bool ProtocolReceiver::poll(Stream& port, ProtocolFrame& frame) {
    // 只要串口里还有数据，就持续读取。
    while (port.available()) {
        // 读取一个字节。
        const uint8_t byte = static_cast<uint8_t>(port.read());

        // 如果当前期待帧头，而收到的不是帧头，就丢弃它。
        if (rxIndex == 0 && byte != FRAME_HEADER) {
            continue;
        }

        // 把当前字节写入接收缓冲区。
        rxBuffer[rxIndex++] = byte;

        // 如果当前还没有收满一帧，就继续读取。
        if (rxIndex < FRAME_LENGTH) {
            continue;
        }

        // 判断长度字段是否正确。
        const bool validLength = rxBuffer[1] == FRAME_LENGTH;
        // 判断帧尾是否正确。
        const bool validTail = rxBuffer[7] == FRAME_TAIL;
        // 重新计算 CRC。
        const uint8_t crc = calculateCRC8(&rxBuffer[2], 4);
        // 判断 CRC 是否匹配。
        const bool validCrc = crc == rxBuffer[6];

        // 一帧处理结束后重置接收索引。
        rxIndex = 0;

        // 如果长度、帧尾或 CRC 错误，就丢弃这一帧。
        if (!validLength || !validTail || !validCrc) {
            ++badFrames;   // 诊断计数
            continue;
        }

        // 把命令字节拷贝到输出结构体。
        frame.cmd = rxBuffer[2];
        // 把第一个数据字节拷贝到输出结构体。
        frame.data0 = rxBuffer[3];
        // 把第二个数据字节拷贝到输出结构体。
        frame.data1 = rxBuffer[4];
        // 把第三个数据字节拷贝到输出结构体。
        frame.data2 = rxBuffer[5];
        // 返回 true，表示成功收到一帧有效命令。
        return true;
    }

    // 如果没有收到完整有效帧，就返回 false。
    return false;
}
