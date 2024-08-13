

#ifndef SPARK_BASE_COMMON_SERIAL_H
#define SPARK_BASE_COMMON_SERIAL_H
#define RECV_BUFFER_SIZE      512

#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <boost/asio.hpp>
#include <algorithm>
#include <iterator>
namespace NxSparkBase {

class SparkSerial {
public:
    SparkSerial(std::string dev_name, uint32_t baudrate);

    ~SparkSerial();

    int OpenSerial();
    int CloseSerial();

    int WriteBuffer(uint8_t *buf, uint16_t length);

    int GetDataGram(unsigned char* r_buffer, int *length);
    void hex_printf(unsigned char *buf, int len);

private:
    std::string serial_name_;
    uint32_t baudrate_;
    int fd_;
};

}

#endif // SPARK_BASE_COMMON_SERIAL_H
