#ifndef THUR__SERIAL_HPP_
#define THUR__SERIAL_HPP_

#include <string>
#include <vector>

//#define THUR_SERIAL_BUFFER_MAX_SIZE           256
//#define THUR_SERIAL_SERIAL_FRAME_MAX_SIZE     128
#define THUR_SERIAL_BUFFER_MAX_SIZE           200
#define THUR_SERIAL_SERIAL_FRAME_MAX_SIZE     100


namespace thur
{
    enum class return_type : std::uint8_t
    {
        SUCCESS = 0,
        ERROR = 1
    };

    struct SerialHdlcFrame
    {
        uint8_t data[THUR_SERIAL_SERIAL_FRAME_MAX_SIZE];
        size_t length;
    };

    class ThurSerialPort
    {
    public:
        ThurSerialPort();
        ~ThurSerialPort();
        
        return_type open(const std::string & port_name);
        return_type close();
        return_type read_frames(std::vector<SerialHdlcFrame>& frames);
        return_type write_frame(const uint8_t* data, size_t size);
        bool is_open() const;
        char* to_string();

    protected:
        void encode_byte(uint8_t data);
        void decode_byte(uint8_t data, std::vector<SerialHdlcFrame>& frames);
        uint16_t crc_update(uint16_t crc, uint8_t data);

    private:
        int serial_port_;
        uint8_t rx_buffer_[THUR_SERIAL_BUFFER_MAX_SIZE];
        uint8_t rx_frame_buffer_[THUR_SERIAL_SERIAL_FRAME_MAX_SIZE];
        size_t rx_frame_length_;
        uint16_t rx_frame_crc_;
        bool rx_frame_escape_;
        uint8_t tx_frame_buffer_[THUR_SERIAL_SERIAL_FRAME_MAX_SIZE];
        size_t tx_frame_length_;
        uint16_t tx_frame_crc_;

    };
}

#endif  // THUR__SERIAL_HPP_
