// VideoStreamNvenc header protocol based on
//
// udp_header.hpp
// ~~~~~~~~~~~~~~~
//
// Copyright (c) 2012 Kevin D. Conley (kcon at stanford dot edu)
// Based on icmp_header.hpp (c) 2003-2010 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef UBITRACK_DEVICE_COMM_VIDEOSTREAM_VIDEOSTREAMNVENCPROTOCOL_H
#define UBITRACK_DEVICE_COMM_VIDEOSTREAM_VIDEOSTREAMNVENCPROTOCOL_H

#include <istream>
#include <ostream>
#include <algorithm>

// The wire format of a UDP header is (size of fields should be optimized..):
//
// 0               8               16                             31
// +-------------------------------+------------------------------+      ---
// |                                                              |       ^
// |                                                              |       |
// |                                                              |       |
// +                            timestamp                         +    8 bytes
// |                                                              |       |
// |                                                              |       |
// |                                                              |       v
// +-------------------------------+------------------------------+      ---
// |                               |                              |       ^
// |          width                |          height              |       |
// |                               |                              |       |
// +-------------------------------+------------------------------+       |
// |                               |                              |       |
// |          codec                |          format              |    12 bytes
// |                               |                              |       |
// +-------------------------------+------------------------------+       |
// |                                                              |       |
// |                            framesize                         |       |
// |                                                              |       v
// +-------------------------------+------------------------------+      ---
//



class VideostreamNvencProtocol
{
public:
    VideostreamNvencProtocol() { std::fill(rep_, rep_ + sizeof(rep_), 0); }

    unsigned long long timestamp() const { return decode64(0); }
    unsigned short width() const { return decode16(4); }
    unsigned short height() const { return decode16(6); }
    unsigned short codec() const { return decode16(8); }
    unsigned short format() const { return decode16(10); }
    unsigned short framesize() const { return decode32(12); }

    void timestamp(unsigned long long n) { encode64(0, n); }
    void width(unsigned short n) { encode16(4, n); }
    void height(unsigned short n) { encode16(6, n); }
    void codec(unsigned short n) { encode16(8, n); }
    void format(unsigned short n) { encode16(10, n); }
    void framesize(unsigned short n) { encode32(12, n); }

    void to_string()
    {
        unsigned int i;
        printf("{");
        for (i = 0; i < sizeof(rep_); i++) {
            printf(i == sizeof(rep_) - 1 ? "%.2X}\n" : "%.2X, ", rep_[i]);
        }
    }

    friend std::istream& operator>>(std::istream& is, VideostreamNvencProtocol& header)
    { return is.read(reinterpret_cast<char*>(header.rep_), 20); }

    friend std::ostream& operator<<(std::ostream& os, const VideostreamNvencProtocol& header)
    { return os.write(reinterpret_cast<const char*>(header.rep_), 20); }

    size_t size() { return sizeof(rep_); }

    void copy_to(std::vector<uint8_t>& buffer) {
        // buffer must be large enough ...
        unsigned int i;
        for (i = 0; i < sizeof(rep_); i++) {
            buffer.at(i) = rep_[i];
        }
    }

    void copy_from(const std::vector<uint8_t>& buffer) {
        // buffer must be large enough ...
        unsigned int i;
        for (i = 0; i < sizeof(rep_); i++) {
            rep_[i] = buffer.at(i);
        }
    }

private:
    unsigned short decode16(int i) const
    {
        unsigned short n = 0;
        n =   ((rep_[i+0] <<  8) & 0x000000000000FF00U)
            | ( rep_[i+1]        & 0x00000000000000FFU);

        return n;
    }

    unsigned int decode32(int i) const
    {
        unsigned int n = 0;
        n =     ((rep_[i+0] << 24) & 0x00000000FF000000U)
              | ((rep_[i+1] << 16) & 0x0000000000FF0000U)
              | ((rep_[i+2] <<  8) & 0x000000000000FF00U)
              | ( rep_[i+3]        & 0x00000000000000FFU);

        return n;
    }

    unsigned long long decode64(int i) const
    {
        unsigned long long n = 0;
        n = (  (rep_[i+0] << 56) & 0xFF00000000000000U)
            | ((rep_[i+1] << 48) & 0x00FF000000000000U)
            | ((rep_[i+2] << 40) & 0x0000FF0000000000U)
            | ((rep_[i+3] << 32) & 0x000000FF00000000U)
            | ((rep_[i+4] << 24) & 0x00000000FF000000U)
            | ((rep_[i+5] << 16) & 0x0000000000FF0000U)
            | ((rep_[i+6] <<  8) & 0x000000000000FF00U)
            | ( rep_[i+7]        & 0x00000000000000FFU);

        return n;
    }

    void encode16(int i, unsigned short num) {
        for(int j = 0; j < 2; j++) rep_[i+j] = num >> (8-1-j)*8;
    }

    void encode32(int i, unsigned int num) {
        for(int j = 0; j < 4; j++) rep_[i+j] = num >> (8-1-j)*8;
    }

    void encode64(int i, unsigned long long num) {
        for(int j = 0; j < 8; j++) rep_[i+j] = num >> (8-1-j)*8;
    }

    unsigned char rep_[20];
};

#endif //UBITRACK_DEVICE_COMM_VIDEOSTREAM_VIDEOSTREAMNVENCPROTOCOL_H
