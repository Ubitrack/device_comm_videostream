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
// |                               |                              |       ^
// |          fixed 0xAA55         |          seq id              |    4 bytes
// |                               |                              |       v
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

// additional packets only use a minimal sequence header
// 0               8               16                             31
// +-------------------------------+------------------------------+      ---
// |                               |                              |       ^
// |          fixed 0x55AA         |          seq id              |    4 bytes
// |                               |                              |       v
// +-------------------------------+------------------------------+      ---

#define FRAME_HEADER_ID 0xAA55U
#define PACKET_HEADER_ID 0x55AAU
#define UDP_PACKET_SIZE 1400 // was 4096 //udp pack size; note that OSX limits < 8100 bytes


class VideostreamNvencFrameHeader
{
public:
    VideostreamNvencFrameHeader() { std::fill(rep_, rep_ + sizeof(rep_), 0); }

    bool is_valid() const { return decode16(0) == FRAME_HEADER_ID;}
    unsigned short seq_id() const { return decode16(2); }
    unsigned long long timestamp() const { return decode64(4); }
    unsigned short width() const { return decode16(12); }
    unsigned short height() const { return decode16(14); }
    unsigned short codec() const { return decode16(16); }
    unsigned short format() const { return decode16(18); }
    unsigned int framesize() const { return decode32(20); }

    void mark_valid() { encode16(0, FRAME_HEADER_ID);}
    void mark_invalid() { encode16(0, 0);}
    void seq_id(unsigned short n) { encode16(2, n); }
    void timestamp(unsigned long long n) { encode64(4, n); }
    void width(unsigned short n) { encode16(12, n); }
    void height(unsigned short n) { encode16(14, n); }
    void codec(unsigned short n) { encode16(16, n); }
    void format(unsigned short n) { encode16(118, n); }
    void framesize(unsigned int n) { encode32(20, n); }

    void to_string()
    {
        unsigned int i;
        printf("{");
        for (i = 0; i < sizeof(rep_); i++) {
            printf(i == sizeof(rep_) - 1 ? "%.2X}\n" : "%.2X, ", rep_[i]);
        }
    }

    friend std::istream& operator>>(std::istream& is, VideostreamNvencFrameHeader& header)
    { return is.read(reinterpret_cast<char*>(header.rep_), 24); }

    friend std::ostream& operator<<(std::ostream& os, const VideostreamNvencFrameHeader& header)
    { return os.write(reinterpret_cast<const char*>(header.rep_), 24); }

    size_t size() { return sizeof(rep_); }

    bool copy_to(std::vector<uint8_t>& buffer) {
        if (buffer.size() < size()) {
            return false;
        }
        std::copy(&rep_[0], &rep_[0] + sizeof(rep_), buffer.begin());
        return true;
    }

    bool copy_from(const std::vector<uint8_t>& buffer) {
        if (buffer.size() < size()) {
            return false;
        }
        std::copy(buffer.begin(), buffer.begin() + sizeof(rep_), &rep_[0] );
        return true;
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
        n = (  ((unsigned long long)(rep_[i+0]) << 56) & 0xFF00000000000000U)
            | (((unsigned long long)(rep_[i+1]) << 48) & 0x00FF000000000000U)
            | (((unsigned long long)(rep_[i+2]) << 40) & 0x0000FF0000000000U)
            | (((unsigned long long)(rep_[i+3]) << 32) & 0x000000FF00000000U)
            | (((unsigned long long)(rep_[i+4]) << 24) & 0x00000000FF000000U)
            | (((unsigned long long)(rep_[i+5]) << 16) & 0x0000000000FF0000U)
            | (((unsigned long long)(rep_[i+6]) <<  8) & 0x000000000000FF00U)
            | ( (unsigned long long)(rep_[i+7])        & 0x00000000000000FFU);

        return n;
    }

    void encode16(int i, unsigned short num) {
        for(int j = 0; j < 2; j++) rep_[i+j] = num >> (2-1-j)*8;
    }

    void encode32(int i, unsigned int num) {
        for(int j = 0; j < 4; j++) rep_[i+j] = num >> (4-1-j)*8;
    }

    void encode64(int i, unsigned long long num) {
        for(int j = 0; j < 8; j++) rep_[i+j] = num >> (8-1-j)*8;
    }

    unsigned char rep_[24];
};





class VideostreamNvencPacketHeader
{
public:
    VideostreamNvencPacketHeader() { std::fill(rep_, rep_ + sizeof(rep_), 0); }

    bool is_valid() const { return decode16(0) == PACKET_HEADER_ID;}
    unsigned short seq_id() const { return decode16(2); }

    void mark_valid() { encode16(0, PACKET_HEADER_ID);}
    void mark_invalid() { encode16(0, 0);}
    void seq_id(unsigned short n) { encode16(2, n); }

    void to_string()
    {
        unsigned int i;
        printf("{");
        for (i = 0; i < sizeof(rep_); i++) {
            printf(i == sizeof(rep_) - 1 ? "%.2X}\n" : "%.2X, ", rep_[i]);
        }
    }

    friend std::istream& operator>>(std::istream& is, VideostreamNvencPacketHeader& header)
    { return is.read(reinterpret_cast<char*>(header.rep_), 4); }

    friend std::ostream& operator<<(std::ostream& os, const VideostreamNvencPacketHeader& header)
    { return os.write(reinterpret_cast<const char*>(header.rep_), 4); }

    size_t size() { return sizeof(rep_); }

    bool copy_to(std::vector<uint8_t>& buffer) {
        if (buffer.size() < size()) {
            return false;
        }
        std::copy(&rep_[0], &rep_[0] + sizeof(rep_), buffer.begin());
        return true;
    }

    bool copy_from(const std::vector<uint8_t>& buffer) {
        if (buffer.size() < size()) {
            return false;
        }
        std::copy(buffer.begin(), buffer.begin() + sizeof(rep_), &rep_[0] );
        return true;
    }

private:
    unsigned short decode16(int i) const
    {
        unsigned short n = 0;
        n =   ((rep_[i+0] <<  8) & 0x000000000000FF00U)
              |( rep_[i+1]       & 0x00000000000000FFU);

        return n;
    }

    void encode16(int i, unsigned short num) {
        for(int j = 0; j < 2; j++) rep_[i+j] = num >> (2-1-j)*8;
    }

    unsigned char rep_[4];
};

#endif //UBITRACK_DEVICE_COMM_VIDEOSTREAM_VIDEOSTREAMNVENCPROTOCOL_H
