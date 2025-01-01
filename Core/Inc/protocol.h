#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>
#include <list>
#include <string>

class Protocol
{
public:
    void Receive(const uint8_t *data, const uint32_t &len);
    bool GetOne(std::string &str);

public:
    static void Pack(std::string &str, const uint8_t *cmd, const std::string &data);

private:
    static constexpr uint8_t mHeaders[2] = {0xac, 0x53};
    static constexpr uint8_t mTrailers[2] = {0x35, 0xca};
    std::list<std::string> mSegments;
    std::string mBuffer;
    uint8_t mState = 0;
    unsigned int mTrailerBegin = 5;
};

#endif