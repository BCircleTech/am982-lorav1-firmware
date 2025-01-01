#include "protocol.h"

void Protocol::Receive(const uint8_t *data, const uint32_t &len)
{
    mBuffer.append((char *)data, len);
    unsigned int bufferLen = mBuffer.length();
    unsigned int loop = bufferLen - 1;
    if (bufferLen > 1 && mState == 0)
    {
        for (unsigned int i = 0; i < loop; i++)
        {
            if (mBuffer[i] == mHeaders[0] && mBuffer[i + 1] == mHeaders[1])
            {
                mBuffer.erase(0, i);
                mState = 1;
                break;
            }
        }
        if (mState == 0)
        {
            mBuffer.erase(0, loop);
        }
    }
    if (mState == 1)
    {
        while (mTrailerBegin < loop)
        {
            if (mBuffer[mTrailerBegin] == mTrailers[0] && mBuffer[mTrailerBegin + 1] == mTrailers[1])
            {
                uint8_t checksum = 0;
                unsigned int checksumIndex = mTrailerBegin - 1;
                for (unsigned int i = 2; i < checksumIndex; i++)
                {
                    checksum += mBuffer[i];
                }
                if (mBuffer[checksumIndex] == checksum)
                {
                    mSegments.push_front(std::string(mBuffer, 2, checksumIndex - 2));
                }
                mBuffer.erase(0, mTrailerBegin + 2);
                mState = 0;
                mTrailerBegin = 5;
                break;
            }
            else
            {
                mTrailerBegin++;
            }
        }
    }
}

bool Protocol::GetOne(std::string &str)
{
    if (!mSegments.empty())
    {
        str = mSegments.back();
        mSegments.pop_back();
        return true;
    }
    else
    {
        return false;
    }
}

void Protocol::Pack(std::string &str, const uint8_t *cmd, const std::string &data)
{
    uint8_t checksum;
    str.resize(data.length() + 7);
    str[0] = mHeaders[0];
    str[1] = mHeaders[1];
    str[2] = cmd[0];
    str[3] = cmd[1];
    checksum = str[2] + str[3];
    unsigned int i = 4;
    for (const auto &it : data)
    {
        str[i++] = it;
        checksum += it;
    }
    str[data.length() + 4] = checksum;
    str[data.length() + 5] = mTrailers[0];
    str[data.length() + 6] = mTrailers[1];
}