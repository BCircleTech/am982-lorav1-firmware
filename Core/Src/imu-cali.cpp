#include "imu-cali.h"

IMUCali::IMUCali()
{
    mKA[0][0] = 1;
    mKA[0][1] = 0;
    mKA[0][2] = 0;
    mKA[1][0] = 0;
    mKA[1][1] = 1;
    mKA[1][2] = 0;
    mKA[2][0] = 0;
    mKA[2][1] = 0;
    mKA[2][2] = 1;

    mBA[0] = 0;
    mBA[1] = 0;
    mBA[2] = 0;

    mKG[0][0] = 1;
    mKG[0][1] = 0;
    mKG[0][2] = 0;
    mKG[1][0] = 0;
    mKG[1][1] = 1;
    mKG[1][2] = 0;
    mKG[2][0] = 0;
    mKG[2][1] = 0;
    mKG[2][2] = 1;

    mBG[0] = 0;
    mBG[1] = 0;
    mBG[2] = 0;
}

void IMUCali::Set(float ka[3][3], float ba[3], float kg[3][3], float bg[3])
{
    mKA[0][0] = ka[0][0];
    mKA[0][1] = ka[0][1];
    mKA[0][2] = ka[0][2];
    mKA[1][0] = ka[1][0];
    mKA[1][1] = ka[1][1];
    mKA[1][2] = ka[1][2];
    mKA[2][0] = ka[2][0];
    mKA[2][1] = ka[2][1];
    mKA[2][2] = ka[2][2];

    mBA[0] = ba[0];
    mBA[1] = ba[1];
    mBA[2] = ba[2];

    mKG[0][0] = kg[0][0];
    mKG[0][1] = kg[0][1];
    mKG[0][2] = kg[0][2];
    mKG[1][0] = kg[1][0];
    mKG[1][1] = kg[1][1];
    mKG[1][2] = kg[1][2];
    mKG[2][0] = kg[2][0];
    mKG[2][1] = kg[2][1];
    mKG[2][2] = kg[2][2];

    mBG[0] = bg[0];
    mBG[1] = bg[1];
    mBG[2] = bg[2];
}

void IMUCali::Cali(float input[6])
{
    mOutput[0] = mKA[0][0] * input[0] + mKA[0][1] * input[1] + mKA[0][2] * input[2] + mBA[0];
    mOutput[1] = mKA[1][0] * input[0] + mKA[1][1] * input[1] + mKA[1][2] * input[2] + mBA[1];
    mOutput[2] = mKA[2][0] * input[0] + mKA[2][1] * input[1] + mKA[2][2] * input[2] + mBA[2];

    mOutput[3] = mKG[0][0] * input[3] + mKG[0][1] * input[4] + mKG[0][2] * input[5] + mBG[0];
    mOutput[4] = mKG[1][0] * input[3] + mKG[1][1] * input[4] + mKG[1][2] * input[5] + mBG[1];
    mOutput[5] = mKG[2][0] * input[3] + mKG[2][1] * input[4] + mKG[2][2] * input[5] + mBG[2];

    input[0] = mOutput[0];
    input[1] = mOutput[1];
    input[2] = mOutput[2];
    input[3] = mOutput[3];
    input[4] = mOutput[4];
    input[5] = mOutput[5];
}