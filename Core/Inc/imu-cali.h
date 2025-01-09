#ifndef IMU_CALI_H
#define IMU_CALI_H

class IMUCali
{
public:
    IMUCali();

    void Set(float ka[3][3], float ba[3], float kg[3][3], float bg[3]);
    void Cali(float input[6]);

private:
    float mKA[3][3];
    float mBA[3];
    float mKG[3][3];
    float mBG[3];
    float mOutput[6];
};

#endif