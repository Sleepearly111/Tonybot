#include <Wire.h>
#include "HWSensor.h"
#include "../base_config.h"

HWSensor::HWSensor()
{
  Wire.begin(IO_SDA,IO_SCL);
}

//写字节
bool HWSensor::wireWriteByte(uint8_t addr, uint8_t val)
{
    Wire.beginTransmission(addr);
    Wire.write(val);
    if( Wire.endTransmission() != 0 )
    {
        return false;
    }
    return true;
}

//写多个字节
bool HWSensor::wireWriteDataArray(uint8_t addr, uint8_t reg,uint8_t *val,unsigned int len)
{
    unsigned int i;

    Wire.beginTransmission(addr);
    Wire.write(reg);
    for(i = 0; i < len; i++)
    {
        Wire.write(val[i]);
    }
    if( Wire.endTransmission() != 0 )
    {
        return false;
    }
    return true;
}

//读指定长度字节
int HWSensor::wireReadDataArray(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len)
{
    unsigned char i = 0;
    /* Indicate which register we want to read from */
    if (!wireWriteByte(addr, reg))
    {
        return -1;
    }
    Wire.requestFrom(addr, len);
    while (Wire.available())
    {
        if (i >= len)
        {
            return -1;
        }
        val[i] = Wire.read();
        i++;
    }
    /* Read block data */
    return i;
}

//参数1表示文本控制标记，h[0]表示自动判断单词发音方式
//v[10]表示发育音量，更多的标记符可以查看数据手册，搜索"文本控制标记列表"
//参数2表示要播放的内容，支持中英文
bool HWSensor::ttsSpeak(unsigned char *sign,unsigned char *words)
{
  unsigned short len,i;
  len = strlen((const char*)sign) + strlen((const char*)words) + 2;
  unsigned char head[] = {0xFD,0x00,0x02,0x01,0x01};   //发送数据文本编码为GB2312
  head[1] = len>>8;
  head[2] = len;

  Wire.beginTransmission(TTS_MODULE_I2C_ADDR);
  Wire.write(head,5);
  Wire.write(sign,strlen((const char*)sign));
  Wire.write(words,strlen((const char*)words));

  if( Wire.endTransmission() != 0 )
   {
        return false;
    }
    return true;
}

//获取识别结果
unsigned char HWSensor::asrGetResult(void)
{
  unsigned char result;

  wireReadDataArray(ASR_IIC_ADDR, ASR_RESULT_ADDR,&result,1);
  return result;
}

static uint8_t send[2];
void HWSensor::asr_speak(uint8_t cmd , uint8_t id)
{
  if(cmd == 0xFF || cmd == 0x00)
  {
    send[0] = cmd;
    send[1] = id;
    wireWriteDataArray(ASR_IIC_ADDR , ASR_SPEAK_ADDR , send , 2);
  }
}
