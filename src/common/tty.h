#pragma once
#include "common.h"

class Tty {

  public:
    static Tty *Instance(void);
    static void ReleaseInstance(void);
    Tty(void);
    ~Tty(void);

    /**
     * @brief  读取串口数据
     * @note   
     * @param  *rcv_buf: 输出读取到的串口数据
     * @param  Len: 读取串口数据最大长度
     * @retval 实际读取串口数据的长度,失败返回-1
     */  
    int ReadDataTty(uint8_t *rcv_buf, uint32_t Len);

    /**
     * @brief  发送串口数据
     * @note   
     * @param  *send_buf:  发送的串口数据
     * @param  Len: 数据长度
     * @retval 成功返回1，失败返回-1
     */  
    int SendDataTty(uint8_t *send_buf, uint32_t Len);

    /**
     * @brief  初始化
     * @note   
     * @retval 成功返回1，失败返回0
     */
    int Init(std::string ttyPort);

    /**
     * @brief  关闭串口
     * @note   
     * @retval None
     */
    void Deinit(void);
    
  private:
    static Tty *p_instance;
    int gFdSerial = 0;
        /**
     * @brief  打开串口设备
     * @note   
     * @param  *ttyName: 设备文件名
     * @retval 成功返回文件描述符，失败返回-1
     */
    int OpenTty(std::string ttyName);
    /**
     * @brief  关闭串口设备
     * @note   
     * @retval None
     */
    void CloseTty(void);
    /**
     * @brief  配置串口设备
     * @note   
     * @retval 成功返回0，失败返回-1；
     */  
    int SetTtyOpt() ;
    /**
     * @brief  清空串口缓冲区
     * @note   
     * @retval None
     */
    void FlushTty(void);
};