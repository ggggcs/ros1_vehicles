#ifndef _JPEG_ENCODE_API_H
#define _JPEG_ENCODE_API_H

#if defined(_WIN32) || defined(_WIN64)
#ifndef CALLBACK
#define CALLBACK __stdcall
#endif
#ifdef  JPEGENC_EXPORTS
#define JPEGENC_API extern "C" __declspec(dllexport)
#else
#define JPEGENC_API extern "C" __declspec(dllimport)
#endif
#elif defined (__linux__)
#define JPEGENC_API extern "C"
#ifndef CALLBACK
#define CALLBACK 
#endif
#else 
#define JPEGENC_API
#ifndef CALLBACK
#define CALLBACK 
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
 
/*
data:编码后的数据
datalen:编码后的数据长度
userdata：用户数据
*/
typedef  void (CALLBACK *JPEGENC_CALLBACK)(unsigned char *data,int datalen,void *userdata);

typedef enum{
        JPEGENC_MODE_MEMORY = 0,      
        JPEGENC_MODE_FD=1,
        JPEGENC_MODE_TZFD=2,
}EJPEGENC_MODE;


typedef struct tagJPEGENC_PARA
{
        unsigned char pixfmt;           //像素格式0:YUV420P,  1:YUYV，其它格式暂不支持
        unsigned char userbuf;        //是否使用缓存
        unsigned char res[2];
        int bufsize;                                    //缓存大小
        int width;                                      //图像宽度
        int height;                                      //图像高度
        int quality;                                   //压缩质量，建议取值80-90之间
        int mode;                                      //0:cpu,1:gpu,2,hw
        int gpuid;                                      //gpu id  

        char res2[128];
}JPEGENC_PARA;

/*
        function:初始化
        return:成功0，失败-1
*/
JPEGENC_API int CALLBACK JPEGENC_Init();

/*
        function:反初始化
        return:成功0，失败-1
*/

JPEGENC_API int CALLBACK JPEGENC_Release();

/*
        function:创建句柄
        para：编码参数
        return:成功返回句柄，失败返回NULL
*/
JPEGENC_API void * CALLBACK JPEGENC_CreateHandle(JPEGENC_PARA *para);

/*
        function:销毁句柄
        handle：句柄
       return:成功0，失败-1
*/
JPEGENC_API int  CALLBACK JPEGENC_DestroyHandle(void *handle);

/*
        function:设置数据回调
        handle：句柄
        callback：回调函数，接收编码后的数据
        userdata：用户数据
       return:成功0，失败-1
*/
JPEGENC_API int CALLBACK JPEGENC_SetDataCallBack(void *handle,JPEGENC_CALLBACK callback,void *userdata);

/*
        function:塞入数据
        handle：句柄
        data：数据
        datalen：数据长度
       return:成功0，失败-1
*/
JPEGENC_API int CALLBACK JPEGENC_InputData(void *handle,unsigned char *data,int datalen);

#ifdef  __cplusplus
}
#endif

#endif
