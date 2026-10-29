#include "tty.h"

Tty *Tty::p_instance = nullptr;

Tty::Tty(void) {
}

Tty::~Tty(void) {
}

Tty *Tty::Instance() {
  if (nullptr == p_instance) 
    p_instance = new Tty();
  return p_instance;
}

void Tty::ReleaseInstance() { 
  if (p_instance) 
    delete p_instance; 
}

int Tty::OpenTty(std::string ttyName) {
  gFdSerial = open(ttyName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (-1 == gFdSerial) {
    AERROR << "Can't Open Serial Port: " << ttyName;
    return -1;
  }

  if (fcntl(gFdSerial, F_SETFL, 0)<0) {
    AERROR << "Fcntl failed!";
  }

  if (isatty(STDIN_FILENO) == 0) {  
    AERROR << "Standard input is not a terminal device!";
  }
  return gFdSerial;
}  

int Tty::SetTtyOpt()  
{  
  struct termios newtio, oldtio;
  if (tcgetattr(gFdSerial, &oldtio) != 0) {  
    AERROR << "Set Serial error!";
    return -1;
  }  
  bzero(&newtio, sizeof(newtio));  
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;
  newtio.c_cflag |= CS8;
  newtio.c_cflag &= ~PARENB;  //无校验  

  cfsetispeed(&newtio, B921600);
  cfsetospeed(&newtio, B921600);

  newtio.c_cflag &= ~CSTOPB;  //停止位为1
  // newtio.c_cflag |= CSTOPB;   //停止位为2
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  tcflush(gFdSerial, TCIFLUSH);
  if ((tcsetattr(gFdSerial, TCSANOW, &newtio)) != 0) {
    AERROR << "TTy com set error!";
    return -1;
  }
  return 0;
}  

int Tty::ReadDataTty(uint8_t *rcv_buf, uint32_t Len)  
{  
  int retval   = 0;  
  fd_set rfds;  
  int ret      = 0;
  uint32_t pos = 0;  
  struct timeval tv; 
  tv.tv_sec  = 0;  
  tv.tv_usec = 10*1000;

#if 1
  FD_ZERO(&rfds);  
  FD_SET(gFdSerial, &rfds);  
  retval = select(gFdSerial + 1, &rfds, NULL, NULL, &tv);  
  if (retval == -1) {  
    AERROR << "TTy com select error!";  
    return ret;  
  }  
  else if (retval) {  
    ret = read(gFdSerial, rcv_buf, Len);  
    if (-1 == ret) {  
      return -1;  
    } else if (ret > 0){
      return ret;
    }  else {  
      return -1;  
    }
  }
  else {  
    return -1;  
  }
#else
  pos = 0;  
  while (1) {  
    FD_ZERO(&rfds);  
    FD_SET(gFdSerial, &rfds);  
    retval = select(gFdSerial + 1, &rfds, NULL, NULL, &tv);  
    if (retval == -1) {  
      AERROR<<"TTy com select error!";  
      break;  
    }  
    else if (retval) {  
      ret = read(gFdSerial, rcv_buf + pos, 1);  
      if (-1 == ret) {  
        break;  
      }
      pos++;  
      if (Len <= pos) {  
        break;  
      }
    }
    else {  
      break;  
    }
  }
#endif  
  return pos;  
}  

int Tty::SendDataTty(uint8_t *send_buf, uint32_t Len) {  
  ssize_t ret;  

  ret = write(gFdSerial, send_buf, Len);  
  if (ret == -1) {  
    AERROR << "Write device error!";  
    return -1;  
  }
  return 1;  
}

void Tty::CloseTty(void) {
  close(gFdSerial);
}

void Tty::FlushTty(void) {
  tcflush(gFdSerial, TCIOFLUSH);//清掉串口缓存  
  fcntl(gFdSerial, F_SETFL, 0);
}

int Tty::Init(std::string ttyPort) {
  int fd = OpenTty(ttyPort);
  if (fd < 0) {  
    return -1;  
  }  

  if (SetTtyOpt() < 0) {  
    return -1;  
  }   

  FlushTty();//清掉串口缓存 
  return fd; 
}

void Tty::Deinit(void) {
  CloseTty();
}