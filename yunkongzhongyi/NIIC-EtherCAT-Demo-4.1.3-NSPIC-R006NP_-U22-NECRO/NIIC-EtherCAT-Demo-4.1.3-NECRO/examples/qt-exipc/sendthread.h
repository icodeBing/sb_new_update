#ifndef SENDTHREAD_H
#define SENDTHREAD_H
 
#include <QObject>
#include <QThread>
#include <QList>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "ServoIpcStruct.h"

 
class SendThread : public QThread
{
    Q_OBJECT //使用信号与槽函数
public:
    explicit SendThread(QObject *parent = nullptr);
    ~SendThread();
    //实现run接口
    void run();

    void Start();
    void Stop(void);
//声明信号
signals:
    void sendto(char *);
public slots:
    void slotServoData(ServoControl_t *servoReq);
private:
    bool m_Stop = false;
    int mExipcFd = -1;
    // uint8_t write_buf[1024];
    QList<IpcEvent_t *> save_buf;
};
 
#endif // SENDTHREAD_H