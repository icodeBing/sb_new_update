#ifndef TESTTHREAD_H
#define TESTTHREAD_H
 
#include <QObject>
#include <QThread>
#include <QMutex>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ServoIpcStruct.h"

 
class ReceiveThread : public QThread
{
    Q_OBJECT //使用信号与槽函数
public:
    explicit ReceiveThread(QObject *parent = nullptr);
    ~ReceiveThread();
    //实现run接口
    void run();

    void Start();
    void Stop(void);
//声明信号
signals:
    void signalServoNotice(ServoNotice_t *servoNotic);
    // void signalServoACK(ServoAck_t *servoAck);
    void signalServoSta(ServoGroupSta_t *servoSta);
    void signalServoInfo(ServoGroupInfo_t *servoInfo);
private:
    bool m_Stop = false;
    int exipcFd = -1;
    QMutex m_mutex;
};
 
#endif // TESTTHREAD_H