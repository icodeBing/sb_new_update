#include "sendthread.h"
#include <QTime>
#include <QDebug>
#define EXIPC_PORT_1 1

SendThread::SendThread(QObject *parent):QThread(parent){}
SendThread::~SendThread(){}

void SendThread::run()
{
    //线程任务
    char *devname;
	int ret;
    size_t len;

	if (asprintf(&devname, "/dev/rtp%d", EXIPC_PORT_1) < 0)
		perror("asprintf");

	mExipcFd = open(devname, O_RDWR);
	free(devname);
	if (mExipcFd < 0)
		perror("open");

	while(m_Stop == false) {
        if (save_buf.isEmpty()){
            usleep(50);
            continue;
        }
        len = sizeof(ServoControl_t) + sizeof(IpcEventID_t);
        IpcEvent_t *pIpcEvent = save_buf.at(0);

		ret = write(mExipcFd, pIpcEvent, len);
		if (ret <= 0)
			perror("write");
        save_buf.removeAt(0);
        delete pIpcEvent;
	}
    close(mExipcFd);
}

void SendThread::Stop()
{
    m_Stop = true;
}

void SendThread::Start()
{
    m_Stop = false;
    this->start();
}

void SendThread::slotServoData(ServoControl_t *servoReq)
{
    IpcEvent_t *pIpcEvent = new IpcEvent_t;

    memcpy(&pIpcEvent->cntl, servoReq, sizeof(ServoControl_t));
    
    pIpcEvent->ID = (IpcEventID_t)(SERVO_REQ | sizeof(ServoControl_t));

    save_buf.append(pIpcEvent);
    
}