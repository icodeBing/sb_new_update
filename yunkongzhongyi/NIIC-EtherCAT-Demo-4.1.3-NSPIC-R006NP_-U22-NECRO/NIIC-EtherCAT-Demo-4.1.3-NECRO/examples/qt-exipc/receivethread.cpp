#include "receivethread.h"
#include <QTime>
#include <QDebug>
#define EXIPC_PORT 0

ReceiveThread::ReceiveThread(QObject *parent):QThread(parent){}
ReceiveThread::~ReceiveThread(){}

void ReceiveThread::run()
{
    //线程任务
    char *devname;
	uint8_t *rxbuf;
    IpcEventID_t eventID;
    int32_t dataEvent;
    size_t dataLength;
	int ret = 0;

	if (asprintf(&devname, "/dev/rtp%d", EXIPC_PORT) < 0)
		perror("asprintf");
    // printf("devname: %s\n", devname);
	exipcFd = open(devname, O_RDWR);
	free(devname);
	if (exipcFd < 0)
        perror("open");
    qDebug() << "Opened device:" << devname;
    qDebug() << "Read" << dataLength << "bytes with event ID:" << eventID;
    bool isPrintf=false;
	while(m_Stop == false) {
        if (m_Stop == false)
		    ret = read(exipcFd, &dataEvent, sizeof(uint32_t));
        if (ret <= 0){
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                qDebug() << "No data available";
            } else {
                perror("Read error");
            }
            // if(!isPrintf)
            // {
            //     qDebug() << "Get the next message read error :" << ret;
            //     isPrintf = true;
            // }
            continue;
        }

		//fetch Header
        dataLength = dataEvent & IPC_LEN_MASK;
        eventID = (IpcEventID_t)(dataEvent & (~IPC_LEN_MASK));

        // read infomation
		rxbuf = (uint8_t *)calloc(1, dataLength);
        if (m_Stop == false)
		    ret = read(exipcFd, rxbuf, dataLength);
		if (ret <= 0){
			free(rxbuf);
            qDebug() << "read " << dataLength << "bytes infomation read error :" << ret;
            continue;
		}
        
		switch (eventID) {
        case SERVO_NOTICE:
            emit signalServoNotice((ServoNotice_t *)rxbuf);
            free(rxbuf);
            break;
        case SERVO_GROUP_INFO:
            emit signalServoInfo((ServoGroupInfo_t *)rxbuf);
            free(rxbuf);
            break;
        case SERVO_GOURP_STA:
            emit signalServoSta((ServoGroupSta_t *)rxbuf);
            free(rxbuf);
            break;
        default:
            free(rxbuf);
            break;
        }
	}
    close(exipcFd);
}

void ReceiveThread::Stop()
{
    m_Stop = true;
}

void ReceiveThread::Start()
{
    m_Stop = false;
    this->start();
}