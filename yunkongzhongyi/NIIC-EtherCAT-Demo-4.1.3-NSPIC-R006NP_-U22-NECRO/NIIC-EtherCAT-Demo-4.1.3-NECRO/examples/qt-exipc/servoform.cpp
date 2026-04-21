#include "servoform.h"
#include "ui_servoform.h"

ServoForm::ServoForm(ServoInfo_t *pInfo,QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ServoForm)
{
    ui->setupUi(this);
    memcpy(&servoInfo,pInfo,sizeof(ServoInfo_t));
    ui->lineEditPID->setText(QString("%1").arg(servoInfo.PID));
    ui->lineEditVID->setText(QString("%1").arg(servoInfo.VID));
    servoCtrl.axisCount = pInfo->axisCount;
    ui->hsPos->setEnabled(false);
    servoCtrl.targetPosition = 0;
}

ServoForm::~ServoForm()
{
    delete ui;
}

void ServoForm::UpdateServoSta(ServoSta_t *pSta)
{
    memcpy(&servoSta,pSta,sizeof(ServoSta_t));
    ui->lineEditCurPos->setText(QString("%1").arg(servoSta.curPos));
    ui->lineEditCurVel->setText(QString("%1").arg(servoSta.curVel));
    ui->lineEditCurStatus->setText(QString("0x%1").arg(servoSta.statusWord,4,16));
    ui->lineEditCurError->setText(QString("0x%1").arg(servoSta.errorCode,4,16));
    ui->lineEditCurMode->setText(QString("%1").arg(servoSta.curMode));
}


void ServoForm::on_pushButtonStartStop_clicked()
{
    started = !started;
    if(started)
    {
        ui->pushButtonStartStop->setText("下电");

        ui->hsPos->setEnabled(true);
        servoCtrl.targetPosition = ui->hsPos->value()*1000;
        servoCtrl.powerON = 1;
        servoCtrl.targetVelocity = 100;
        emit signalServoCtrl(&servoCtrl);
    }
    else
    {
        ui->pushButtonStartStop->setText("上电");
        ui->hsPos->setEnabled(false);
        servoCtrl.powerON = 0;
        servoCtrl.targetVelocity = 100;
        emit signalServoCtrl(&servoCtrl);
    }
}

void ServoForm::on_hsPos_valueChanged(int value)
{
    if(started)
    {
        servoCtrl.targetVelocity = 100; 
        servoCtrl.targetPosition = value * 1000;
        emit signalServoCtrl(&servoCtrl);
    }
    ui->hsPosDisp->setText(QString("%1 r").arg(value * 1000));
}

void ServoForm::on_testStartStop_clicked()
{
    if(started)
    {
        tested = !tested;
        if(tested)
        {
            ui->testStartStop->setText("测试结束");
            ui->hsPos->setEnabled(false);
            servoCtrl.powerON = 1;
            ui->testStartStop->setEnabled(true);
            servoCtrl.targetVelocity = 10086;       // 作为测试信号
            emit signalServoCtrl(&servoCtrl);
        }
        else
        {
            ui->testStartStop->setText("测试开始");
            ui->hsPos->setEnabled(false);
            ui->testStartStop->setEnabled(true);
            servoCtrl.powerON = 0;
            servoCtrl.targetVelocity = 10086;       // 作为测试信号
            emit signalServoCtrl(&servoCtrl);
        }
    }
}
