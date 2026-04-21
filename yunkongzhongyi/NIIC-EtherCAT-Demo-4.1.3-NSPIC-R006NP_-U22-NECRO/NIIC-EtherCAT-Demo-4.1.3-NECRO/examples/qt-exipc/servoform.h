#ifndef SERVOFORM_H
#define SERVOFORM_H

#include <QWidget> 
#include "ServoIpcStruct.h"

namespace Ui {
class ServoForm;
}

class ServoForm : public QWidget
{
    Q_OBJECT

public:
    explicit ServoForm(ServoInfo_t *pInfo,QWidget *parent = nullptr);
    ~ServoForm();

private:
    Ui::ServoForm *ui;
    ServoInfo_t servoInfo;
    ServoSta_t servoSta;
    ServoControl_t servoCtrl;
    bool started = false;
    bool tested = false;

public:
    void UpdateServoSta(ServoSta_t *pSta);

signals:
    void signalServoCtrl(ServoControl_t *servoCtrl);

private slots:
    void on_pushButtonStartStop_clicked();
    void on_hsPos_valueChanged(int value);
    void on_testStartStop_clicked();
};

#endif // SERVOFORM_H
