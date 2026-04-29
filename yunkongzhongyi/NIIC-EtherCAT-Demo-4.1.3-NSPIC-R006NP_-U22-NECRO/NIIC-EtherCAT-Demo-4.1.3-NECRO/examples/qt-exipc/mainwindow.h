#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QListWidgetItem>
#include <QProcess>
#include "servoform.h"
#include "receivethread.h"
#include "sendthread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_listWidget_itemClicked(QListWidgetItem *item);
    void on_ServoSta(ServoGroupSta_t *servoSta);
    void on_ServoInfo(ServoGroupInfo_t *servoInfo);

    void on_pushStartStop_clicked();

    void deleteAllSlot();

    void on_ctrlProcessStarted();
    void on_ctrlProcessFinished();
    void on_ctrlProcessErrorOccurred(QProcess::ProcessError error);

private:
    void CreateTableItems(int index, ServoInfo_t *pServo);

private:
    Ui::MainWindow *ui;
    QProcess *ethercatProcess;
    ReceiveThread *th;
    SendThread *se;
    QList<ServoForm *> listServoForm;
    int slotCounts;

};
#endif // MAINWINDOW_H
