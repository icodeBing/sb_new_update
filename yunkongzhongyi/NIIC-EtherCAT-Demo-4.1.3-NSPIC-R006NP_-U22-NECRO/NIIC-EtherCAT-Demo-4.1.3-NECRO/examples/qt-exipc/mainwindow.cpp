#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "QDebug"

#include <QProcess>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    th = new ReceiveThread();
    se = new SendThread();

    qRegisterMetaType<ServoControl_t *>("ServoControl_t *");
    qRegisterMetaType<ServoGroupSta_t *>("ServoGroupSta_t *");
    qRegisterMetaType<ServoGroupInfo_t *>("ServoGroupInfo_t *");
}

MainWindow::~MainWindow()
{
    delete ui;
}

static int ItemGetServoInfo(QListWidgetItem *item)
{
    QVariant variant = item->data(Qt::UserRole);
    if(variant.isValid()) return variant.value<int>();
    else    return -1;
}

void MainWindow::on_listWidget_itemClicked(QListWidgetItem *item)
{
    int index = ItemGetServoInfo(item);
    ui->tabWidget->setCurrentWidget(listServoForm[index]);
}

void MainWindow::CreateTableItems(int index,ServoInfo_t* pServo)
{
    listServoForm.append(new ServoForm(pServo));
    ui->tabWidget->setAttribute(Qt::WA_DeleteOnClose);
    ui->tabWidget->addTab(listServoForm[index],pServo->name);
    QListWidgetItem * IconItem = new QListWidgetItem(pServo->name);
    ui->listWidget->addItem(IconItem);
    IconItem->setData(Qt::UserRole,QVariant::fromValue(index));

    connect(listServoForm[index],SIGNAL(signalServoCtrl(ServoControl_t *)),se,SLOT(slotServoData(ServoControl_t *)));
}

void MainWindow::on_ServoSta(ServoGroupSta_t *servoSta)
{
    for(int index = 0 ; index < servoSta->slotCount;index++)
    {
        listServoForm[index]->UpdateServoSta(&servoSta->sta[index]);
    }
}

void MainWindow::on_ServoInfo(ServoGroupInfo_t *servoInfo)
{
    qDeleteAll(listServoForm);
    listServoForm.clear();
    slotCounts = servoInfo->slotCount;
    for(int index = 0 ; index < servoInfo->slotCount;index++)
    {
        CreateTableItems(index,&servoInfo->info[index]);
    }
}

void MainWindow::on_pushStartStop_clicked()
{
    if(ui->pushStartStop->text() == "启动")
    {
        printf("push start\n");
        connect(th,SIGNAL(signalServoSta(ServoGroupSta_t *)),this,SLOT(on_ServoSta(ServoGroupSta_t *)),Qt::BlockingQueuedConnection);
        connect(th,SIGNAL(signalServoInfo(ServoGroupInfo_t *)),this,SLOT(on_ServoInfo(ServoGroupInfo_t *)),Qt::BlockingQueuedConnection);
        ui->pushStartStop->setEnabled(false);
        QStringList arg;
        arg << "-f" << ui->comboBoxEni->currentText();
        qDebug() << arg.at(0);
        qDebug() << ui->comboBoxCtrlProg->currentText().toLocal8Bit();
        ethercatProcess = new QProcess(this);
        connect(ethercatProcess,SIGNAL(started()),this,SLOT(on_ctrlProcessStarted()));
        connect(ethercatProcess,SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(on_ctrlProcessFinished()));
        connect(ethercatProcess,SIGNAL(errorOccurred(QProcess::ProcessError)),this,SLOT(on_ctrlProcessErrorOccurred(QProcess::ProcessError)));
        ethercatProcess->start(ui->comboBoxCtrlProg->currentText().toLocal8Bit(), arg);
        ui->pushStartStop->setText("停止");
        th->Start();
        printf("receive thread start\n");
        se->Start();
        printf("send thread start\n");
    }
    else
    {
        printf("push stop\n");
        th->disconnect();
        th->Stop();

        for (int d = 0; d < slotCounts; d++){
            listServoForm[d]->disconnect(se);
        }
        se->Stop();
        
        if (ethercatProcess)
            ethercatProcess->close();
        // usleep(1000);
        ethercatProcess->waitForFinished(-1);
        delete ethercatProcess;
        ethercatProcess = 0;
        
        ui->pushStartStop->setText("启动");

        deleteAllSlot();
    }
}

void MainWindow::deleteAllSlot()
{
    int icount = ui->tabWidget->count();
    for (int i = 0; i < icount; i++){
        ui->tabWidget->removeTab(0);
    }

    int counter = ui->listWidget->count();
    for(int index = 0; index < counter; index++)
    {
        QListWidgetItem *item = ui->listWidget->takeItem(0);
        ui->listWidget->removeItemWidget(item);
        delete item;
    }
    
    ui->listWidget->clear();
    ui->tabWidget->clear();
}

void MainWindow::on_ctrlProcessStarted()
{
    ui->pushStartStop->setEnabled(true);
    ui->pushStartStop->setText("停止");
}

void MainWindow::on_ctrlProcessFinished()
{
    ui->pushStartStop->setEnabled(true);
    ui->pushStartStop->setText("启动");
}

void MainWindow::on_ctrlProcessErrorOccurred(QProcess::ProcessError error)
{
    // qDebug() << "ProcessError" << error;
    ui->pushStartStop->setEnabled(true);
    ui->pushStartStop->setText("启动");
}
