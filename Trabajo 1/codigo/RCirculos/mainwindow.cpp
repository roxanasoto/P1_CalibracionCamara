#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    calibrator = new CameraCalibrator();
    calibrator->setVisualizer(this);
    on_rbCircle_clicked();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btnLoadVideo_clicked()
{
    QString pathImage = QFileDialog::getOpenFileName(this, tr("Search video"), "", tr("Video Files (*.avi *.mp4 *.wmv)"));

    if(!calibrator->loadVideo(pathImage.toStdString())) {
        ui->lblMsg->setText("Video is not found");
        ui->btnInitProc->setEnabled(false);
    }
    else {
        ui->lblMsg->setText("");
        ui->btnInitProc->setEnabled(true);
    }
}

void MainWindow::on_btnInitProc_clicked()
{
    ui->btnInitProc->setEnabled(false);

    calibrator->setSizePattern(ui->numRows->text().toInt(), ui->numCols->text().toInt());
    calibrator->setCurrentCalibrator(currCalibrator);
    calibrator->clearCalibrationInputs();
    calibrator->initProcessing(pattSelected);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    calibrator->setActived(false);
}

void MainWindow::visualizeMsg(std::string msg)
{
    ui->lblMsg->setText(QString::fromStdString(msg));
}

void MainWindow::visualizeValue(string label, double value)
{
    ui->lblMsg->setText(QString::fromStdString(label) + " = " + QString::number(value));
}

void MainWindow::visualizeImage(int id, QImage img, std::string title)
{
    QPixmap image = QPixmap::fromImage(img);
    switch (id) {
    case PROCFIN:
        ui->winFinalRes->setPixmap(image);
        if(title == "") title = "RESULTADO FINAL";
        ui->mainTitle->setText(QString::fromStdString(title));
        break;
    case PROC1:
        ui->winProc1->setPixmap(image);
        ui->subtitle1->setText(QString::fromStdString(title));
        break;
    case PROC2:
        ui->winProc2->setPixmap(image);
        ui->subtitle2->setText(QString::fromStdString(title));
        break;
    case PROC3:
        ui->winProc3->setPixmap(image);
        ui->subtitle3->setText(QString::fromStdString(title));
        break;
    case PROC5:
        ui->winProc5->setPixmap(image);
        ui->subtitle5->setText(QString::fromStdString(title));
        break;

    }
}

void MainWindow::cleanImage(int id)
{
    switch (id) {
    case PROCFIN:
        ui->winFinalRes->clear();
        break;
    case PROC1:
        ui->winProc1->clear();
        ui->subtitle1->setText("");
        break;
    case PROC2:
        ui->winProc2->clear();
        ui->subtitle2->setText("");
        break;
    case PROC3:
        ui->winProc3->clear();
        ui->subtitle3->setText("");
        break;
    case PROC5:
        ui->winProc5->clear();
        ui->subtitle5->setText("");
        break;
    }
}

void MainWindow::on_rbCircle_clicked()
{
    pattSelected = PATT_CIRCLE;
    ui->numRows->setText("4");
    ui->numCols->setText("11");
    //ui->centersDistance->setText("24.8");
    //ui->centersDistance->setText("12.4");
}

void MainWindow::on_rbRing_clicked()
{
    pattSelected = PATT_RING;
    ui->numRows->setText("4");
    ui->numCols->setText("5");
    //ui->centersDistance->setText("50");
}

// TIME & ACCURACY
void MainWindow::visualizeTimeExec(int totalFrames, int framesAna, double accu ,double timeValue){

    ui->leTFrames->setText(QString::number(totalFrames));
    ui->leAnalized->setText(QString::number(framesAna));
    ui->leAccu->setText(QString::number(accu));
    ui->timeExec->setText(QString::number(timeValue));

}
// tiempo real time
void MainWindow::visualizetimeReal(double timeframe)
{
    ui->leTimeframe->setText(QString::number(timeframe)+"ms");

}

void MainWindow::visualizaframesReal(int sumFrame ){

    ui->leAnalized->setText(QString::number(sumFrame));
//    ui->leTFrames->setText(QString::number(sumTotalFrames));
}

